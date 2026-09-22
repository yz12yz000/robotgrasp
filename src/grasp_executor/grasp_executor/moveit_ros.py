"""MoveIt OMPL + Cartesian paths, with separate planning and execution.

Uses Humble ROS services/actions directly; no moveit_commander or Pilz dependency.
All blocking methods run in the task worker while the ROS executor keeps spinning.
"""

from copy import deepcopy
from dataclasses import dataclass
import fcntl
import math
import os
from pathlib import Path
from threading import Event, RLock
import time
import xml.etree.ElementTree as ET

from .cartesian import pose_error, tool_target_to_tip
from .grasp import Pose, check_workspace
from .trajectory import samples, seconds, slow_down, validate_trajectory


JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")


@dataclass
class Segment:
    name: str
    trajectory: object
    start: object
    goal: Pose
    linear: bool
    checked_positions: list


class MoveItArm:
    def __init__(self, node, config, cancel):
        from geometry_msgs.msg import Pose as RosPose
        from moveit_msgs.action import ExecuteTrajectory
        from moveit_msgs.msg import DisplayTrajectory
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from tf2_ros import Buffer, TransformListener

        self.node, self.config, self.cancel = node, config, cancel
        self.lock = RLock()
        self.stopped = Event()
        self.group = ReentrantCallbackGroup()
        self.clients = {}
        self.RosPose = RosPose
        self.ExecuteTrajectory = ExecuteTrajectory
        self.action = ActionClient(node, ExecuteTrajectory, "/execute_trajectory", callback_group=self.group)
        self.buffer = Buffer(node=node)
        self.listener = TransformListener(self.buffer, node)
        self.joint_state = None
        self.joint_received = 0.0
        self.subscription = node.create_subscription(JointState, "/joint_states", self._joint, qos_profile_sensor_data)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.display = node.create_publisher(DisplayTrajectory, "/display_planned_path", retained)
        self.program = None
        self.safety = None
        self.robot_mode = None
        if not config.plan_only:
            from std_msgs.msg import Bool
            from ur_dashboard_msgs.msg import RobotMode, SafetyMode
            self.program_sub = node.create_subscription(Bool, "/io_and_status_controller/robot_program_running",
                                                       lambda m: setattr(self, "program", m.data), retained)
            self.safety_sub = node.create_subscription(SafetyMode, "/io_and_status_controller/safety_mode",
                                                      lambda m: setattr(self, "safety", m.mode), retained)
            self.mode_sub = node.create_subscription(RobotMode, "/io_and_status_controller/robot_mode",
                                                    lambda m: setattr(self, "robot_mode", m.mode), retained)
        self.plans = []
        self.next_segment = 0
        self.active = None
        self.goal_future = None
        self.goal_handle = None
        self.result_future = None
        self.action_error = None
        self.commanded = False
        self.settle_started = None
        self.mode_lock = None

    def _joint(self, message):
        with self.lock:
            self.joint_state = message
            self.joint_received = time.monotonic()

    def _check_cancel(self):
        if self.cancel.is_set() or self.stopped.is_set():
            raise RuntimeError("cancelled")

    def _wait_future(self, future, deadline, ignore_cancel=False):
        while not future.done():
            if not ignore_cancel:
                self._check_cancel()
            if time.monotonic() >= deadline:
                raise TimeoutError("moveit_response_timeout")
            time.sleep(0.01)
        if not ignore_cancel:
            self._check_cancel()
        result = future.result()
        if result is None:
            raise RuntimeError("moveit_empty_response")
        return result

    def _call(self, service_type, name, request, deadline, ignore_cancel=False):
        key = (service_type, name)
        if key not in self.clients:
            self.clients[key] = self.node.create_client(service_type, name, callback_group=self.group)
        client = self.clients[key]
        while not client.service_is_ready():
            if not ignore_cancel:
                self._check_cancel()
            if time.monotonic() >= deadline:
                raise TimeoutError("moveit_service_unavailable:" + name)
            time.sleep(0.02)
        future = client.call_async(request)
        try:
            return self._wait_future(future, deadline, ignore_cancel)
        except Exception:
            future.cancel()  # Services only; action acceptance futures must remain alive.
            raise

    def _parameters(self, names, deadline):
        from rcl_interfaces.srv import GetParameters
        from rclpy.parameter import parameter_value_to_python
        result = self._call(GetParameters, self.config.move_group_node.rstrip('/') + "/get_parameters",
                            GetParameters.Request(names=names), deadline)
        return dict(zip(names, (parameter_value_to_python(v) for v in result.values)))

    def _reserve_mode(self):
        if self.mode_lock is not None:
            return
        domain = os.environ.get("ROS_DOMAIN_ID", "0")
        if not domain.isdigit():
            raise ValueError("invalid_ROS_DOMAIN_ID")
        root = Path(os.environ.get("XDG_RUNTIME_DIR", f"/tmp/robot-startup-{os.getuid()}")) / f"robot-grasp-{domain}"
        root.mkdir(parents=True, exist_ok=True)
        stream = (root / "arm-mode.lock").open('a')
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            stream.close()
            raise RuntimeError("arm_mode_in_use:stop_joystick_or_other_grasp_session")
        self.mode_lock = stream

    def _state(self, stationary=False):
        from moveit_msgs.msg import RobotState
        with self.lock:
            message, received = deepcopy(self.joint_state), self.joint_received
        if message is None or time.monotonic() - received > self.config.feedback_timeout:
            raise RuntimeError("stale_joint_feedback")
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = (self.node.get_clock().now().nanoseconds - stamp) / 1e9
        if stamp <= 0 or not -self.config.future_tolerance <= age <= self.config.feedback_timeout:
            raise RuntimeError("stale_joint_timestamp")
        if len(message.name) != len(set(message.name)) or len(message.position) != len(message.name):
            raise ValueError("invalid_joint_feedback")
        for name in JOINTS:
            if name not in message.name or not math.isfinite(message.position[message.name.index(name)]):
                raise ValueError("missing_or_invalid_joint:" + name)
        if stationary:
            if len(message.velocity) != len(message.name):
                raise ValueError("joint_velocity_feedback_missing")
            if any(not math.isfinite(message.velocity[message.name.index(j)]) or
                   abs(message.velocity[message.name.index(j)]) > 0.02 for j in JOINTS):
                raise RuntimeError("robot_not_stationary")
        state = RobotState()
        state.joint_state = message
        state.is_diff = True
        return state

    @staticmethod
    def _state_at(state, names, positions):
        result = deepcopy(state)
        values = dict(zip(result.joint_state.name, result.joint_state.position))
        values.update(zip(names, positions))
        result.joint_state.name = list(values)
        result.joint_state.position = list(values.values())
        result.joint_state.velocity = [0.0] * len(values)
        result.joint_state.effort = []
        result.joint_state.header.stamp.sec = 0
        result.joint_state.header.stamp.nanosec = 0
        return result

    @staticmethod
    def _joint_error(state, names, positions):
        actual = dict(zip(state.joint_state.name, state.joint_state.position))
        # Raw difference deliberately rejects a different 2*pi joint branch.
        return max(abs(actual[name] - value) for name, value in zip(names, positions))

    def read_pose(self):
        from rclpy.time import Time
        transform = self.buffer.lookup_transform(self.config.base_frame, self.config.tool_frame, Time())
        stamp = transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec
        age = (self.node.get_clock().now().nanoseconds - stamp) / 1e9
        if stamp <= 0 or not -self.config.future_tolerance <= age <= self.config.feedback_timeout:
            raise RuntimeError("stale_robot_tf_feedback")
        p, q = transform.transform.translation, transform.transform.rotation
        pose = Pose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w), self.config.base_frame, self.config.tool_frame)
        if not all(math.isfinite(v) for v in pose.position + pose.orientation):
            raise ValueError("invalid_robot_tf_feedback")
        check_workspace(pose, self.config)
        return pose

    def _robot_running(self):
        if self.config.plan_only:
            return
        if self.program is not True or self.robot_mode != 7 or self.safety not in (1, 2):
            raise RuntimeError("ur_external_control_or_safety_not_ready")

    def _controller_ready(self, deadline):
        from controller_manager_msgs.srv import ListControllers
        result = self._call(ListControllers, self.config.controller_manager.rstrip('/') + "/list_controllers",
                            ListControllers.Request(), deadline)
        target = next((c for c in result.controller if c.name == self.config.controller_name), None)
        if target is None or target.state != 'active' or target.type != 'ur_controllers/ScaledJointTrajectoryController':
            raise RuntimeError("trajectory_controller_not_active:run_run_grasp_position_control.sh_first")
        for controller in result.controller:
            if controller.name != self.config.controller_name and controller.state == 'active' and (
                    controller.name in ('forward_position_controller', 'forward_velocity_controller',
                                        'cartesian_motion_controller', 'joint_trajectory_controller',
                                        'passthrough_trajectory_controller', 'freedrive_mode_controller', 'force_mode_controller')
                    or any(v.split('/')[0] in JOINTS for v in controller.claimed_interfaces)):
                raise RuntimeError("conflicting_motion_controller:" + controller.name)

    def check_ready(self, timeout):
        from rclpy.time import Time
        self._reserve_mode()
        deadline = time.monotonic() + timeout
        self._check_cancel()
        nodes = self.node.get_node_names()
        if 'logitech_f710_servo_twist' in nodes or 'servo_node' in nodes:
            raise RuntimeError("stop_joystick_and_servo_before_grasp")
        if nodes.count('grasp_executor') > 1:
            raise RuntimeError("duplicate_grasp_executor")
        params = self._parameters(['robot_description', 'robot_description_semantic', 'use_sim_time'], deadline)
        if params['use_sim_time'] != self.node.get_parameter('use_sim_time').value:
            raise RuntimeError("moveit_clock_mismatch")
        model = ET.fromstring(params['robot_description'])
        self.model_id = model.attrib['name']
        semantic = ET.fromstring(params['robot_description_semantic'])
        chain = semantic.find(f"./group[@name='{self.config.move_group}']/chain")
        if chain is None or chain.get('tip_link') != self.config.planning_link or chain.get('base_link') != self.config.base_frame:
            raise RuntimeError("moveit_group_frame_mismatch")
        self.joint_limits = {}
        for name in JOINTS:
            limit = model.find(f"./joint[@name='{name}']/limit")
            if limit is None:
                raise RuntimeError("joint_limit_missing:" + name)
            low, high, velocity = (float(limit.get(k)) for k in ('lower', 'upper', 'velocity'))
            if not all(math.isfinite(v) for v in (low, high, velocity)) or low >= high or velocity <= 0:
                raise RuntimeError("invalid_joint_limit:" + name)
            self.joint_limits[name] = (low, high, min(velocity*self.config.velocity_scaling, self.config.max_joint_velocity))
        # Tool conversion must traverse fixed joints only.
        parents = {joint.find('child').get('link'): joint for joint in model.findall('joint')}
        child, visited = self.config.tool_frame, set()
        while child != self.config.planning_link:
            if child in visited or child not in parents or parents[child].get('type') != 'fixed':
                raise RuntimeError("tool_transform_must_be_fixed")
            visited.add(child)
            child = parents[child].find('parent').get('link')
        while True:
            self._check_cancel()
            try:
                state = self._state(stationary=True)
                actual = self.read_pose()
                transform = self.buffer.lookup_transform(self.config.planning_link, self.config.tool_frame, Time()).transform
                p, q = transform.translation, transform.rotation
                self.tip_to_tool = ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
                self._robot_running()
                break
            except Exception:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.02)
        fk = self._fk(state, deadline)
        if pose_error(actual, fk)[0] > self.config.position_tolerance or pose_error(actual, fk)[1] > self.config.orientation_tolerance:
            raise RuntimeError("moveit_model_and_tf_disagree")
        if not self.config.plan_only:
            self._controller_ready(deadline)
            mapping = self._parameters(['moveit_manage_controllers',
                'moveit_simple_controller_manager.controller_names',
                f'moveit_simple_controller_manager.{self.config.controller_name}.default'], deadline)
            if (mapping['moveit_manage_controllers'] is not False or
                    self.config.controller_name not in mapping['moveit_simple_controller_manager.controller_names'] or
                    mapping[f'moveit_simple_controller_manager.{self.config.controller_name}.default'] is not True):
                raise RuntimeError("moveit_execution_controller_mapping_mismatch")
            while not self.action.server_is_ready():
                self._check_cancel()
                if time.monotonic() >= deadline:
                    raise TimeoutError("execute_trajectory_unavailable")
                time.sleep(0.02)
            while not any(p.node_name == 'hand_control_node' for p in
                          self.node.get_subscriptions_info_by_topic('/handControlCmd')):
                self._check_cancel()
                if time.monotonic() >= deadline:
                    raise RuntimeError("gripper_driver_subscription_missing")
                time.sleep(0.02)
        return True

    def _ros_pose(self, pose):
        result = self.RosPose()
        result.position.x, result.position.y, result.position.z = pose.position
        result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = pose.orientation
        return result

    def _fk(self, state, deadline):
        from moveit_msgs.srv import GetPositionFK
        request = GetPositionFK.Request()
        request.header.frame_id = self.config.base_frame
        request.fk_link_names = [self.config.tool_frame]
        request.robot_state = state
        result = self._call(GetPositionFK, '/compute_fk', request, deadline)
        if result.error_code.val != 1 or len(result.pose_stamped) != 1:
            raise RuntimeError(f"fk_failed:{result.error_code.val}")
        p, q = result.pose_stamped[0].pose.position, result.pose_stamped[0].pose.orientation
        return Pose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w), self.config.base_frame, self.config.tool_frame)

    def _valid(self, state, deadline):
        from moveit_msgs.srv import GetStateValidity
        request = GetStateValidity.Request(robot_state=state, group_name=self.config.move_group)
        result = self._call(GetStateValidity, '/check_state_validity', request, deadline)
        if not result.valid:
            contacts = ','.join(c.contact_body_1 + '/' + c.contact_body_2 for c in result.contacts[:3])
            raise RuntimeError("collision_or_invalid_state:" + contacts)

    def _scene(self, deadline):
        from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
        from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
        from shape_msgs.msg import SolidPrimitive
        if self.config.table_enabled:
            obj = CollisionObject()
            obj.id, obj.header.frame_id, obj.operation = self.config.table_id, self.config.base_frame, CollisionObject.ADD
            primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[self.config.table_size[k] for k in 'xyz'])
            pose = self.RosPose()
            pose.position.x, pose.position.y, pose.position.z = (self.config.table_center[k] for k in 'xyz')
            pose.orientation.w = 1.0
            obj.primitives, obj.primitive_poses = [primitive], [pose]
            scene = PlanningScene(is_diff=True)
            scene.robot_state.is_diff = True
            scene.world.collision_objects = [obj]
            result = self._call(ApplyPlanningScene, '/apply_planning_scene', ApplyPlanningScene.Request(scene=scene), deadline)
            if not result.success:
                raise RuntimeError("table_scene_apply_failed")
        request = GetPlanningScene.Request()
        request.components.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES | PlanningSceneComponents.OCTOMAP)
        scene = self._call(GetPlanningScene, '/get_planning_scene', request, deadline).scene
        self.world_objects = [obj.id for obj in scene.world.collision_objects]
        if not self.world_objects and not scene.world.octomap.octomap.data:
            if not self.config.plan_only and self.config.require_collision_world_for_execution:
                raise RuntimeError("collision_world_empty:configure_table_or_existing_scene")
            self.node.get_logger().info("collision_world_empty: self-collision checks remain enabled; real tabletop is not modeled")

    def _pose_plan(self, start, goal, deadline):
        from moveit_msgs.msg import Constraints, JointConstraint
        from moveit_msgs.srv import GetMotionPlan, GetPositionIK
        target = tool_target_to_tip(goal, self.tip_to_tool, self.config.planning_link)
        # Seed IK from the actual start before asking OMPL to plan. Sampling a
        # free pose goal can choose a distant wrist/elbow branch even for a tiny
        # TCP displacement. The explicit joint goal keeps the local IK branch.
        ik = GetPositionIK.Request()
        ik.ik_request.group_name = self.config.move_group
        ik.ik_request.robot_state = start
        ik.ik_request.ik_link_name = self.config.planning_link
        ik.ik_request.pose_stamped.header.frame_id = self.config.base_frame
        ik.ik_request.pose_stamped.pose = self._ros_pose(target)
        ik.ik_request.avoid_collisions = True
        ik.ik_request.timeout.sec = max(1, math.ceil(self.config.allowed_planning_time))
        solved = self._call(GetPositionIK, '/compute_ik', ik, deadline)
        if solved.error_code.val != 1:
            raise RuntimeError(f"approach_ik_failed:{solved.error_code.val}")
        solution = dict(zip(solved.solution.joint_state.name, solved.solution.joint_state.position))
        if any(j not in solution or not math.isfinite(solution[j]) for j in JOINTS):
            raise RuntimeError("invalid_ik_solution")
        request = GetMotionPlan.Request()
        plan = request.motion_plan_request
        plan.group_name, plan.pipeline_id, plan.planner_id = self.config.move_group, self.config.planning_pipeline, self.config.planner_id
        plan.start_state = start
        plan.goal_constraints = [Constraints(joint_constraints=[JointConstraint(joint_name=j,
            position=solution[j], tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0) for j in JOINTS])]
        plan.num_planning_attempts, plan.allowed_planning_time = 3, self.config.allowed_planning_time
        plan.max_velocity_scaling_factor = self.config.velocity_scaling
        plan.max_acceleration_scaling_factor = self.config.acceleration_scaling
        plan.workspace_parameters.header.frame_id = self.config.base_frame
        for k in 'xyz':
            setattr(plan.workspace_parameters.min_corner, k, self.config.workspace_min[k])
            setattr(plan.workspace_parameters.max_corner, k, self.config.workspace_max[k])
        response = self._call(GetMotionPlan, '/plan_kinematic_path', request, deadline).motion_plan_response
        if response.error_code.val != 1:
            raise RuntimeError(f"approach_planning_failed:{response.error_code.val}")
        return response.trajectory

    def _linear_plan(self, start, goal, deadline):
        from moveit_msgs.srv import GetCartesianPath
        target = tool_target_to_tip(goal, self.tip_to_tool, self.config.planning_link)
        request = GetCartesianPath.Request()
        request.header.frame_id = self.config.base_frame
        request.start_state, request.group_name, request.link_name = start, self.config.move_group, self.config.planning_link
        request.waypoints = [self._ros_pose(target)]
        request.max_step, request.jump_threshold = self.config.cartesian_step, self.config.jump_threshold
        request.avoid_collisions = True
        result = self._call(GetCartesianPath, '/compute_cartesian_path', request, deadline)
        if result.error_code.val != 1 or not math.isfinite(result.fraction) or abs(result.fraction - 1.0) > 1e-9:
            raise RuntimeError(f"cartesian_path_incomplete:fraction={result.fraction},code={result.error_code.val}")
        return result.solution

    def _validate_and_time(self, name, trajectory, start, goal, linear, deadline):
        validate_trajectory(trajectory, JOINTS)
        joint = trajectory.joint_trajectory
        if self._joint_error(start, joint.joint_names, joint.points[0].positions) > self.config.start_joint_tolerance:
            raise RuntimeError("planner_changed_start_state")
        if linear and any(max(abs(x-y) for x, y in zip(a.positions, b.positions)) > self.config.max_joint_step
                          for a, b in zip(joint.points, joint.points[1:])):
            raise RuntimeError("cartesian_joint_jump")
        sample_list = samples(trajectory, self.config)
        factor, previous, initial_pose = 1.0, None, None
        positions = []
        for t, q, v, acc in sample_list:
            self._check_cancel()
            for j, value, velocity, acceleration in zip(joint.joint_names, q, v, acc):
                low, high, max_velocity = self.joint_limits[j]
                if not all(math.isfinite(x) for x in (value, velocity, acceleration)) or not low <= value <= high:
                    raise RuntimeError("trajectory_joint_limit:" + j)
                factor = max(factor, abs(velocity)/max_velocity,
                             math.sqrt(abs(acceleration)/self.config.max_joint_acceleration))
            state = self._state_at(start, joint.joint_names, q)
            self._valid(state, deadline)
            pose = self._fk(state, deadline)
            check_workspace(pose, self.config)
            if initial_pose is None:
                initial_pose = pose
            if linear:
                if (math.dist(pose.position[:2], goal.position[:2]) > self.config.linear_lateral_tolerance
                        or pose_error(pose, goal)[1] > self.config.orientation_tolerance
                        or not min(initial_pose.position[2], goal.position[2]) - self.config.position_tolerance <=
                        pose.position[2] <= max(initial_pose.position[2], goal.position[2]) + self.config.position_tolerance):
                    raise RuntimeError("cartesian_path_deviates_from_vertical")
            if previous is not None:
                old_t, old_pose = previous
                distance, angle = pose_error(pose, old_pose)
                speed = self.config.descend_speed if linear else self.config.approach_speed
                factor = max(factor, distance / ((t-old_t)*speed), angle / ((t-old_t)*self.config.angular_speed))
            previous = t, pose
            positions.append(q)
        if pose_error(previous[1], goal)[0] > self.config.position_tolerance or pose_error(previous[1], goal)[1] > self.config.orientation_tolerance:
            raise RuntimeError("planned_endpoint_mismatch")
        timed = slow_down(trajectory, max(1.0, factor * 1.05))
        if seconds(timed.joint_trajectory.points[-1].time_from_start) + self.config.settle_time >= self.config.motion_timeout:
            raise RuntimeError(f"motion_timeout_too_short_for_trajectory:{name}:"
                               f"duration={seconds(timed.joint_trajectory.points[-1].time_from_start):.2f}s,scale={factor:.2f}")
        return Segment(name, timed, deepcopy(start), goal, linear, positions)

    def prepare_plans(self, approach, grasp):
        from moveit_msgs.msg import DisplayTrajectory
        deadline = time.monotonic() + self.config.planning_timeout
        self._scene(deadline)
        start = self._state(stationary=True)
        original = deepcopy(start)
        self._valid(start, deadline)
        plans = []
        for name, goal, linear in (('approach', approach, False), ('descend', grasp, True), ('lift', approach, True)):
            self._check_cancel()
            deadline = time.monotonic() + self.config.planning_timeout
            trajectory = self._linear_plan(start, goal, deadline) if linear else self._pose_plan(start, goal, deadline)
            segment = self._validate_and_time(name, trajectory, start, goal, linear, deadline)
            plans.append(segment)
            joint = segment.trajectory.joint_trajectory
            start = self._state_at(start, joint.joint_names, joint.points[-1].positions)
        self._check_cancel()
        first = plans[0].trajectory.joint_trajectory
        if self._joint_error(self._state(stationary=True), first.joint_names, first.points[0].positions) > self.config.start_joint_tolerance:
            raise RuntimeError("robot_moved_during_planning")
        self.plans, self.next_segment = plans, 0
        display = DisplayTrajectory(model_id=self.model_id, trajectory_start=original,
                                    trajectory=[p.trajectory for p in plans])
        self.display.publish(display)
        summary = ', '.join(f'{p.name}={seconds(p.trajectory.joint_trajectory.points[-1].time_from_start):.2f}s' for p in plans)
        self.node.get_logger().info('complete_grasp_plan: ' + summary)
        return True

    def _begin(self, pose, timeout, linear):
        if self.config.plan_only:
            raise RuntimeError("plan_only_execution_forbidden")
        self._check_cancel()
        if self.next_segment >= len(self.plans):
            raise RuntimeError("no_prepared_segment")
        segment = self.plans[self.next_segment]
        if segment.linear != linear or pose != segment.goal:
            raise RuntimeError("prepared_segment_mismatch")
        deadline = time.monotonic() + min(timeout, self.config.planning_timeout)
        self._robot_running()
        self._controller_ready(deadline)
        joint = segment.trajectory.joint_trajectory
        current = self._state(stationary=True)
        if self._joint_error(current, joint.joint_names, joint.points[0].positions) > self.config.start_joint_tolerance:
            raise RuntimeError("trajectory_start_state_changed")
        if linear:
            if math.dist(self.read_pose().position[:2], pose.position[:2]) > self.config.position_tolerance:
                raise RuntimeError("linear_start_pose_changed")
        # Recheck sampled geometry against the latest planning scene before
        # every segment. Do not reuse collision results across gripper waits.
        for q in segment.checked_positions:
            self._valid(self._state_at(current, joint.joint_names, q), deadline)
        if self._joint_error(self._state(stationary=True), joint.joint_names, joint.points[0].positions) > self.config.start_joint_tolerance:
            raise RuntimeError("robot_moved_before_execution")
        with self.lock:
            self._check_cancel()
            self.active = segment
            self.goal_handle, self.result_future, self.action_error = None, None, None
            self.settle_started = None
            self.commanded = True
            self.goal_future = self.action.send_goal_async(self.ExecuteTrajectory.Goal(trajectory=segment.trajectory))
            self.goal_future.add_done_callback(self._goal_response)
            self.next_segment += 1
        return True

    def _goal_response(self, future):
        try:
            handle = future.result()
            with self.lock:
                self.goal_handle = handle
                if handle is None or not handle.accepted:
                    self.action_error = 'trajectory_goal_rejected'
                    return
                self.result_future = handle.get_result_async()
                if self.stopped.is_set() or self.cancel.is_set():
                    handle.cancel_goal_async()  # Includes late acceptance after a timeout.
        except Exception as exc:
            self.action_error = 'trajectory_goal_error:' + str(exc)

    def move_to_pose(self, pose, timeout):
        return self._begin(pose, timeout, False)

    def move_linear(self, pose, timeout):
        return self._begin(pose, timeout, True)

    def is_pose_reached(self, timeout):
        from action_msgs.msg import GoalStatus
        self._check_cancel()
        self._robot_running()
        self._state()
        actual = self.read_pose()
        with self.lock:
            if self.action_error:
                raise RuntimeError(self.action_error)
            segment, future = self.active, self.result_future
        if segment is None:
            raise RuntimeError("no_active_segment")
        if segment.linear and (math.dist(actual.position[:2], segment.goal.position[:2]) > self.config.linear_lateral_tolerance
                               or pose_error(actual, segment.goal)[1] > self.config.max_tracking_angle):
            raise RuntimeError("linear_tracking_error")
        if future is None or not future.done():
            return False
        result = future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code.val != 1:
            raise RuntimeError(f"trajectory_execution_failed:status={result.status},code={result.result.error_code.val}")
        distance, angle = pose_error(actual, segment.goal)
        if distance > self.config.position_tolerance or angle > self.config.orientation_tolerance:
            self.settle_started = None
            return False
        self._state(stationary=True)
        now = time.monotonic()
        self.settle_started = self.settle_started if self.settle_started is not None else now
        return now - self.settle_started >= self.config.settle_time

    def stop_motion(self, timeout):
        from controller_manager_msgs.srv import SwitchController, ListControllers
        started = time.monotonic()
        deadline = started + timeout
        with self.lock:
            self.stopped.set()
            commanded, pending = self.commanded, self.goal_future
        if not commanded:
            return True
        terminal = False
        try:
            if pending is not None:
                handle = self._wait_future(pending, time.monotonic() + timeout * 0.35, True)
                if not handle.accepted:
                    terminal = True
                else:
                    # Request cancellation even if the worker was interrupted
                    # before processing its goal-acceptance response.
                    self._wait_future(handle.cancel_goal_async(), time.monotonic() + timeout * 0.2, True)
                    result = self._wait_future(handle.get_result_async(), time.monotonic() + timeout * 0.2, True)
                    terminal = result.status in (4, 5, 6)
        except Exception:
            terminal = False
        if not terminal:
            request = SwitchController.Request()
            request.deactivate_controllers = [self.config.controller_name]
            request.strictness = SwitchController.Request.STRICT
            request.timeout.sec = max(0, int(deadline-time.monotonic()) - 1)
            request.timeout.nanosec = 100_000_000
            response = self._call(SwitchController, self.config.controller_manager.rstrip('/') + '/switch_controller',
                                  request, deadline, True)
            if not response.ok:
                raise RuntimeError("cancel_unconfirmed_and_controller_stop_rejected")
            state = self._call(ListControllers, self.config.controller_manager.rstrip('/') + '/list_controllers',
                               ListControllers.Request(), deadline, True)
            if any(c.name == self.config.controller_name and c.state == 'active' for c in state.controller):
                raise RuntimeError("controller_stop_not_confirmed")
        stationary_since = None
        while time.monotonic() < deadline:
            try:
                self._state(stationary=True)
                if self.joint_received <= started:
                    raise RuntimeError("no_new_stop_feedback")
            except Exception:
                stationary_since = None
                time.sleep(0.02)
                continue
            now = time.monotonic()
            stationary_since = now if stationary_since is None else stationary_since
            if now - stationary_since >= self.config.settle_time:
                if not terminal:
                    # Controller deactivation contains motion, but a pending
                    # action must not be mistaken for confirmed cancellation.
                    raise RuntimeError("controller_deactivated_but_action_cancel_unconfirmed")
                return True
            time.sleep(0.02)
        raise RuntimeError("robot_stop_feedback_unconfirmed")

    def finish(self):
        if self.next_segment != len(self.plans) or not self.is_pose_reached(0.1):
            raise RuntimeError("final_pose_not_reached")

    def close(self):
        if self.mode_lock is not None:
            self.mode_lock.close()
            self.mode_lock = None

#!/usr/bin/env python3
"""Real MoveIt/model integration, isolated DDS; NO hardware driver is launched.

Source the same ROS overlays as run_robot_prepare.sh, then run this file.
Only move_group, robot_state_publisher and synthetic joint feedback are started.
Logs stay in /tmp/moveit-grasp-test-* for diagnosis.
"""
import json
import math
import os
from pathlib import Path
import random
import signal
import subprocess
import tempfile
from threading import Event, Thread
import time
from dataclasses import replace

# Set isolation BEFORE importing/initializing ROS. Reject occupied domains below.
original_domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
os.environ['ROS_DOMAIN_ID'] = str(random.SystemRandom().choice(
    [domain for domain in range(150, 220) if domain != original_domain]))
for setting in ('ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                'FASTDDS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI'):
    os.environ.pop(setting, None)
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose as RosPose
from moveit_msgs.msg import (DisplayTrajectory, RobotState, CollisionObject, PlanningScene,
                            AttachedCollisionObject, AllowedCollisionEntry)
from moveit_msgs.srv import GetStateValidity, ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from ament_index_python.packages import get_package_share_directory as share
import xacro
import yaml

from grasp_executor.config import load_config
from grasp_executor.grasp import Pose, build_place_drop_pose, execute_grasp, execute_grasp_place, run_batch_task
from grasp_executor.scene_cleanup import scene_request, has_legacy_objects
from grasp_executor.moveit_ros import MoveItArm, JOINTS


def check_candidate_rounds(node, preview_arm, execution_arm, fake_gripper, positions,
                           initial_positions, bowls, fake_goals, directory):
    """Use actual MoveIt rejection/planning with only simulated controller actions."""
    from copy import deepcopy
    import numpy as np
    from grasp_executor.candidates import run_candidate_group_task
    from grasp_executor.interfaces import HardwareInterfaces
    from grasp_executor.planning_errors import CandidatePlanRejected

    journal = directory / 'candidate_journal'
    task_id = 'isolated_moveit_candidates'
    unreachable = replace(bowls[0], position=(1.4, 1.4, 1.3))
    also_unreachable = replace(bowls[0], position=(1.35, 1.4, 1.3))

    def group(object_id, poses):
        return {'object_id': object_id, 'distance': math.hypot(*poses[0].position[:2]),
                'candidates': [{'candidate_id': index,
                    'position': [float(value) for value in pose.position],
                    'orientation': [float(value) for value in pose.orientation],
                    'geometry': {'method': 'synthetic_moveit_fixture'}}
                    for index, pose in enumerate(poses)]}

    def batch(observation_id, groups):
        return {'schema_version': 1, 'task_id': task_id, 'observation_id': observation_id,
                'source_stamp_ns': node.get_clock().now().nanoseconds,
                'frame_id': 'base_link', 'tool_frame': 'grasp_center',
                'objects': groups, 'rejected': []}

    def state_positions(state):
        joint = state.joint_state
        return [joint.position[joint.name.index(name)] for name in JOINTS]

    def record_plans(adapter):
        records = []
        original = adapter.prepare_plans
        def capture(*args, **kwargs):
            start = (deepcopy(adapter.preview_state) if adapter.config.plan_only
                     and getattr(adapter, 'preview_state', None) is not None
                     else adapter._state(stationary=True))
            entry = {'start': state_positions(start), 'grasp': args[1], 'rejected': False,
                     'measured_start': state_positions(adapter._state(stationary=True)),
                     'actions_before': len(fake_goals), 'gripper_before': len(fake_gripper.commands)}
            records.append(entry)
            try:
                return original(*args, **kwargs)
            except CandidatePlanRejected:
                entry['rejected'] = True
                raise
            except Exception as exc:
                print('Candidate integration planning failure:', str(exc),
                      'measured before:', entry['measured_start'],
                      'measured after:', state_positions(adapter._state(stationary=True)),
                      'synthetic feedback:', list(positions),
                      'action count:', entry['actions_before'], len(fake_goals), flush=True)
                raise
            finally:
                entry['plans'] = deepcopy(adapter.plans)
                entry['actions_after'] = len(fake_goals)
                entry['gripper_after'] = len(fake_gripper.commands)
        adapter.prepare_plans = capture
        return records, original

    def run(payload, adapter, events, source_not_before_ns=0):
        return run_candidate_group_task(payload, lambda: node.get_clock().now().nanoseconds,
            HardwareInterfaces(adapter, fake_gripper), adapter.config,
            expected_task_id=task_id, expected_observation_id=payload['observation_id'],
            source_not_before_ns=source_not_before_ns,
            task_deadline_monotonic_ns=time.monotonic_ns() + 240_000_000_000,
            on_state=lambda value: events.append(deepcopy(value)), cancel=adapter.cancel)

    execution_arm.close()
    positions[:] = initial_positions
    time.sleep(.4)
    preview_arm.config = replace(preview_arm.config, plan_only=True, execution_journal=str(journal))
    previews, original_preview = record_plans(preview_arm)
    executions, original_execution = record_plans(execution_arm)
    try:
        goals_before, commands_before = len(fake_goals), len(fake_gripper.commands)
        preview_batch = batch('preview_two_objects', [group(2, [bowls[0]]),
                                                       group(7, [unreachable, bowls[1]])])
        result = run(preview_batch, preview_arm, [])
        assert result.status == 'planned' and result.object_count == 2, result
        assert result.completed_count == 0 and result.candidate_attempt_count == 3
        assert len(result.candidate_failures) == 1 and result.candidate_id == 1
        assert [entry['rejected'] for entry in previews] == [False, True, False]
        assert not previews[1]['plans']  # Failed candidate never exposes previous successful plans.
        previous_end = list(previews[0]['plans'][-1].trajectory.joint_trajectory.points[-1].positions)
        np.testing.assert_allclose(previews[1]['start'], previous_end, atol=1e-8)
        np.testing.assert_allclose(previews[2]['start'], previous_end, atol=1e-8)
        assert len(previews[0]['plans']) == len(previews[2]['plans']) == 6
        assert len(fake_goals) == goals_before and len(fake_gripper.commands) == commands_before
        assert not journal.exists() and preview_arm.preview_state is None
        print('PASS grouped preview: real unreachable contact falls back from same virtual start, no commands', flush=True)

        preview_arm.close()
        execution_arm.config = replace(execution_arm.config, plan_only=False, execution_journal=str(journal))
        exhausted_batch = batch('all_candidates_unreachable', [group(7, [unreachable, also_unreachable])])
        result = run(exhausted_batch, execution_arm, [])
        assert result.status == 'candidates_exhausted' and result.candidate_attempt_count == 2, result
        assert result.command_attempted is False and result.completed_count == 0
        assert len(result.candidate_failures) == 2
        assert len(executions) == 2 and all(entry['rejected'] for entry in executions)
        assert not execution_arm.plans
        assert len(fake_goals) == goals_before and len(fake_gripper.commands) == commands_before
        assert not journal.exists()
        print('PASS execute mode: two real unreachable candidates exhaust without arm/gripper commands or stale plans', flush=True)
        executions.clear()

        first_start = list(positions)
        events = []
        first = batch('execute_round_0001', [group(7, [unreachable, bowls[0]])])
        result = run(first, execution_arm, events)
        assert result.status == 'success' and result.completed_count == 1, result
        assert result.candidate_attempt_count == 2 and result.candidate_id == 1
        assert len(result.candidate_failures) == 1
        assert len(fake_goals) - goals_before == 6
        assert fake_gripper.commands[commands_before:] == ['Open', 'Close', 'Open']
        assert any(event['status'] == 'candidate_rejected' and event['command_attempted'] is False
                   for event in events)
        np.testing.assert_allclose(executions[0]['start'], first_start, atol=1e-8)
        np.testing.assert_allclose(executions[1]['start'], first_start, atol=1e-8)
        assert not executions[0]['plans']
        assert executions[0]['actions_before'] == executions[0]['actions_after'] == goals_before
        assert executions[0]['gripper_before'] == executions[0]['gripper_after'] == commands_before
        assert len(list(journal.glob('*.json'))) == 1
        print('PASS grouped execution: unreachable candidate sends no commands; fallback executes exactly six simulated segments', flush=True)

        # A new observation reuses the frame-local ID, changes its target, and
        # plans from the newly measured post-retreat joints, not the first start.
        time.sleep(.3)
        second_start = list(positions)
        assert not np.allclose(first_start, second_start, atol=1e-3)
        barrier = node.get_clock().now().nanoseconds
        second = batch('execute_round_0002', [group(7, [bowls[1]])])
        assert second['source_stamp_ns'] > barrier > first['source_stamp_ns']
        result = run(second, execution_arm, [], source_not_before_ns=barrier)
        assert result.status == 'success' and result.completed_count == 1, result
        assert result.candidate_attempt_count == 1
        np.testing.assert_allclose(executions[2]['start'], second_start, atol=1e-8)
        assert executions[2]['grasp'] != executions[1]['grasp']
        assert len(fake_goals) - goals_before == 12
        assert fake_gripper.commands[commands_before:] == ['Open', 'Close', 'Open'] * 2
        assert len(list(journal.glob('*.json'))) == 2
        print('PASS new observation: reused local ID and new pose plan from fresh post-retreat measured joints', flush=True)
        return {'preview_candidate_attempts': 3, 'exhausted_candidate_attempts': 2,
                'executed_objects': 2, 'executed_segments': 12, 'journal_records': 2,
                'fresh_measured_start_verified': True, 'exhausted_in_execute_mode': True,
                'hardware_connected': False}
    finally:
        preview_arm.prepare_plans = original_preview
        execution_arm.prepare_plans = original_execution
        preview_arm.close()
        execution_arm.close()


def _main_locked():
    directory = Path(tempfile.mkdtemp(prefix='moveit-grasp-test-'))
    print('Test logs:', directory, 'ROS_DOMAIN_ID:', os.environ['ROS_DOMAIN_ID'], flush=True)
    processes, logs = [], []
    rclpy.init()
    node = Node('moveit_grasp_test')
    arm = None
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = None
    try:
        limit = time.monotonic() + 1.5
        while time.monotonic() < limit:
            executor.spin_once(timeout_sec=.1)
        if any(n != 'moveit_grasp_test' for n in node.get_node_names()):
            raise RuntimeError('Test domain occupied; rerun to choose another domain')
        description = xacro.process_file(str(Path(share('mobile_ur_description'))/'urdf/ur5e_on_new_base_with_gripper_mobile.urdf.xacro'),
            mappings={'name':'ur', 'ur_type':'ur5e', 'use_fake_hardware':'true', 'safety_limits':'true'}).toxml()
        moveit = Path(share('ur_moveit_config'))
        semantic = xacro.process_file(str(moveit/'srdf/mobile_ur.srdf.xacro'), mappings={'name':'ur', 'prefix':''}).toxml()
        def read(name): return yaml.safe_load((moveit/'config'/name).read_text())
        params = {'robot_description': description, 'robot_description_semantic': semantic,
                  'robot_description_planning': read('joint_limits.yaml'),
                  'move_group': {'planning_plugin':'ompl_interface/OMPLPlanner',
                    'request_adapters':'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
                    'start_state_max_bounds_error':.1, **read('ompl_planning.yaml')},
                  'moveit_controller_manager':'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                  'moveit_simple_controller_manager':read('controllers.yaml'), 'moveit_manage_controllers':False,
                  'use_sim_time':False, 'publish_robot_description_semantic':True}
        params.update(read('kinematics.yaml')['/**']['ros__parameters'])
        config_file = directory/'moveit.yaml'
        config_file.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
        for package, executable in [('robot_state_publisher','robot_state_publisher'), ('moveit_ros_move_group','move_group')]:
            output = (directory/(executable+'.log')).open('w')
            logs.append(output)
            processes.append(subprocess.Popen(['ros2','run',package,executable,'--ros-args','--params-file',str(config_file)],
                stdout=output, stderr=subprocess.STDOUT, start_new_session=True))
        positions = [0., -1.2, 1.6, -1.97, -1.57, 0.]
        publisher = node.create_publisher(JointState, '/joint_states', 10)
        def publish():
            message = JointState()
            message.header.stamp = node.get_clock().now().to_msg()
            message.name, message.position, message.velocity = list(JOINTS), positions, [0.]*6
            publisher.publish(message)
        timer = node.create_timer(.02, publish)
        displays = []
        sub = node.create_subscription(DisplayTrajectory, '/display_planned_path', displays.append,
            QoSProfile(depth=10))
        root = Path(__file__).resolve().parents[1]
        config = load_config(root / "src/grasp_executor/config/config.json")
        arm = MoveItArm(node, config, Event())
        thread = Thread(target=executor.spin, daemon=True)
        thread.start()
        deadline = time.monotonic()+40
        client = node.create_client(GetStateValidity, '/check_state_validity')
        while not client.service_is_ready():
            if any(p.poll() is not None for p in processes): raise RuntimeError('MoveIt process exited; inspect logs')
            if time.monotonic() > deadline: raise TimeoutError('MoveIt startup')
            time.sleep(.1)
        # Choose a well-conditioned, collision-free fixture pose using the real model.
        candidates = [positions, [0., -1.57, 1.57, -1.57, -1.57, 0.],
                      [0., -1.5, 1.8, -1.9, -1.57, 0.], [0., -1.8, 1.4, -1.2, -1.57, 0.]]
        for candidate in candidates:
            positions[:] = candidate
            time.sleep(.3)
            state = arm._state(stationary=True)
            response = arm._call(GetStateValidity, '/check_state_validity',
                GetStateValidity.Request(robot_state=state, group_name='ur_manipulator'), deadline)
            print('Fixture', candidate, 'valid:', response.valid,
                  [(c.contact_body_1,c.contact_body_2) for c in response.contacts], flush=True)
            if response.valid: break
        else: raise RuntimeError('No collision-free fixture pose')
        initial_positions = list(positions)
        assert arm.check_ready(15.)
        actual = arm.read_pose()
        print('Actual grasp_center:', actual, flush=True)
        # Seed the old, blocking objects as if MoveIt remained open across an
        # upgrade. Production cleanup must remove them without reading a model.
        def seed_legacy_scene():
            scene = PlanningScene(is_diff=True)
            scene.robot_state.is_diff = True
            obj = CollisionObject(id='grasp_table', operation=CollisionObject.ADD)
            obj.header.frame_id = arm.config.base_frame
            box = RosPose()
            box.position.x, box.position.y, box.position.z = actual.position
            box.orientation.w = 1.
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.3,.3,.3])]
            obj.primitive_poses = [box]
            scene.world.collision_objects = [obj]
            attached = AttachedCollisionObject(link_name='gripper_base_link',
                                               touch_links=['gripper_base_link', 'tool0', 'flange', 'wrist_3_link'])
            from copy import deepcopy
            attached.object = deepcopy(obj)
            attached.object.id = 'grasp_gripper_envelope'
            attached.object.header.frame_id = 'gripper_base_link'
            attached.object.primitive_poses[0].position.x = 0.
            attached.object.primitive_poses[0].position.y = 0.
            attached.object.primitive_poses[0].position.z = .1
            scene.robot_state.attached_collision_objects = [attached]
            current = arm._call(GetPlanningScene, '/get_planning_scene', scene_request(), time.monotonic()+5.).scene
            acm = deepcopy(current.allowed_collision_matrix)
            original_acm = deepcopy(acm)
            for name in ('grasp_table', 'grasp_gripper_envelope'):
                for row in acm.entry_values:
                    row.enabled.append(False)
                acm.entry_names.append(name)
                acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
            scene.allowed_collision_matrix = acm
            assert arm._call(ApplyPlanningScene, '/apply_planning_scene',
                             ApplyPlanningScene.Request(scene=scene), time.monotonic()+5.).success
            return original_acm

        original_acm = seed_legacy_scene()
        # Exercise the same cleanup command called by run_robot_prepare.sh.
        subprocess.run(['/usr/bin/python3', str(root / 'tools/clear_legacy_grasp_scene.py')],
                       check=True, timeout=40)
        cleaned = arm._call(GetPlanningScene, '/get_planning_scene', scene_request(), time.monotonic()+5.).scene
        assert not has_legacy_objects(cleaned)
        assert cleaned.allowed_collision_matrix == original_acm
        print('PASS startup cleanup removed legacy table/envelope and preserved SRDF matrix', flush=True)
        seed_legacy_scene()  # Backend must also clean an already-running scene.
        approach = replace(actual, position=(actual.position[0]+.015, actual.position[1], actual.position[2]+.035))
        grasp = replace(approach, position=(*approach.position[:2], approach.position[2]-.03))
        assert arm.prepare_plans(approach, grasp)
        cleaned = arm._call(GetPlanningScene, '/get_planning_scene', scene_request(), time.monotonic()+5.).scene
        assert not has_legacy_objects(cleaned) and not arm.world_objects
        assert cleaned.allowed_collision_matrix == original_acm
        print('PASS backend removes residual objects; plans with no added collision model', flush=True)
        time.sleep(.2)
        assert len(arm.plans) == 3 and len(displays[-1].trajectory) == 3
        assert arm.commanded is False
        print('PASS real OMPL approach + Cartesian descent/lift + RViz display (no execution)', flush=True)
        grasp_plans = arm.plans
        arm.config = load_config(root / "src/grasp_executor/config/place_config.json")
        place_ready = arm.resolve_place_pose()
        place_drop = build_place_drop_pose(place_ready, arm.config)
        assert arm.prepare_plans(approach, grasp, place_ready, place_drop, place_ready)
        assert len(arm.plans) == 6 and len(displays[-1].trajectory) == 6
        print('PASS fixed place reference + 20 cm drop/retreat six-segment planning', flush=True)
        place_plans = arm.plans
        arm.plans, arm.next_segment = grasp_plans, 0
        # A fake FollowJointTrajectory server verifies the REAL MoveIt execution
        # mapping without starting ros2_control or connecting a hardware driver.
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionServer
        from rclpy.callback_groups import ReentrantCallbackGroup
        from controller_manager_msgs.srv import ListControllers
        from controller_manager_msgs.msg import ControllerState
        from std_msgs.msg import Bool, String
        from ur_dashboard_msgs.msg import RobotMode, SafetyMode
        from grasp_executor.interfaces import HardwareInterfaces
        fake_goals = []
        def follow(handle):
            fake_goals.append(handle.request.trajectory)
            end = dict(zip(handle.request.trajectory.joint_names, handle.request.trajectory.points[-1].positions))
            positions[:] = [end[j] for j in JOINTS]
            time.sleep(.15)  # Permit fresh joint state/TF delivery before success.
            handle.succeed()
            return FollowJointTrajectory.Result(error_code=0)
        action_server = ActionServer(node, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory', follow,
            callback_group=ReentrantCallbackGroup())
        def controllers(request, response):
            response.controller = [ControllerState(name='scaled_joint_trajectory_controller', state='active',
                type='ur_controllers/ScaledJointTrajectoryController', claimed_interfaces=[j+'/position' for j in JOINTS])]
            return response
        service = node.create_service(ListControllers, '/controller_manager/list_controllers', controllers)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        program_pub = node.create_publisher(Bool, '/io_and_status_controller/robot_program_running', retained)
        mode_pub = node.create_publisher(RobotMode, '/io_and_status_controller/robot_mode', retained)
        safety_pub = node.create_publisher(SafetyMode, '/io_and_status_controller/safety_mode', retained)
        hand_node = Node('hand_control_node')
        hand_sub = hand_node.create_subscription(String, '/handControlCmd', lambda msg: None, 10)
        executor.add_node(hand_node)
        class FakeGripper:
            def __init__(self): self.commands = []
            def command(self, command, timeout): self.commands.append(command); return True
            def stop(self): pass
            def check_ready(self, timeout): return True
        preview_arm = arm
        preview_arm.close()
        arm = MoveItArm(node, replace(preview_arm.config, plan_only=False), Event())
        program_pub.publish(Bool(data=True))
        mode_pub.publish(RobotMode(mode=7))
        safety_pub.publish(SafetyMode(mode=1))
        time.sleep(.4)
        assert arm.check_ready(15.)
        arm.plans = preview_arm.plans
        fake_gripper = FakeGripper()
        result = execute_grasp(approach, grasp, HardwareInterfaces(arm, fake_gripper), arm.config)
        assert result.status == 'success', result
        assert len(fake_goals) == 3 and fake_gripper.commands == ['Open','Close']
        print('PASS real ExecuteTrajectory -> simulated scaled-controller Action; 3 segments and Open/Close', flush=True)
        positions[:] = initial_positions
        time.sleep(.4)
        arm.plans, arm.next_segment = place_plans, 0
        result = execute_grasp_place(approach, grasp, place_ready, place_drop,
                                     HardwareInterfaces(arm, fake_gripper), arm.config)
        assert result.status == 'success', result
        assert len(fake_goals) == 9 and fake_gripper.commands == ['Open', 'Close', 'Open', 'Close', 'Open']
        print('PASS simulated six-segment grasp/place execution, release and retreat', flush=True)
        # End-to-end three-object batch: second preview starts at the previous
        # retreat, and execution replans from freshly simulated feedback.
        from copy import deepcopy
        import numpy as np
        from rim_locator.config import load_config as load_rim_config
        from rim_locator.rim_detection import locate_grasp_pose
        positions[:] = initial_positions
        time.sleep(.4)
        batch_offset = dict(x=.001, y=0., z=.002)
        preview_arm.config = replace(preview_arm.config, execution_journal=str(directory/'journal'),
                                     grasp_offset=batch_offset)
        rim_config = load_rim_config(root/'src/rim_locator/config/place_config.json')
        batch = []
        for cx, cy, tilt in ((.74, .18, 20.), (.70, .10, 0.), (.72, .14, 20.)):
            theta, depth = np.meshgrid(np.linspace(0.,2*np.pi,240,endpoint=False),
                                       np.linspace(0.,.05,40))
            radius = .085-depth*math.tan(math.radians(tilt))
            cloud = np.c_[(cx+radius*np.cos(theta)).ravel(), (cy+radius*np.sin(theta)).ravel(),
                           (.15-depth).ravel()]
            located = locate_grasp_pose(cloud, rim_config)
            batch.append(Pose(tuple(located.point), located.orientation, 'base_link', 'grasp_center'))
            print('Synthetic bowl wall tilt:',tilt,'grasp:',batch[-1],flush=True)
        batch.sort(key=lambda p:math.hypot(*p.position[:2]))
        arm.close()  # Release the execution lock before preview reserves it.
        captured, batch_previews = [], []
        original_prepare = preview_arm.prepare_plans
        def capture(*args, **kwargs):
            ok = original_prepare(*args, **kwargs)
            captured.append(deepcopy(preview_arm.plans))
            return ok
        preview_arm.prepare_plans = capture
        stamp = node.get_clock().now().nanoseconds
        result = run_batch_task(batch, stamp, lambda:node.get_clock().now().nanoseconds,
                                HardwareInterfaces(preview_arm, fake_gripper), preview_arm.config,
                                on_poses=lambda *poses: batch_previews.append(poses))
        assert result.status == 'planned', result
        assert len(captured) == len(batch_previews) == 3
        for raw, (ap, gp, ready, drop) in zip(batch, batch_previews):
            np.testing.assert_allclose(gp.position, np.array(raw.position) + [.001, 0., .002], atol=1e-12)
            assert gp.orientation == raw.orientation
            np.testing.assert_allclose(ap.position, np.array(gp.position) +
                                       [0., 0., preview_arm.config.approach_height], atol=1e-12)
            assert ready == place_ready and drop == place_drop
        assert all(len(plans) == 6 for plans in captured)
        last = captured[0][-1].trajectory.joint_trajectory.points[-1].positions
        next_start = captured[1][0].start.joint_state
        assert list(last) == [next_start.position[next_start.name.index(j)] for j in JOINTS]
        print('PASS three-object preview applies base-frame offset once and chains second start to first retreat', flush=True)
        preview_arm.close()
        arm.config = replace(arm.config, execution_journal=str(directory/'journal'),
                             grasp_offset=batch_offset)
        goals_before = len(fake_goals)
        commands_before = len(fake_gripper.commands)
        execution_previews = []
        result = run_batch_task(batch, stamp, lambda:node.get_clock().now().nanoseconds,
                                HardwareInterfaces(arm, fake_gripper), arm.config,
                                on_poses=lambda *poses: execution_previews.append(poses))
        assert result.status == 'success' and result.completed_count == 3, result
        assert execution_previews == batch_previews
        assert len(fake_goals)-goals_before == 18
        assert fake_gripper.commands[commands_before:] == ['Open','Close','Open']*3
        print('PASS three-object simulated execution: 18 segments, three releases/returns', flush=True)
        preview_arm.prepare_plans = original_prepare
        candidate_checks = check_candidate_rounds(node, preview_arm, arm, fake_gripper,
            positions, initial_positions, batch, fake_goals, directory)
        # Reset synthetic feedback and use preview mode again for rejection tests.
        arm.close()
        arm = preview_arm
        positions[:] = initial_positions
        time.sleep(.3)
        executor.remove_node(hand_node)
        hand_node.destroy_node()
        action_server.destroy()
        # Same exact model conversion is checked in check_ready via FK vs live TF.
        assert arm.tip_to_tool[0] != (0.,0.,0.)
        print('PASS real grasp_center -> tool0 transform, FK/TF agreement', flush=True)
        unreachable = replace(approach, position=(1.4,1.4,1.4))
        try:
            # Production configuration allows 15 s of IK search; the client
            # deadline must also include service/transport overhead.
            arm._pose_plan(arm._state(stationary=True), unreachable,
                           time.monotonic() + arm.config.allowed_planning_time + 5.)
        except RuntimeError as exc:
            assert 'planning_failed' in str(exc) or 'ik_failed' in str(exc), str(exc)
            print('PASS unreachable target rejected:', exc, flush=True)
        else: raise AssertionError('Unreachable target unexpectedly planned')
        # Synthetic obstacle deliberately overlapping the current gripper verifies
        # the real collision engine. It is never installed in the hardware domain.
        obj = CollisionObject(id='test_obstacle', operation=CollisionObject.ADD)
        obj.header.frame_id = arm.config.base_frame
        box = RosPose()
        box.position.x, box.position.y, box.position.z = actual.position
        box.orientation.w = 1.
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.3,.3,.3])]
        obj.primitive_poses = [box]
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [obj]
        result = arm._call(ApplyPlanningScene, '/apply_planning_scene', ApplyPlanningScene.Request(scene=scene), time.monotonic()+5.)
        assert result.success
        try:
            arm.prepare_plans(approach, grasp)
        except RuntimeError as exc:
            assert 'collision_or_invalid_state' in str(exc), str(exc)
            print('PASS real collision rejection:', exc, flush=True)
        else: raise AssertionError('Colliding start unexpectedly accepted')
        assert not arm.commanded
        (directory/'result.json').write_text(json.dumps({'passed':15,'hardware_connected':False,
            'domain':os.environ['ROS_DOMAIN_ID'], 'batch_grasp_offset':batch_offset,
            'offset_preview_matches_execution':True, 'object_count':3, 'executed_segments':18,
            'candidate_rounds':candidate_checks}))
        print('15 MoveIt integration checks passed; hardware was not connected.', flush=True)
    finally:
        if arm: arm.close()
        executor.shutdown(timeout_sec=2)
        if thread: thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
        for p in processes:
            if p.poll() is None: os.killpg(p.pid, signal.SIGINT)
        for p in processes:
            try: p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait(timeout=2)
        for output in logs: output.close()


def main():
    # Independent empty-domain probes can race or discover other RMWs late.
    # Hold the common fixture lock from before rclpy.init until all nodes and
    # child processes have been cleaned up, including startup exceptions.
    from isolated_ros import IsolatedRosTestLock
    with IsolatedRosTestLock():
        return _main_locked()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Real MoveIt/model integration, isolated DDS; NO hardware driver is launched.

Source the same ROS overlays as run_robot_prepare.sh, then run this file.
Only move_group, robot_state_publisher and synthetic joint feedback are started.
Logs stay in /tmp/moveit-grasp-test-* for diagnosis.
"""
import json
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
from moveit_msgs.msg import DisplayTrajectory, RobotState, CollisionObject, PlanningScene
from moveit_msgs.srv import GetStateValidity, ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from ament_index_python.packages import get_package_share_directory as share
import xacro
import yaml

from grasp_executor.config import GraspConfig
from grasp_executor.grasp import Pose, execute_grasp
from grasp_executor.moveit_ros import MoveItArm, JOINTS


def main():
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
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        arm = MoveItArm(node, GraspConfig(), Event())
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
        approach = replace(actual, position=(actual.position[0]+.015, actual.position[1], actual.position[2]+.035))
        grasp = replace(approach, position=(*approach.position[:2], approach.position[2]-.03))
        assert arm.prepare_plans(approach, grasp)
        time.sleep(.2)
        assert len(arm.plans) == 3 and len(displays[-1].trajectory) == 3
        assert arm.commanded is False
        print('PASS real OMPL approach + Cartesian descent/lift + RViz display (no execution)', flush=True)
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
            arm._pose_plan(arm._state(stationary=True), unreachable, time.monotonic()+15.)
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
        (directory/'result.json').write_text(json.dumps({'passed':5,'hardware_connected':False,'domain':os.environ['ROS_DOMAIN_ID']}))
        print('5 MoveIt integration checks passed; hardware was not connected.', flush=True)
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


if __name__ == '__main__':
    main()

"""Offline logic tests plus optional isolated fake-ROS integration tests.

Run pure checks: python3 -B tools/test_robot_control.py
Run fake ROS graph: ROBOT_STARTUP_ROS_TEST=1 /usr/bin/python3 -B tools/test_robot_control.py
The integration fixture selects an unused localhost-only domain and never starts
a driver, a hardware interface, Servo, or a physical robot connection.
"""

import os
from pathlib import Path
import random
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest

from robot_control import Controller, CONTROLLER_TYPES, JOINTS, joint_values, switch_plan, camera_pair_ready


def controller(name, state="inactive"):
    return Controller(name, state, CONTROLLER_TYPES.get(name, "test/type"))


class CameraTests(unittest.TestCase):
    def rows(self, *stamps):
        return [(s, 1280, 800, 'camera_color_optical_frame') for s in stamps]

    def test_fresh_advancing_pairs(self):
        self.assertTrue(camera_pair_ready(self.rows(9_800_000_000, 9_900_000_000),
                                         self.rows(9_810_000_000, 9_910_000_000), 10_000_000_000))

    def test_frozen_stamps_rejected(self):
        rows = self.rows(9_900_000_000, 9_900_000_000)
        self.assertFalse(camera_pair_ready(rows, rows, 10_000_000_000))

    def test_stale_future_and_unsynchronized_rejected(self):
        for stamps in ((1, 2), (12_000_000_000, 12_100_000_000), (9_400_000_000, 9_500_000_000)):
            with self.subTest(stamps=stamps):
                self.assertFalse(camera_pair_ready(self.rows(9_800_000_000, 9_900_000_000),
                                                  self.rows(*stamps), 10_000_000_000))

    def test_unaligned_dimensions_rejected(self):
        rows = self.rows(9_800_000_000, 9_900_000_000)
        self.assertFalse(camera_pair_ready(rows, [(r[0], 640, 400, r[3]) for r in rows], 10_000_000_000))

    def test_two_rgb_and_depth_samples_need_two_actual_pairs(self):
        self.assertFalse(camera_pair_ready(self.rows(9_800_000_000,9_900_000_000),
                                          self.rows(8_000_000_000,9_810_000_000),10_000_000_000))

    def test_mismatched_optical_frames_rejected(self):
        rows=self.rows(9_800_000_000,9_900_000_000)
        self.assertFalse(camera_pair_ready(rows,[(s,w,h,'depth_frame') for s,w,h,f in rows],10_000_000_000))

    def test_recent_receipt_does_not_make_old_source_fresh(self):
        rows=[(*r,30_000_000_000) for r in self.rows(9_800_000_000,9_900_000_000)]
        self.assertFalse(camera_pair_ready(rows,rows,30_000_000_000))


class SelectionTests(unittest.TestCase):
    def test_inactive_activation(self):
        self.assertEqual(switch_plan([controller("scaled_joint_trajectory_controller")],
                                    "scaled_joint_trajectory_controller"),
                         (["scaled_joint_trajectory_controller"], []))

    def test_idempotent_and_conflicts(self):
        target = controller("scaled_joint_trajectory_controller", "active")
        self.assertEqual(switch_plan([target], target.name), ([], []))
        self.assertEqual(switch_plan([target, controller("forward_position_controller", "active"),
                                     controller("cartesian_motion_controller", "active")], target.name),
                         ([], ["forward_position_controller", "cartesian_motion_controller"]))

    def test_unknown_owner_is_not_silently_stopped(self):
        stranger = Controller("other_task", "active", "unknown", ("wrist_1_joint/velocity",))
        with self.assertRaisesRegex(RuntimeError, "未知控制器"):
            switch_plan([controller("forward_position_controller"), stranger], "forward_position_controller")

    def test_missing_unconfigured_and_wrong_type(self):
        for controllers in ([], [controller("forward_position_controller", "unconfigured")],
                            [Controller("forward_position_controller", "inactive", "wrong/type")]):
            with self.subTest(controllers=controllers), self.assertRaises(RuntimeError):
                switch_plan(controllers, "forward_position_controller")

    def test_feedback_freshness_and_motion(self):
        message = NS(header=NS(stamp=NS(sec=10, nanosec=0)), name=list(JOINTS),
                     position=[0.0] * 6, velocity=[0.0] * 6)
        self.assertEqual(joint_values(message, 10_100_000_000)[1], (0.0,) * 6)
        with self.assertRaisesRegex(ValueError, "时间戳"):
            joint_values(message, 12_000_000_000)
        message.velocity[2] = 0.1
        with self.assertRaisesRegex(ValueError, "仍在运动"):
            joint_values(message, 10_100_000_000)
        message.velocity = []
        with self.assertRaisesRegex(ValueError, "完整位置/速度"):
            joint_values(message, 10_100_000_000)


class ShellOwnershipTests(unittest.TestCase):
    common = str(Path(__file__).with_name("robot_startup_common.sh").resolve())

    def test_mode_lock_excludes_other_session_and_releases(self):
        with tempfile.TemporaryDirectory() as directory:
            holder = subprocess.Popen(["bash", "-c",
                'source "$1"; ROBOT_RUNTIME_DIR="$2"; lock_mode; echo locked; read -r release',
                "test-lock", self.common, directory], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, text=True)
            try:
                self.assertEqual(holder.stdout.readline().strip(), "locked")
                contender = ["bash", "-c", 'source "$1"; ROBOT_RUNTIME_DIR="$2"; lock_mode',
                             "test-lock", self.common, directory]
                result = subprocess.run(contender, capture_output=True, text=True, timeout=3)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("另一个模式脚本", result.stderr)
                holder.communicate("release\n", timeout=3)
                result = subprocess.run(contender, capture_output=True, text=True, timeout=3)
                self.assertEqual(result.returncode, 0, result.stderr)
            finally:
                if holder.poll() is None:
                    holder.kill()
                holder.communicate()

    def test_owned_children_exit_without_stopping_unrelated_process(self):
        unrelated = subprocess.Popen(["sleep", "60"])
        with tempfile.TemporaryDirectory() as directory:
            holder = subprocess.Popen(["bash", "-c",
                'source "$1"; ROBOT_RUNTIME_DIR="$2"; install_cleanup; '
                'start_owned fixture sleep 60; echo "child=${ROBOT_CHILDREN[0]}"; read -r release',
                "test-cleanup", self.common, directory], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, text=True)
            try:
                child = None
                for _ in range(5):
                    line = holder.stdout.readline().strip()
                    if line.startswith("child="):
                        child = int(line.split("=")[1])
                        break
                self.assertIsNotNone(child)
                # Let setsid execute before triggering shutdown.
                time.sleep(0.1)
                holder.communicate("release\n", timeout=6)
                self.assertEqual(holder.returncode, 0)
                self.assertFalse(Path(f"/proc/{child}").exists())
                self.assertIsNone(unrelated.poll())
            finally:
                if holder.poll() is None:
                    holder.kill()
                holder.communicate()
                unrelated.terminate()
                unrelated.wait(timeout=3)


@unittest.skipUnless(os.environ.get("ROBOT_STARTUP_ROS_TEST") == "1", "isolated ROS integration disabled")
class FakeRosTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Do not inherit a DDS discovery server or custom network configuration.
        for name in ("ROS_DISCOVERY_SERVER", "FASTRTPS_DEFAULT_PROFILES_FILE", "CYCLONEDDS_URI"):
            os.environ.pop(name, None)
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from rclpy.action import ActionServer
        from controller_manager_msgs.srv import ListControllers, SwitchController
        from control_msgs.action import FollowJointTrajectory
        from moveit_msgs.action import MoveGroup, ExecuteTrajectory
        from moveit_msgs.srv import GetMotionPlan, GetPositionIK, GetPositionFK, GetCartesianPath, GetStateValidity, GetPlanningScene
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool
        from ur_dashboard_msgs.msg import RobotMode, SafetyMode
        from action_msgs.msg import GoalStatusArray
        from tf2_ros import TransformBroadcaster

        cls.rclpy = rclpy
        current_domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
        for domain in random.sample([i for i in range(150, 220) if i != current_domain], 5):
            context = Context()
            rclpy.init(context=context, domain_id=domain)
            sentinel = Node("startup_test_domain_probe", context=context)
            discovery_executor = SingleThreadedExecutor(context=context)
            discovery_executor.add_node(sentinel)
            until = time.monotonic() + 1.2
            while time.monotonic() < until:
                discovery_executor.spin_once(timeout_sec=0.05)
            occupied = any(name != "startup_test_domain_probe" for name in sentinel.get_node_names())
            discovery_executor.shutdown()
            sentinel.destroy_node()
            if not occupied:
                cls.context = context
                os.environ["ROS_DOMAIN_ID"] = str(domain)
                break
            context.shutdown()
        else:
            raise unittest.SkipTest("no empty test domain found")

        cls.switches = []
        cls.states = {name: "inactive" for name in CONTROLLER_TYPES}
        cls.running = True
        cls.reject_switch = False
        cls.busy = False
        cls.manager = Node("controller_manager", context=cls.context)
        cls.moveit = Node("move_group", context=cls.context)
        cls.feedback = Node("fake_robot_feedback", context=cls.context)
        cls.moveit.declare_parameter("robot_description", '<robot name="test"/>')
        cls.moveit.declare_parameter("robot_description_semantic", '<robot name="test"><group name="ur_manipulator"><chain base_link="base_link" tip_link="tool0"/></group></robot>')

        def list_controllers(request, response):
            from controller_manager_msgs.msg import ControllerState
            response.controller = [ControllerState(name=name, state=state, type=CONTROLLER_TYPES[name])
                                   for name, state in cls.states.items()]
            return response

        def switch_controllers(request, response):
            cls.switches.append((list(request.activate_controllers), list(request.deactivate_controllers)))
            response.ok = not cls.reject_switch
            if response.ok:
                for name in request.deactivate_controllers:
                    cls.states[name] = "inactive"
                for name in request.activate_controllers:
                    cls.states[name] = "active"
            return response

        cls.services = [cls.manager.create_service(ListControllers, "/controller_manager/list_controllers", list_controllers),
                        cls.manager.create_service(SwitchController, "/controller_manager/switch_controller", switch_controllers),
                        cls.moveit.create_service(GetMotionPlan, "/plan_kinematic_path", lambda request, response: response)]
        for kind, service in ((GetPositionIK, '/compute_ik'), (GetPositionFK, '/compute_fk'),
                              (GetCartesianPath, '/compute_cartesian_path'), (GetStateValidity, '/check_state_validity'),
                              (GetPlanningScene, '/get_planning_scene')):
            cls.services.append(cls.moveit.create_service(kind, service, lambda request, response: response))
        cls.actions = []
        for action_type, name in ((FollowJointTrajectory, "/scaled_joint_trajectory_controller/follow_joint_trajectory"),
                                  (MoveGroup, "/move_action"), (ExecuteTrajectory, "/execute_trajectory")):
            def reject_execution(goal_handle):
                raise AssertionError("Startup checks must never submit a motion goal")
            cls.actions.append(ActionServer(cls.moveit, action_type, name, execute_callback=reject_execution))
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.program_pub = cls.feedback.create_publisher(Bool, "/io_and_status_controller/robot_program_running", retained)
        cls.robot_pub = cls.feedback.create_publisher(RobotMode, "/io_and_status_controller/robot_mode", retained)
        cls.safety_pub = cls.feedback.create_publisher(SafetyMode, "/io_and_status_controller/safety_mode", retained)
        cls.joint_pub = cls.feedback.create_publisher(JointState, "/joint_states", 10)
        cls.status_pub = cls.feedback.create_publisher(GoalStatusArray, "/test_motion/_action/status", retained)
        cls.tf = TransformBroadcaster(cls.feedback)

        def publish():
            from geometry_msgs.msg import TransformStamped
            from action_msgs.msg import GoalStatus
            stamp = cls.feedback.get_clock().now().to_msg()
            cls.program_pub.publish(Bool(data=cls.running))
            cls.robot_pub.publish(RobotMode(mode=7))
            cls.safety_pub.publish(SafetyMode(mode=1))
            joint = JointState(name=list(JOINTS), position=[0.0] * 6, velocity=[0.0] * 6)
            joint.header.stamp = stamp
            cls.joint_pub.publish(joint)
            transforms = []
            for child in ("tool0", "grasp_center"):
                transform = TransformStamped()
                transform.header.stamp = stamp
                transform.header.frame_id = "base_link"
                transform.child_frame_id = child
                transform.transform.translation.z = 0.5
                transform.transform.rotation.w = 1.0
                transforms.append(transform)
            cls.tf.sendTransform(transforms)
            statuses = [GoalStatus(status=2)] if cls.busy else []
            cls.status_pub.publish(GoalStatusArray(status_list=statuses))

        cls.timer = cls.feedback.create_timer(0.03, publish)
        cls.executor = SingleThreadedExecutor(context=cls.context)
        for node in (cls.manager, cls.moveit, cls.feedback):
            cls.executor.add_node(node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.thread.join(timeout=3)
        for action in cls.actions:
            action.destroy()
        for node in (cls.manager, cls.moveit, cls.feedback):
            node.destroy_node()
        cls.context.shutdown()

    def setUp(self):
        type(self).switches.clear()
        type(self).states = {name: "inactive" for name in CONTROLLER_TYPES}
        type(self).running = True
        type(self).reject_switch = False
        type(self).busy = False

    def run_probe(self, *args):
        result = subprocess.run([sys.executable, "-B", str(Path(__file__).with_name("robot_control.py")),
                                 "--timeout", "3", *args], capture_output=True, text=True, timeout=25)
        return result

    def test_check_is_read_only(self):
        result = self.run_probe("check-position")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.switches, [])

    def test_position_switch_and_repeat(self):
        type(self).states["forward_position_controller"] = "active"
        result = self.run_probe("position")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.switches, [(["scaled_joint_trajectory_controller"], ["forward_position_controller"])])
        result = self.run_probe("position")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(len(self.switches), 1)

    def test_external_control_missing_does_not_switch(self):
        type(self).running = False
        result = self.run_probe("position")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("External Control", result.stderr)
        self.assertEqual(self.switches, [])

    def test_busy_action_does_not_switch_or_claim_ownership(self):
        type(self).busy = True
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "ownership"
            result = self.run_probe("joystick", str(marker))
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("未完成", result.stderr)
            self.assertFalse(marker.exists())
            self.assertEqual(self.switches, [])

    def test_rejected_switch_reports_failure(self):
        type(self).reject_switch = True
        result = self.run_probe("position")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("切换被拒绝", result.stderr)

    def test_joystick_ownership_and_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "ownership"
            result = self.run_probe("joystick", str(marker))
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertTrue(marker.exists())
            result = self.run_probe("stop-joystick")
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(self.states["forward_position_controller"], "inactive")

    def test_servo_parameters_parse_with_ros(self):
        from rclpy.node import Node
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "servo.yaml"
            result = self.run_probe("servo-config", str(path))
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            node = Node("servo_node", context=self.context,
                        cli_args=["--ros-args", "--params-file", str(path)],
                        automatically_declare_parameters_from_overrides=True)
            try:
                self.assertEqual(node.get_parameter("robot_description").value, '<robot name="test"/>')
                self.assertEqual(node.get_parameter("moveit_servo.command_out_topic").value,
                                 "/forward_position_controller/commands")
            finally:
                node.destroy_node()
            self.assertEqual(self.switches, [])


if __name__ == "__main__":
    unittest.main(verbosity=2)

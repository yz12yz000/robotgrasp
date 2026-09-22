"""ROS 2 transport for the document's FZI cartesian_motion_controller."""

import math
from threading import Event
import time

from .grasp import Pose


class RosCartesianTransport:
    def __init__(self, node, config):
        # Keep ROS imports here so geometry and state-machine tests stay offline.
        from controller_manager_msgs.srv import ListControllers, SwitchController
        from geometry_msgs.msg import PoseStamped
        from rcl_interfaces.srv import GetParameters
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from tf2_ros import Buffer, TransformListener

        self.node, self.config = node, config
        self.tip_frame = None
        self.PoseStamped = PoseStamped
        self.ListControllers, self.SwitchController, self.GetParameters = ListControllers, SwitchController, GetParameters
        controller = '/' + '/'.join(filter(None, [config.controller_namespace.strip('/'), config.controller_name]))
        manager = '/' + config.controller_manager.strip('/')
        self.topic = controller + '/target_frame'
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.publisher = node.create_publisher(PoseStamped, self.topic, qos)
        group = ReentrantCallbackGroup()
        self.list_client = node.create_client(ListControllers, manager + '/list_controllers', callback_group=group)
        self.switch_client = node.create_client(SwitchController, manager + '/switch_controller', callback_group=group)
        self.parameter_client = node.create_client(GetParameters, controller + '/get_parameters', callback_group=group)
        self.buffer = Buffer(node=node)
        self.listener = TransformListener(self.buffer, node)

    def start_timer(self, callback):
        from rclpy.clock import Clock, ClockType

        self.timer = self.node.create_timer(1/self.config.command_rate, callback,
                                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    @staticmethod
    def _call(client, request, deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not client.wait_for_service(timeout_sec=remaining):
            raise TimeoutError("controller_service_unavailable")
        future = client.call_async(request)
        event = Event()
        future.add_done_callback(lambda _: event.set())
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not event.wait(remaining):
            future.cancel()
            raise TimeoutError("controller_service_timeout")
        result = future.result()
        if result is None:
            raise RuntimeError("controller_service_failed")
        return result

    def check_ready(self, timeout):
        deadline = time.monotonic() + timeout
        state = self._call(self.list_client, self.ListControllers.Request(), deadline)
        active = any(c.name == self.config.controller_name and c.state == 'active' for c in state.controller)
        if not active:
            raise RuntimeError("cartesian_motion_controller_not_active")
        request = self.GetParameters.Request()
        request.names = ['robot_base_link', 'end_effector_link']
        values = self._call(self.parameter_client, request, deadline).values
        if len(values) != 2 or any(value.type != 4 for value in values):
            raise RuntimeError("controller_frame_parameters_unavailable")
        if values[0].string_value != self.config.base_frame:
            raise ValueError(f"controller_robot_base_link_mismatch:configured={self.config.base_frame},actual={values[0].string_value}")
        tip_frame = values[1].string_value
        if not tip_frame or tip_frame.startswith('/'):
            raise ValueError("invalid_controller_end_effector_link")
        if self.config.controller_tip_frame != 'auto' and tip_frame != self.config.controller_tip_frame:
            raise ValueError(f"controller_end_effector_link_mismatch:configured={self.config.controller_tip_frame},actual={tip_frame}")
        self.tip_frame = tip_frame
        self.node.get_logger().info(f"Cartesian frames: base={self.config.base_frame}, controller_tip={self.tip_frame}, grasp_center={self.config.tool_frame}")
        if not self.switch_client.wait_for_service(timeout_sec=max(0.0, deadline-time.monotonic())):
            raise RuntimeError("controller_stop_service_unavailable")
        while time.monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                try:
                    self.read_pose()
                    self.tip_to_tool()
                    return True
                except Exception as exc:
                    last_error = str(exc)
            else:
                last_error = "no_target_frame_subscriber"
            time.sleep(0.02)
        raise RuntimeError("controller_feedback_unavailable:" + locals().get('last_error', 'timeout'))

    def read_pose(self):
        from rclpy.time import Time

        transform = self.buffer.lookup_transform(self.config.base_frame, self.config.tool_frame, Time())
        stamp = transform.header.stamp.sec*1_000_000_000 + transform.header.stamp.nanosec
        now = self.node.get_clock().now().nanoseconds
        age = (now-stamp)/1e9
        if stamp <= 0 or age > self.config.feedback_timeout or age < -self.config.future_tolerance:
            raise RuntimeError("stale_robot_tf_feedback")
        p, q = transform.transform.translation, transform.transform.rotation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values):
            raise RuntimeError("invalid_robot_tf_feedback")
        return Pose(values[:3], values[3:], self.config.base_frame, self.config.tool_frame)

    def tip_to_tool(self):
        from rclpy.time import Time

        if self.tip_frame is None:
            raise RuntimeError("controller_tip_not_resolved")
        if self.config.tool_frame == self.tip_frame:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
        t = self.buffer.lookup_transform(self.tip_frame, self.config.tool_frame, Time()).transform
        p, q = t.translation, t.rotation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values):
            raise RuntimeError("invalid_tool_calibration_tf")
        return values[:3], values[3:]

    def publish(self, pose):
        if self.publisher.get_subscription_count() == 0:
            raise RuntimeError("target_controller_subscriber_lost")
        message = self.PoseStamped()
        message.header.frame_id = self.config.base_frame
        message.header.stamp = self.node.get_clock().now().to_msg()
        p, q = message.pose.position, message.pose.orientation
        p.x, p.y, p.z = pose.position
        q.x, q.y, q.z, q.w = pose.orientation
        self.publisher.publish(message)

    def deactivate(self, timeout):
        # A PoseStamped goal has no cancellation handle. Stop its controller,
        # rather than claiming that ceasing publication cancels the old goal.
        from rclpy.duration import Duration

        deadline = time.monotonic() + timeout
        request = self.SwitchController.Request()
        request.activate_controllers = []
        request.deactivate_controllers = [self.config.controller_name]
        request.strictness = self.SwitchController.Request.STRICT
        request.activate_asap = False
        request.timeout = Duration(seconds=max(0.01, timeout*0.8)).to_msg()
        if not self._call(self.switch_client, request, deadline).ok:
            raise RuntimeError("controller_deactivation_rejected")
        state = self._call(self.list_client, self.ListControllers.Request(), deadline)
        controller = next((c for c in state.controller if c.name == self.config.controller_name), None)
        if controller is None or controller.state == 'active':
            raise RuntimeError("controller_stop_not_confirmed")
        return True

"""ROS adapter with non-blocking, timestamp-specific TF waiting."""

import json
from pathlib import Path
import signal
from threading import Event
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener

from .config import load_config
from .pointcloud import make_cloud, read_xyz
from .rim_detection import locate_rim, transform_points, valid_points


def retained_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class RimNode(Node):
    def __init__(self):
        super().__init__("rim_locator")
        self.state = "initializing"
        self.status_pub = self.create_publisher(String, "~/status", retained_qos())
        self.pending = None
        self.tf_reason = "tf_unavailable"
        try:
            share = Path(get_package_share_directory("rim_locator"))
            path = self.declare_parameter("config_path", str(share / "config/config.json")).value
            self.config = load_config(path)
            self.publisher = self.create_publisher(PointStamped, self.config.output_topic, retained_qos())
            if self.config.publish_debug:
                self.cloud_pub = self.create_publisher(PointCloud2, "~/base_cloud", retained_qos())
                self.rim_pub = self.create_publisher(PointCloud2, "~/rim_candidates", retained_qos())
                self.local_pub = self.create_publisher(PointCloud2, "~/local_points", retained_qos())
            self.tf_buffer = Buffer(node=self)
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.subscription = self.create_subscription(PointCloud2, self.config.input_topic, self.on_cloud, retained_qos())
            self.wait_started = time.monotonic()
            self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.report("waiting_input")
        except Exception as exc:
            self.report("failed", failed_step="initializing", reason=str(exc))

    def report(self, status, **details):
        self.state = status
        payload = json.dumps({"status": status, **details}, ensure_ascii=False)
        self.status_pub.publish(String(data=payload))
        self.get_logger().info(payload)

    def on_cloud(self, cloud):
        if self.state != "waiting_input":
            return
        self.report("processing")
        try:
            if not cloud.header.frame_id.strip() or cloud.header.frame_id.startswith("/"):
                raise ValueError("invalid_frame_id")
            points = valid_points(read_xyz(cloud).reshape(-1, 3))
            if len(points) == 0:
                raise ValueError("empty_cloud")
            if len(points) < self.config.min_points:
                raise ValueError("insufficient_points")
            if cloud.header.frame_id == self.config.target_frame:
                self.finish(points, cloud.header.stamp)
                return
            # TF interprets a zero stamp as "latest", which is not the requested policy.
            if cloud.header.stamp.sec == 0 and cloud.header.stamp.nanosec == 0:
                raise ValueError("zero_stamp_cannot_select_acquisition_tf")
            self.pending = (cloud.header, points)
            self.tf_started = time.monotonic()
            self.report("waiting_tf")
        except Exception as exc:
            self.report("failed", failed_step="processing", reason=str(exc))

    def tick(self):
        if self.state == "waiting_input":
            if time.monotonic() - self.wait_started > self.config.input_timeout:
                self.report("failed", failed_step="waiting_input", reason="input_timeout")
            return
        if self.state != "waiting_tf":
            return
        if time.monotonic() - self.tf_started > self.config.tf_timeout:
            self.pending = None
            self.report("failed", failed_step="waiting_tf", reason="tf_unavailable", detail=self.tf_reason)
            return
        header, points = self.pending
        try:
            # Zero timeout: return to executor so TF subscriptions can continue.
            transform = self.tf_buffer.lookup_transform(
                self.config.target_frame, header.frame_id, Time.from_msg(header.stamp),
            ).transform
        except TransformException as exc:
            self.tf_reason = str(exc)
            return
        self.pending = None
        try:
            t, q = transform.translation, transform.rotation
            points = transform_points(points, [t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
            self.finish(points, header.stamp)
        except Exception as exc:
            self.report("failed", failed_step="processing", reason=str(exc))

    def finish(self, points, stamp):
        result = locate_rim(points, self.config)
        # The input stamp is used for the acquisition-time TF lookup, but a
        # freshly computed target must carry the current ROS time.  Camera
        # device clocks can lead/lag the host clock and would otherwise make
        # grasp_executor reject this new target as future or stale.
        output_stamp = self.get_clock().now().to_msg()
        header = Header(stamp=output_stamp, frame_id=self.config.target_frame)
        message = PointStamped(header=header)
        message.point.x, message.point.y, message.point.z = map(float, result.point)
        if self.config.publish_debug:
            self.cloud_pub.publish(make_cloud(header, points))
            self.rim_pub.publish(make_cloud(header, result.candidates))
            self.local_pub.publish(make_cloud(header, result.local_points))
        self.publisher.publish(message)
        self.report("success", point=result.point.tolist(), z_top=result.z_top,
                    candidate_count=len(result.candidates), local_count=len(result.local_points))


def main(args=None):
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor()
    shutdown_requested = Event()
    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def request_shutdown(signum, frame):
        shutdown_requested.set()
        executor.wake()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    node = None
    try:
        node = RimNode()
        executor.add_node(node)
        while not shutdown_requested.is_set():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        shutdown_requested.set()
    finally:
        if node is not None:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)

"""One-shot ROS 2 node. Successful output retains the sensor acquisition stamp."""

import json
from pathlib import Path
import signal
from threading import Event
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String

from .config import load_config, resolve_model_path
from .pointcloud import extract_object_points, make_cloud, read_xyz
from .segmentation import Segmenter


def retained_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class VisionNode(Node):
    def __init__(self):
        super().__init__("yolo_vision")
        self.state = "initializing"
        self.status_pub = self.create_publisher(String, "~/status", retained_qos())
        try:
            share = Path(get_package_share_directory("yolo_vision"))
            path = self.declare_parameter("config_path", str(share / "config/config.json")).value
            self.config = load_config(path)
            self.publisher = self.create_publisher(PointCloud2, self.config.output_topic, retained_qos())
            self.bridge = CvBridge()
            if self.config.publish_debug:
                self.mask_pub = self.create_publisher(Image, "~/mask", retained_qos())
                self.overlay_pub = self.create_publisher(Image, "~/overlay", retained_qos())
            self.segmenter = Segmenter(resolve_model_path(self.config, share), self.config)
            self.rgb_sub = Subscriber(self, Image, self.config.rgb_topic, qos_profile=qos_profile_sensor_data)
            self.cloud_sub = Subscriber(self, PointCloud2, self.config.pointcloud_topic, qos_profile=qos_profile_sensor_data)
            self.synchronizer = ApproximateTimeSynchronizer(
                [self.rgb_sub, self.cloud_sub], self.config.sync_queue_size,
                self.config.sync_slop, allow_headerless=False,
            )
            self.synchronizer.registerCallback(self.on_pair)
            self.wait_started = time.monotonic()
            self.timer = self.create_timer(0.1, self.check_timeout, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.report("ready")
        except Exception as exc:
            self.report("failed", failed_step="initializing", reason=str(exc))

    def report(self, status, **details):
        self.state = status
        payload = json.dumps({"status": status, **details}, ensure_ascii=False)
        self.status_pub.publish(String(data=payload))
        self.get_logger().info(payload)

    def check_timeout(self):
        if self.state == "ready" and time.monotonic() - self.wait_started > self.config.input_timeout:
            self.report("failed", failed_step="ready", reason="input_timeout")

    def on_pair(self, image, cloud):
        if self.state != "ready":
            return
        self.report("processing")  # Lock the job BEFORE any inference.
        try:
            if not cloud.header.frame_id.strip():
                raise ValueError("empty_frame_id")
            shape = (image.height, image.width)
            if shape != (cloud.height, cloud.width) or shape != (self.config.expected_height, self.config.expected_width):
                raise ValueError("dimension_mismatch")
            if cloud.height <= 1:
                raise ValueError("cloud_not_organized")
            bgr = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            result = self.segmenter.predict(bgr)
            points = extract_object_points(read_xyz(cloud), result.mask, self.config.min_points)
            output = make_cloud(cloud.header, points)
            if self.config.publish_debug:
                mask_message = self.bridge.cv2_to_imgmsg(result.mask.astype(np.uint8) * 255, encoding="mono8")
                mask_message.header = image.header
                overlay = bgr.copy()
                overlay[result.mask] = (0.5 * overlay[result.mask] + np.array([0, 127, 0])).astype(np.uint8)
                overlay_message = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
                overlay_message.header = image.header
                self.mask_pub.publish(mask_message)
                self.overlay_pub.publish(overlay_message)
            self.publisher.publish(output)
            self.report("success", point_count=len(points), class_name=result.class_name,
                        confidence=result.confidence)
        except Exception as exc:
            reason = str(exc)
            if reason.startswith("no_target"):
                # A scene without a bowl is expected during live operation.
                # Keep consuming fresh synchronized pairs instead of making
                # the one-shot node terminally failed on the first miss.
                self.wait_started = time.monotonic()
                self.report("ready", reason=reason, retry=True)
            else:
                self.report("failed", failed_step="processing", reason=reason)


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
        node = VisionNode()
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

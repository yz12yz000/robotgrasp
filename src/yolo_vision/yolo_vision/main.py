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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String

from .config import load_config, resolve_model_path
from .pointcloud import extract_object_points, make_cloud, read_xyz, read_aligned_depth_xyz
from .segmentation import Segmenter
from .sensor_input import SerializedSensor, raw_subscriber
from .diagnostics import detection_debug


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
            self.inputs = {}
            sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            accept = lambda: self.state == "ready"
            on_error = lambda exc: self.report("failed", failed_step="camera_input", reason=str(exc))
            self.rgb_sub = raw_subscriber(self, Image, self.config.rgb_topic, sensor_qos, accept, on_error)
            spatial_type = Image if self.config.depth_image_topic else PointCloud2
            spatial_topic = self.config.depth_image_topic or self.config.pointcloud_topic
            self.cloud_sub = raw_subscriber(self, spatial_type, spatial_topic, sensor_qos, accept, on_error)
            self.rgb_sub.registerCallback(lambda message: self.observe_input("rgb", message))
            self.cloud_sub.registerCallback(lambda message: self.observe_input("cloud", message))
            inputs = [self.rgb_sub, self.cloud_sub]
            if self.config.depth_image_topic:
                self.info_sub = Subscriber(self, CameraInfo, self.config.depth_info_topic, qos_profile=sensor_qos)
                self.info_sub.registerCallback(lambda message: self.observe_input("depth_info", message))
                inputs.append(self.info_sub)
            self.synchronizer = ApproximateTimeSynchronizer(
                inputs, self.config.sync_queue_size,
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
        if self.state == "ready" and self.deadline_expired():
            self.report("failed", failed_step="ready", reason="input_timeout",
                        camera=self.camera_details())

    def observe_input(self, name, message):
        previous = self.inputs.get(name, {})
        self.inputs[name] = {"received": previous.get("received", 0) + 1,
                             "width": message.width, "height": message.height,
                             "frame_id": message.header.frame_id,
                             "stamp_ns": message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec}

    def camera_details(self):
        details = {"rgb_topic": self.config.rgb_topic,
                   "spatial_topic": self.config.depth_image_topic or self.config.pointcloud_topic,
                   "depth_info_topic": self.config.depth_info_topic if self.config.depth_image_topic else None,
                   "expected_size": [self.config.expected_width, self.config.expected_height],
                   "sync_slop": self.config.sync_slop, "inputs": getattr(self, "inputs", {})}
        inputs = details["inputs"]
        required = ("rgb", "cloud", "depth_info") if self.config.depth_image_topic else ("rgb", "cloud")
        missing = [name for name in required if name not in inputs]
        details["missing"] = missing
        if not missing:
            details["latest_stamp_delta_s"] = abs(inputs["rgb"]["stamp_ns"] - inputs["cloud"]["stamp_ns"]) / 1e9
        return details

    def deadline_expired(self):
        return time.monotonic() - self.wait_started >= self.config.input_timeout

    def on_pair(self, image, cloud, camera_info=None):
        if self.state != "ready":
            return
        if self.deadline_expired():
            self.report("failed", failed_step="ready", reason="input_timeout")
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
            now_ns = self.get_clock().now().nanoseconds
            stamps = [m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec for m in (image, cloud)]
            if image.header.frame_id != cloud.header.frame_id:
                raise ValueError("depth_not_registered_to_rgb_frame")
            if abs(stamps[0] - stamps[1]) > self.config.sync_slop * 1e9:
                raise ValueError("rgb_depth_not_synchronized")
            if any(s <= 0 or not -0.1 <= (now_ns - s)/1e9 <= self.config.max_source_age for s in stamps):
                raise ValueError("source_timestamp_stale_or_future")
            if self.config.depth_image_topic:
                if camera_info is None:
                    raise ValueError("depth_camera_info_missing")
                info_stamp = camera_info.header.stamp.sec * 1_000_000_000 + camera_info.header.stamp.nanosec
                if abs(info_stamp - stamps[1]) > self.config.sync_slop * 1e9:
                    raise ValueError("depth_camera_info_not_synchronized")
            if isinstance(image, SerializedSensor):
                image = image.decode()
            if isinstance(cloud, SerializedSensor):
                cloud = cloud.decode()
            bgr = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            results = self.segmenter.predict_all(bgr)
            if self.deadline_expired():
                raise TimeoutError("input_timeout")
            xyz = (read_aligned_depth_xyz(cloud, camera_info) if self.config.depth_image_topic
                   else read_xyz(cloud))
            if not results:
                if self.deadline_expired():
                    raise TimeoutError("input_timeout")
                self.report("no_targets", detected_count=0, object_count=0,
                            reason="empty_detection", source_stamp_ns=stamps[1])
                return
            all_points, all_ids, accepted, rejected = [], [], [], []
            occupied = np.zeros(xyz.shape[:2], dtype=bool)
            for object_id, result in enumerate(results):
                if not isinstance(result.mask, np.ndarray) or result.mask.dtype != np.bool_:
                    raise ValueError("mask_must_be_boolean")
                if result.mask.shape != xyz.shape[:2]:
                    raise ValueError("dimension_mismatch")
                # Highest confidence wins overlaps; confidence never sets grasp order.
                mask = result.mask & ~occupied
                occupied |= result.mask
                mask_pixels = int(mask.sum())
                valid_count = int((mask & np.isfinite(xyz).all(axis=2) & (xyz[:, :, 2] > 0)).sum())
                depth_details = dict(mask_pixels=mask_pixels, valid_depth_points=valid_count,
                                     valid_depth_ratio=valid_count/max(1, mask_pixels))
                try:
                    points = extract_object_points(xyz, mask, self.config.min_points)
                except ValueError as exc:
                    if str(exc) != "insufficient_pointcloud":
                        raise
                    rejected.append({"object_id": object_id, "reason": str(exc), **depth_details})
                    continue
                all_points.append(points)
                all_ids.append(np.full(len(points), object_id, dtype=np.uint32))
                accepted.append({"object_id": object_id, "point_count": len(points),
                                 "confidence": result.confidence, **depth_details})
            if self.config.publish_debug:
                combined_mask, overlay = detection_debug(bgr, results, xyz)
                mask_message = self.bridge.cv2_to_imgmsg(combined_mask, encoding="mono8")
                mask_message.header = image.header
                overlay_message = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
                overlay_message.header = image.header
                if self.deadline_expired():
                    raise TimeoutError("input_timeout")
                self.mask_pub.publish(mask_message)
                self.overlay_pub.publish(overlay_message)
            if self.deadline_expired():
                raise TimeoutError("input_timeout")
            if not all_points:
                self.report("failed", failed_step="processing", reason="no_valid_depth_for_detected_targets",
                            detected_count=len(results), object_count=0, rejected=rejected,
                            source_stamp_ns=stamps[1])
                return
            output = make_cloud(cloud.header, np.concatenate(all_points), np.concatenate(all_ids))
            self.publisher.publish(output)
            self.report("success", detected_count=len(results), object_count=len(accepted), objects=accepted,
                        class_name=self.config.target_class, rejected=rejected,
                        source_stamp_ns=cloud.header.stamp.sec*1_000_000_000+cloud.header.stamp.nanosec,
                        rgb_depth_delta_s=abs((image.header.stamp.sec-cloud.header.stamp.sec) +
                                              (image.header.stamp.nanosec-cloud.header.stamp.nanosec)/1e9))
        except Exception as exc:
            self.report("failed", failed_step="processing", reason=str(exc))


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

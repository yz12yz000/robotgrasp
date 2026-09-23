"""ROS adapter with non-blocking, timestamp-specific TF waiting."""

import json
from pathlib import Path
import signal
from threading import Event
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseArray
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener

from .config import load_config
from .pointcloud import make_cloud, read_xyz_object_ids
from .rim_detection import locate_grasp_pose, transform_points, valid_points


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
            self.poses_publisher = self.create_publisher(PoseArray, self.config.poses_topic, retained_qos())
            if self.config.publish_debug:
                self.cloud_pub = self.create_publisher(PointCloud2, "~/base_cloud", retained_qos())
                self.rim_pub = self.create_publisher(PointCloud2, "~/rim_candidates", retained_qos())
                self.local_pub = self.create_publisher(PointCloud2, "~/local_points", retained_qos())
                self.wall_pub = self.create_publisher(PointCloud2, "~/wall_points", retained_qos())
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
            points, object_ids = read_xyz_object_ids(cloud)
            points = valid_points(points)
            if len(points) == 0:
                raise ValueError("empty_cloud")
            if len(points) < self.config.min_points:
                raise ValueError("insufficient_points")
            if len(object_ids) != len(points):
                raise ValueError("object_id_length_mismatch")
            if cloud.header.frame_id == self.config.target_frame:
                self.finish(points, object_ids, cloud.header.stamp)
                return
            # TF interprets a zero stamp as "latest", which is not the requested policy.
            if cloud.header.stamp.sec == 0 and cloud.header.stamp.nanosec == 0:
                raise ValueError("zero_stamp_cannot_select_acquisition_tf")
            self.pending = (cloud.header, points, object_ids)
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
        header, points, object_ids = self.pending
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
            self.finish(points, object_ids, header.stamp)
        except Exception as exc:
            self.report("failed", failed_step="processing", reason=str(exc))

    def finish(self, points, object_ids, stamp):
        object_ids = object_ids.reshape(-1)
        if len(points) != len(object_ids):
            raise ValueError("object_id_length_mismatch")
        header = Header(stamp=stamp, frame_id=self.config.target_frame)
        # Failed localization must remain inspectable, with instance IDs intact.
        if self.config.publish_debug:
            self.cloud_pub.publish(make_cloud(header, points, object_ids))
        results = []
        rejected = []
        for object_id in sorted(set(int(v) for v in object_ids)):
            object_points = points[object_ids == object_id]
            if len(object_points) < self.config.min_points:
                rejected.append({"object_id": object_id, "reason": "insufficient_points"})
                continue
            try:
                result = locate_grasp_pose(object_points, self.config)
            except ValueError as exc:
                detail = {"object_id": object_id, "reason": str(exc)}
                if getattr(exc, 'diagnostics', None):
                    detail['geometry'] = exc.diagnostics
                rejected.append(detail)
                continue
            if not self.config.min_output_z <= float(result.point[2]) <= self.config.max_output_z:
                self.get_logger().warning(f"object_{object_id}_rejected:output_z={float(result.point[2]):.4f}")
                rejected.append({"object_id": object_id, "reason": "output_z_outside_workspace",
                                 "z": float(result.point[2])})
                continue
            distance = float(np.hypot(result.point[0], result.point[1]))
            results.append((distance, object_id, result))
        if not results:
            self.report("failed", failed_step="processing", reason="pose_not_found:no_valid_object",
                        object_count=0, rejected=rejected)
            return
        results.sort(key=lambda item: (item[0], item[1]))
        # Preserve observation time so stale retained detections cannot be
        # relabelled as fresh targets by restarting this node.
        message = PointStamped(header=header)
        message.point.x, message.point.y, message.point.z = map(float, results[0][2].point)
        poses = PoseArray(header=header)
        for _, _, result in results:
            from geometry_msgs.msg import Pose
            pose = Pose()
            poses.poses.append(pose)
            pose.position.x, pose.position.y, pose.position.z = map(float, result.point)
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = result.orientation
        self.poses_publisher.publish(poses)
        if self.config.publish_debug:
            self.rim_pub.publish(make_cloud(header, np.concatenate([r[2].candidates for r in results])))
            self.local_pub.publish(make_cloud(header, np.concatenate([r[2].local_points for r in results])))
            self.wall_pub.publish(make_cloud(header, np.concatenate([r[2].wall_points for r in results]),
                                            np.concatenate([np.full(len(r[2].wall_points), r[1], dtype=np.uint32)
                                                            for r in results])))
        objects = [{"object_id": object_id, "distance": distance,
                    "xyz": [float(v) for v in result.point],
                    "rpy": [float(v) for v in result.rpy],
                    "candidate_count": len(result.candidates),
                    "local_count": len(result.local_points),
                    "geometry": result.diagnostics}
                   for distance, object_id, result in results]
        self.publisher.publish(message)
        self.report("success", source_stamp_ns=stamp.sec*1_000_000_000+stamp.nanosec, object_count=len(results), objects=objects, rejected=rejected,
                    ordered_object_ids=[x[1] for x in results])


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

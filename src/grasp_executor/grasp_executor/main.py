"""ROS node: executor keeps spinning while a worker waits for hardware results."""

from dataclasses import asdict, replace
import json
from pathlib import Path
from queue import Empty, Queue
import signal
from threading import Event, Thread
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .config import load_config
from .grasp import build_approach_pose, build_grasp_pose, run_task, validate_target
from .interfaces import create_interfaces


def retained_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class GraspNode(Node):
    def __init__(self):
        super().__init__("grasp_executor")
        self.state = "initializing"
        self.events = Queue()
        self.cancel = Event()
        self.worker = None
        self.interfaces = None
        self.status_pub = self.create_publisher(String, "~/status", retained_qos())
        try:
            share = Path(get_package_share_directory("grasp_executor"))
            path = self.declare_parameter("config_path", str(share / "config/config.json")).value
            self.config = load_config(path)
            self.config = replace(self.config, plan_only=self.declare_parameter("plan_only", self.config.plan_only).value)
            self.interfaces = create_interfaces(self.config, self)
            if self.config.publish_debug:
                self.approach_pub = self.create_publisher(PoseStamped, "~/approach_pose", retained_qos())
                self.grasp_pub = self.create_publisher(PoseStamped, "~/grasp_pose", retained_qos())
            self.subscription = self.create_subscription(PointStamped, self.config.input_topic, self.on_target, retained_qos())
            self.wait_started = time.monotonic()
            self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.report("waiting_target", plan_only=self.config.plan_only, max_target_age=self.config.max_target_age)
        except Exception as exc:
            self.report("failed", failed_step="initializing", reason=str(exc))

    def report(self, status, **details):
        self.state = status
        details.setdefault("backend", "moveit")
        details.setdefault("plan_only", self.config.plan_only if hasattr(self, "config") else True)
        if status == "success":
            details["grasp_verified"] = False
            details["gripper_feedback"] = "command_exit_and_settle_only"
        payload = json.dumps({"status": status, **details}, ensure_ascii=False)
        self.status_pub.publish(String(data=payload))
        # rclpy/rcutils requires a logger name to keep one severity level;
        # alternating info/error calls can raise during error handling.
        self.get_logger().info(payload)

    def tick(self):
        while True:
            try:
                event = self.events.get_nowait()
            except Empty:
                break
            self.report(**event)
        if self.state == "waiting_target" and time.monotonic() - self.wait_started > self.config.input_timeout:
            self.report("failed", failed_step="waiting_target", reason="input_timeout")

    @staticmethod
    def pose_message(pose, stamp):
        message = PoseStamped()
        message.header.frame_id = pose.frame_id
        message.header.stamp = stamp
        message.pose.position.x, message.pose.position.y, message.pose.position.z = pose.position
        q = message.pose.orientation
        q.x, q.y, q.z, q.w = pose.orientation
        return message

    def on_target(self, message):
        if self.state != "waiting_target":
            return
        self.report("validating")
        try:
            point = (message.point.x, message.point.y, message.point.z)
            stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            validate_target(message.header.frame_id, point, stamp_ns, self.get_clock().now().nanoseconds, self.config)
            grasp_pose = build_grasp_pose(point, self.config)
            approach_pose = build_approach_pose(grasp_pose, self.config)
            if self.config.publish_debug:
                self.approach_pub.publish(self.pose_message(approach_pose, message.header.stamp))
                self.grasp_pub.publish(self.pose_message(grasp_pose, message.header.stamp))
            self.worker = Thread(target=self.run_job, args=(point, stamp_ns), daemon=True)
            self.worker.start()
        except Exception as exc:
            # Invalid input has not submitted a motion command.
            self.wait_started = time.monotonic()
            self.report("waiting_target", failed_step="validating", reason=str(exc), retry=True)

    def run_job(self, point, stamp_ns):
        result = run_task(
            point, stamp_ns, lambda: self.get_clock().now().nanoseconds,
            self.interfaces, self.config,
            on_state=lambda status: self.events.put({"status": status}), cancel=self.cancel,
        )
        self.events.put(asdict(result))

    def close(self):
        self.cancel.set()
        if self.worker is not None and self.worker.is_alive():
            # Worker observes cancellation, then attempts stop_motion with its own deadline.
            self.worker.join(timeout=self.config.stop_timeout + 0.5)
            if self.worker.is_alive():
                self.get_logger().info("worker_shutdown_timeout: check robot stop feedback")
        if hasattr(self, "timer") and rclpy.ok():
            self.tick()
        if self.interfaces is not None:
            self.interfaces.close()


def main(args=None):
    # Keep ROS alive during Ctrl+C cleanup so an adapter can send a stop request.
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    shutdown_requested = Event()

    def request_shutdown(signum, frame):
        shutdown_requested.set()
        executor.wake()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = GraspNode()
        executor.add_node(node)
        while not shutdown_requested.is_set():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        # Ignore repeated SIGINT while cancellation and ROS cleanup complete.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    finally:
        if node is not None:
            node.cancel.set()
            # Continue serving action/service responses during bounded cancellation.
            limit = time.monotonic() + (node.config.stop_timeout + 0.5 if hasattr(node, "config") else 0.5)
            while node.worker is not None and node.worker.is_alive() and time.monotonic() < limit and rclpy.ok():
                executor.spin_once(timeout_sec=0.02)
            node.close()
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)

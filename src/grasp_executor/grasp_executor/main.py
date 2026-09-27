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
from geometry_msgs.msg import PointStamped, Pose, PoseArray, PoseStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .config import load_config
from .grasp import (Pose as GraspPose, build_approach_pose, build_grasp_pose,
                    run_batch_task, run_task, validate_target)
from .interfaces import create_interfaces
from .candidates import run_candidate_group_task
from .process_guard import supervisor_alive
from rim_locator.candidate_protocol import decode_candidate_batch


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
        self.task_id = self.declare_parameter("task_id", "").value
        self.observation_id = self.declare_parameter("observation_id", "").value
        self.source_not_before_ns = self.declare_parameter("source_not_before_ns", 0).value
        self.task_deadline_monotonic_ns = self.declare_parameter("task_deadline_monotonic_ns", 0).value
        self.supervisor_pid = self.declare_parameter("supervisor_pid", 0).value
        self.supervisor_start_ticks = self.declare_parameter("supervisor_start_ticks", 0).value
        self.supervisor_lost = False
        self.deadline_expired = False
        self.status_pub = self.create_publisher(String, "~/status", retained_qos())
        try:
            share = Path(get_package_share_directory("grasp_executor"))
            path = self.declare_parameter("config_path", str(share / "config/config.json")).value
            self.config = load_config(path)
            self.config = replace(self.config,
                plan_only=self.declare_parameter("plan_only", self.config.plan_only).value,
                task_preview_topic=self.declare_parameter("task_preview_topic", self.config.task_preview_topic).value)
            for name in ("source_not_before_ns", "task_deadline_monotonic_ns", "supervisor_pid", "supervisor_start_ticks"):
                if type(getattr(self, name)) is not int or getattr(self, name) < 0:
                    raise ValueError("invalid_parameter:" + name)
            if bool(self.supervisor_pid) != bool(self.supervisor_start_ticks):
                raise ValueError("incomplete_supervisor_identity")
            if not supervisor_alive(self.supervisor_pid, self.supervisor_start_ticks):
                raise RuntimeError("supervisor_lost")
            if self.config.input_mode == "candidate_groups" and (not self.task_id or not self.observation_id):
                raise ValueError("candidate_context_required")
            self.interfaces = create_interfaces(self.config, self)
            if self.config.publish_debug:
                self.approach_pub = self.create_publisher(PoseStamped, "~/approach_pose", retained_qos())
                self.grasp_pub = self.create_publisher(PoseStamped, "~/grasp_pose", retained_qos())
                self.place_ready_pub = self.create_publisher(PoseStamped, "~/place_ready_pose", retained_qos())
                self.place_pub = self.create_publisher(PoseStamped, "~/place_pose", retained_qos())
            callback, message_type = {
                "point": (self.on_target, PointStamped),
                "pose_array": (self.on_pose_array, PoseArray),
                "candidate_groups": (self.on_candidate_groups, String),
            }[self.config.input_mode]
            self.subscription = self.create_subscription(message_type, self.config.input_topic, callback, retained_qos())
            self.wait_started = time.monotonic()
            self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.report("waiting_target", plan_only=self.config.plan_only, max_target_age=self.config.max_target_age)
        except Exception as exc:
            self.report("failed", failed_step="initializing", reason=str(exc))

    def report(self, status, **details):
        self.state = status
        details.setdefault("task_id", self.task_id)
        details.setdefault("observation_id", self.observation_id)
        details.setdefault("backend", "moveit")
        if self.supervisor_lost and status == "failed" and details.get("reason") == "cancelled":
            details["reason"] = "supervisor_lost"
        if self.deadline_expired and status == "failed" and details.get("reason") == "cancelled":
            details["reason"] = "task_deadline_exceeded"
        details.setdefault("plan_only", self.config.plan_only if hasattr(self, "config") else True)
        if status == "success":
            details["grasp_verified"] = False
            details["placement_verified"] = False
            details["gripper_feedback"] = "command_exit_and_settle_only"
        payload = json.dumps({"status": status, **details}, ensure_ascii=False)
        self.status_pub.publish(String(data=payload))
        # rclpy/rcutils requires a logger name to keep one severity level;
        # alternating info/error calls can raise during error handling.
        self.get_logger().info(payload)

    def check_supervisor(self):
        if supervisor_alive(getattr(self, "supervisor_pid", 0), getattr(self, "supervisor_start_ticks", 0)):
            return True
        self.supervisor_lost = True
        self.cancel.set()  # Existing worker performs the bounded stop procedure.
        if self.worker is None:
            self.report("failed", failed_step="waiting_target", reason="supervisor_lost",
                        command_attempted=False)
        return False

    def tick(self):
        terminal = ("success", "planned", "failed", "no_targets", "candidates_exhausted")
        if self.state not in terminal:
            self.check_supervisor()
        if (self.task_deadline_monotonic_ns and time.monotonic_ns() >= self.task_deadline_monotonic_ns
                and self.state not in terminal):
            self.deadline_expired = True
            self.cancel.set()
            if self.worker is None:
                self.report("failed", failed_step="waiting_target", reason="task_deadline_exceeded")
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
        if not GraspNode.check_supervisor(self):
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

    @staticmethod
    def grasp_pose_from_ros(message_pose, config):
        q = message_pose.orientation
        return GraspPose((float(message_pose.position.x), float(message_pose.position.y),
                          float(message_pose.position.z)),
                         (float(q.x), float(q.y), float(q.z), float(q.w)),
                         config.base_frame, config.tool_frame)

    def on_pose_array(self, message):
        if self.state != "waiting_target":
            return
        if not GraspNode.check_supervisor(self):
            return
        self.report("validating")
        try:
            stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            if message.header.frame_id != self.config.base_frame:
                raise ValueError("frame_mismatch")
            poses = [self.grasp_pose_from_ros(pose, self.config) for pose in message.poses]
            self.worker = Thread(target=self.run_batch_job, args=(poses, stamp_ns), daemon=True)
            self.worker.start()
        except Exception as exc:
            self.wait_started = time.monotonic()
            self.report("waiting_target", failed_step="validating", reason=str(exc), retry=True)

    def on_candidate_groups(self, message):
        if self.state != "waiting_target":
            return
        if not GraspNode.check_supervisor(self):
            return
        self.report("validating")
        try:
            batch = decode_candidate_batch(message.data, expected_task_id=self.task_id,
                expected_observation_id=self.observation_id, expected_frame_id=self.config.base_frame,
                expected_tool_frame=self.config.tool_frame, source_not_before_ns=self.source_not_before_ns)
            if not self.config.plan_only and len(batch["objects"]) != 1:
                raise ValueError("execution_requires_single_object_group")
            self.worker = Thread(target=self.run_candidate_job, args=(batch,), daemon=True)
            self.worker.start()
        except Exception as exc:
            self.report("failed", failed_step="validating", reason=str(exc), command_attempted=False)

    def run_candidate_job(self, batch):
        result = run_candidate_group_task(batch, lambda: self.get_clock().now().nanoseconds,
            self.interfaces, self.config, on_state=self.events.put, cancel=self.cancel,
            on_poses=self.publish_batch_poses, expected_task_id=self.task_id,
            expected_observation_id=self.observation_id, source_not_before_ns=self.source_not_before_ns,
            task_deadline_monotonic_ns=self.task_deadline_monotonic_ns)
        self.events.put(asdict(result))

    def run_job(self, point, stamp_ns):
        result = run_task(
            point, stamp_ns, lambda: self.get_clock().now().nanoseconds,
            self.interfaces, self.config,
            on_state=lambda status: self.events.put({"status": status}), cancel=self.cancel,
        )
        self.events.put(asdict(result))

    def publish_batch_poses(self, approach, grasp, ready, drop):
        if self.config.publish_debug:
            stamp = self.get_clock().now().to_msg()
            for publisher, pose in ((self.approach_pub, approach), (self.grasp_pub, grasp),
                                    (self.place_ready_pub, ready), (self.place_pub, drop)):
                publisher.publish(self.pose_message(pose, stamp))

    def run_batch_job(self, poses, stamp_ns):
        result = run_batch_task(
            poses, stamp_ns, lambda: self.get_clock().now().nanoseconds,
            self.interfaces, self.config,
            on_state=lambda status: self.events.put({"status": status}), cancel=self.cancel,
            on_poses=self.publish_batch_poses,
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

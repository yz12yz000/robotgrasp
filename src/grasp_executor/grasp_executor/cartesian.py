"""Cartesian reference generation and feedback checks, independent of ROS.

This generates a straight Cartesian reference; it does not replace a trajectory
planner or promise a mathematically exact physical path. Actual tracking is
checked against configured tolerances using measured TF feedback.
"""

import math
from threading import RLock
import time

from .grasp import Pose, check_workspace


def normalized(q):
    norm = math.hypot(*q)
    if len(q) != 4 or not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid_feedback_quaternion")
    return tuple(x / norm for x in q)


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X + x*W + y*Z - z*Y, w*Y - x*Z + y*W + z*X,
            w*Z + x*Y - y*X + z*W, w*W - x*X - y*Y - z*Z)


def rotate(q, v):
    q = normalized(q)
    return multiply(multiply(q, (*v, 0.0)), (-q[0], -q[1], -q[2], q[3]))[:3]


def pose_error(a, b):
    distance = math.dist(a.position, b.position)
    dot = abs(sum(x*y for x, y in zip(normalized(a.orientation), normalized(b.orientation))))
    return distance, 2 * math.acos(min(1.0, dot))


def interpolate(start, goal, fraction):
    fraction = max(0.0, min(1.0, fraction))
    a, b = normalized(start.orientation), normalized(goal.orientation)
    dot = sum(x*y for x, y in zip(a, b))
    if dot < 0:
        b, dot = tuple(-x for x in b), -dot
    if dot > 0.9995:
        q = normalized(tuple(x + fraction*(y-x) for x, y in zip(a, b)))
    else:
        angle = math.acos(min(1.0, dot))
        q = tuple((math.sin((1-fraction)*angle)*x + math.sin(fraction*angle)*y) / math.sin(angle)
                  for x, y in zip(a, b))
    return Pose(tuple(x + fraction*(y-x) for x, y in zip(start.position, goal.position)),
                q, goal.frame_id, goal.tool_frame)


def tool_target_to_tip(target, tip_to_tool, tip_frame):
    """T_base_tip = T_base_tool * inverse(T_tip_tool), never a raw XYZ offset."""
    p, q = tip_to_tool
    q = normalized(q)
    inverse_q = (-q[0], -q[1], -q[2], q[3])
    tip_q = normalized(multiply(normalized(target.orientation), inverse_q))
    offset = rotate(tip_q, p)
    return Pose(tuple(x - y for x, y in zip(target.position, offset)), tip_q,
                target.frame_id, tip_frame)


class CartesianArm:
    def __init__(self, transport, config, clock=time.monotonic):
        self.transport, self.config, self.clock = transport, config, clock
        self.lock = RLock()
        self.stopped = False
        self.commanded = False
        self.ready = False
        self.goal = None
        self.error = None
        self.reached = False
        self.tip_to_tool = None

    def check_ready(self, timeout):
        self.transport.check_ready(timeout)
        tip_to_tool = self.transport.tip_to_tool()
        actual = self.transport.read_pose()
        self._validate_pose(actual)
        with self.lock:
            if self.stopped:
                raise RuntimeError("cancelled")
            self.tip_to_tool = tip_to_tool
            self.ready = True
        return True

    def _validate_pose(self, pose):
        if pose.frame_id != self.config.base_frame or pose.tool_frame != self.config.tool_frame:
            raise ValueError("feedback_frame_mismatch")
        if len(pose.position) != 3 or not all(math.isfinite(v) for v in pose.position):
            raise ValueError("invalid_feedback_position")
        normalized(pose.orientation)

    def _begin(self, pose, timeout, linear):
        with self.lock:
            if self.stopped:
                raise RuntimeError("cancelled")
            if self.error:
                raise RuntimeError(self.error)
            if not self.ready:
                raise RuntimeError("controller_not_ready")
            self._validate_pose(pose)
            check_workspace(pose, self.config)
            actual = self.transport.read_pose()
            self._validate_pose(actual)
            # Both ends of the reference must lie inside the configured box.
            check_workspace(actual, self.config)
            start = actual
            if linear:
                if self.goal is None or not self.reached:
                    raise ValueError("approach_not_reached")
                distance, angle = pose_error(actual, self.goal)
                if distance > self.config.position_tolerance or angle > self.config.orientation_tolerance:
                    raise ValueError("arm_moved_after_approach")
                if (math.dist(pose.position[:2], self.goal.position[:2]) > 1e-9
                        or pose_error(pose, self.goal)[1] > 1e-7
                        or pose.position[2] >= self.goal.position[2]):
                    raise ValueError("descent_must_be_vertical")
                start = self.goal  # Commanded XY and orientation stay exactly constant.
            distance, angle = pose_error(start, pose)
            speed = self.config.descend_speed if linear else self.config.approach_speed
            duration = max(distance / speed, angle / self.config.angular_speed, 1/self.config.command_rate)
            if duration + self.config.settle_time >= timeout:
                raise ValueError("motion_timeout_too_short_for_reference")
            self.start, self.goal, self.reference = start, pose, start
            self.duration, self.elapsed = duration, 0.0
            self.last_tick = self.clock()
            self.linear = linear
            self.reached, self.settle_started = False, None
            self.deadline = self.clock() + timeout
        return True

    def move_to_pose(self, pose, timeout):
        return self._begin(pose, timeout, False)

    def move_linear(self, pose, timeout):
        return self._begin(pose, timeout, True)

    def tick(self):
        # Publishing and setting the stopped latch use the same lock: no late
        # target publication after stop, even if a worker timed out or cancelled.
        with self.lock:
            if self.stopped or self.goal is None or self.error:
                return
            try:
                actual = self.transport.read_pose()
                self._validate_pose(actual)
                distance, angle = pose_error(actual, self.reference)
                if distance > self.config.max_tracking_error or angle > self.config.max_tracking_angle:
                    raise RuntimeError("cartesian_tracking_error")
                if self.linear and math.dist(actual.position[:2], self.goal.position[:2]) > self.config.linear_lateral_tolerance:
                    raise RuntimeError("descent_lateral_error")
                now = self.clock()
                if not self.reached and now >= self.deadline:
                    raise TimeoutError("motion_timeout")
                dt = min(max(0.0, now-self.last_tick), 2/self.config.command_rate)
                self.last_tick = now
                self.elapsed = min(self.duration, self.elapsed + dt)
                self.reference = interpolate(self.start, self.goal, self.elapsed/self.duration)
                target = tool_target_to_tip(self.reference, self.tip_to_tool, self.transport.tip_frame)
                self.commanded = True  # Publish can fail after handing off a command.
                self.transport.publish(target)
                distance, angle = pose_error(actual, self.goal)
                within = distance <= self.config.position_tolerance and angle <= self.config.orientation_tolerance
                if self.elapsed >= self.duration and within:
                    if self.settle_started is None:
                        self.settle_started = now
                    self.reached = now-self.settle_started >= self.config.settle_time
                else:
                    self.settle_started = None
                    self.reached = False
            except Exception as exc:
                self.error = str(exc)

    def is_pose_reached(self, timeout):
        with self.lock:
            if self.error:
                raise RuntimeError(self.error)
            if self.stopped:
                raise RuntimeError("cancelled")
            return self.reached

    def stop_motion(self, timeout):
        with self.lock:
            self.stopped = True
            self.goal = None
            commanded = self.commanded
        # No command sent by this instance => no controller switching.
        if not commanded:
            return True
        return self.transport.deactivate(timeout)

    def finish(self):
        """Stop streaming after a completed job; controller holds its final target."""
        with self.lock:
            if self.error or not self.reached:
                raise RuntimeError(self.error or "final_pose_not_reached")
            self.goal = None

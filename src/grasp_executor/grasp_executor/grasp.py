"""ROS-free pose generation, input validation and fixed execution state machine."""

from dataclasses import dataclass, replace
import hashlib
import json
import math
from pathlib import Path
from queue import Empty, Queue
from threading import Event, Thread
import time

from .config import vector


@dataclass(frozen=True)
class Pose:
    position: tuple
    orientation: tuple
    frame_id: str
    tool_frame: str


@dataclass(frozen=True)
class ExecutionResult:
    status: str
    failed_step: str = ""
    reason: str = ""
    stop_error: str = ""
    object_index: int = -1
    object_count: int = 0
    completed_count: int = 0


def build_grasp_pose(rim_point, config):
    if len(rim_point) != 3 or not all(math.isfinite(v) for v in rim_point):
        raise ValueError("invalid_target_coordinates")
    offset = vector(config.grasp_offset, "xyz", "grasp_offset")
    position = tuple(float(a + b) for a, b in zip(rim_point, offset))
    q = vector(config.fixed_orientation, "xyzw", "fixed_orientation")
    norm = math.hypot(*q)
    pose = Pose(position, tuple(v / norm for v in q), config.base_frame, config.tool_frame)
    check_workspace(pose, config)
    return pose


def build_approach_pose(grasp_pose, config):
    x, y, z = grasp_pose.position
    pose = Pose((x, y, z + config.approach_height), grasp_pose.orientation,
                grasp_pose.frame_id, grasp_pose.tool_frame)
    check_workspace(pose, config)
    return pose


def apply_grasp_offset(pose, config):
    """Apply metres along base-frame XYZ once, preserving input orientation."""
    validate_pose(pose, config)
    offset = vector(config.grasp_offset, "xyz", "grasp_offset")
    adjusted = replace(pose, position=tuple(float(a + b) for a, b in zip(pose.position, offset)))
    check_workspace(adjusted, config)
    return adjusted


def build_place_ready_pose(config):
    position = vector(config.place_ready_position, "xyz", "place_ready_position")
    q = vector(config.place_ready_orientation, "xyzw", "place_ready_orientation")
    norm = math.hypot(*q)
    pose = Pose(position, tuple(v / norm for v in q), config.base_frame, config.tool_frame)
    check_workspace(pose, config)
    return pose


def build_place_drop_pose(ready_pose, config):
    x, y, z = ready_pose.position
    pose = Pose((x, y, z - config.place_drop_distance), ready_pose.orientation,
                ready_pose.frame_id, ready_pose.tool_frame)
    check_workspace(pose, config)
    return pose


def validate_pose(pose, config):
    if not isinstance(pose, Pose) or pose.frame_id != config.base_frame or pose.tool_frame != config.tool_frame:
        raise ValueError("pose_frame_mismatch")
    if len(pose.position) != 3 or len(pose.orientation) != 4:
        raise ValueError("invalid_pose_shape")
    if not all(math.isfinite(v) for v in pose.position + pose.orientation):
        raise ValueError("invalid_pose_values")
    norm = math.hypot(*pose.orientation)
    if norm < 1e-12 or abs(norm - 1.) > 0.02:
        raise ValueError("invalid_pose_orientation")
    check_workspace(pose, config)


def check_workspace(pose, config):
    low = vector(config.workspace_min, "xyz", "workspace_min")
    high = vector(config.workspace_max, "xyz", "workspace_max")
    bounded = (
        low[0] <= pose.position[0] <= high[0],
        low[1] <= pose.position[1] <= high[1],
        low[2] <= pose.position[2] <= high[2],
    )
    if any(not math.isfinite(v) for v in pose.position) or not all(bounded):
        raise ValueError("pose_outside_workspace")


def validate_target(frame_id, point, stamp_ns, now_ns, config):
    if frame_id != config.base_frame:
        raise ValueError("frame_mismatch")
    if len(point) != 3 or not all(math.isfinite(v) for v in point):
        raise ValueError("invalid_target_coordinates")
    if stamp_ns <= 0 or now_ns <= 0:
        raise ValueError("invalid_target_timestamp")
    age = (now_ns - stamp_ns) / 1_000_000_000
    if age > config.max_target_age:
        raise ValueError("stale_target")
    if age < -config.future_tolerance:
        raise ValueError("target_timestamp_in_future")


def claim_target(directory, frame_id, point, stamp_ns):
    """Atomic, persistent claim before real motion; never release on failure.

    Prevents re-executing the same retained target after a node restart. A new
    perception stamp constitutes a new task, not a guarantee of scene freshness.
    """
    directory = Path(directory).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    payload = json.dumps({"frame_id": frame_id, "point": list(point), "stamp_ns": stamp_ns}, sort_keys=True)
    digest = hashlib.sha256(payload.encode("utf-8")).hexdigest()
    try:
        with (directory / (digest + ".json")).open("x", encoding="utf-8") as stream:
            stream.write(payload)
            stream.flush()
            import os
            os.fsync(stream.fileno())
    except FileExistsError as exc:
        raise ValueError("target_already_claimed") from exc


def claim_batch(directory, poses, stamp_ns):
    payload = {"frame_id": poses[0].frame_id if poses else "", "stamp_ns": int(stamp_ns),
               "poses": [{"position": list(p.position), "orientation": list(p.orientation),
                          "tool_frame": p.tool_frame} for p in poses]}
    directory = Path(directory).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    digest = hashlib.sha256(encoded.encode("utf-8")).hexdigest()
    try:
        with (directory / (digest + ".json")).open("x", encoding="utf-8") as stream:
            stream.write(encoded)
            stream.flush()
            import os
            os.fsync(stream.fileno())
    except FileExistsError as exc:
        raise ValueError("batch_already_claimed") from exc


def bounded_call(function, timeout, cancel=None, timeout_reason="interface_timeout"):
    """Bound caller waiting, while the adapter must also honor its own timeout.

    Python cannot kill an in-flight SDK call. Real adapters must ensure timeout /
    stop cancels queued hardware work; this wrapper alone cannot provide that.
    """
    if timeout <= 0:
        raise TimeoutError(timeout_reason)
    if cancel is not None and cancel.is_set():
        raise RuntimeError("cancelled")
    mailbox = Queue(maxsize=1)

    def run():
        try:
            mailbox.put((True, function()))
        except Exception as exc:
            mailbox.put((False, exc))

    Thread(target=run, daemon=True).start()
    deadline = time.monotonic() + timeout
    while True:
        if cancel is not None and cancel.is_set():
            raise RuntimeError("cancelled")
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(timeout_reason)
        try:
            success, value = mailbox.get(timeout=min(remaining, 0.02))
        except Empty:
            continue
        if not success:
            raise value
        return value


def stop_safely(interfaces, timeout):
    try:
        result = bounded_call(lambda: interfaces.stop_motion(timeout), timeout, timeout_reason="stop_timeout")
        if result is not True:
            raise RuntimeError("stop_command_failed")
        return ""
    except Exception as exc:
        return str(exc)


def execute_grasp(approach_pose, grasp_pose, interfaces, config, on_state=None, cancel=None):
    cancel = cancel if cancel is not None else Event()
    on_state = on_state if on_state is not None else lambda state: None
    step = "validating"
    command_attempted = False

    def motion(method, pose):
        nonlocal command_attempted
        if cancel.is_set():
            raise RuntimeError("cancelled")
        # A submitted call may raise after partially commanding hardware.
        command_attempted = True
        deadline = time.monotonic() + config.motion_timeout
        accepted = bounded_call(lambda: method(pose, config.motion_timeout), config.motion_timeout,
                                cancel, "motion_timeout")
        if accepted is not True:
            raise RuntimeError("motion_command_rejected")
        while True:
            remaining = deadline - time.monotonic()
            reached = bounded_call(lambda: interfaces.is_pose_reached(remaining), remaining,
                                   cancel, "motion_timeout")
            if reached is True:
                return
            if reached is not False:
                raise RuntimeError("invalid_motion_feedback")
            if cancel.wait(min(config.poll_interval, max(0.0, deadline - time.monotonic()))):
                raise RuntimeError("cancelled")

    try:
        check_workspace(grasp_pose, config)
        check_workspace(approach_pose, config)
        if (approach_pose.frame_id != config.base_frame or grasp_pose.frame_id != config.base_frame
                or approach_pose.tool_frame != config.tool_frame or grasp_pose.tool_frame != config.tool_frame
                or approach_pose.position[:2] != grasp_pose.position[:2]
                or approach_pose.orientation != grasp_pose.orientation
                or approach_pose.position[2] <= grasp_pose.position[2]):
            raise ValueError("invalid_vertical_approach")
        step = "moving_to_approach"
        on_state(step)
        motion(interfaces.move_to_pose, approach_pose)
        step = "opening_gripper"
        on_state(step)
        if bounded_call(lambda: interfaces.open_gripper(config.gripper_timeout),
                        config.gripper_timeout, cancel, "gripper_timeout") is not True:
            raise RuntimeError("gripper_command_failed")
        step = "moving_to_grasp"
        on_state(step)
        motion(interfaces.move_linear, grasp_pose)
        step = "closing_gripper"
        on_state(step)
        if bounded_call(lambda: interfaces.close_gripper(config.gripper_timeout),
                        config.gripper_timeout, cancel, "gripper_timeout") is not True:
            raise RuntimeError("gripper_command_failed")
        step = "lifting_after_grasp"
        on_state(step)
        motion(interfaces.move_linear, approach_pose)
        if cancel.is_set():
            raise RuntimeError("cancelled")
        if callable(getattr(interfaces, "finish", None)):
            interfaces.finish()
        return ExecutionResult("success")
    except Exception as exc:
        stop_error = stop_safely(interfaces, config.stop_timeout) if command_attempted else ""
        return ExecutionResult("failed", step, str(exc), stop_error)


def execute_grasp_place(approach_pose, grasp_pose, place_ready, place_drop, interfaces, config,
                        on_state=None, cancel=None):
    """Execute one six-segment grasp/place sequence using prepared MoveIt plans."""
    cancel = cancel if cancel is not None else Event()
    on_state = on_state if on_state is not None else lambda state: None
    step = "validating"
    command_attempted = False

    def motion(method, pose):
        nonlocal command_attempted
        if cancel.is_set():
            raise RuntimeError("cancelled")
        command_attempted = True
        deadline = time.monotonic() + config.motion_timeout
        accepted = bounded_call(lambda: method(pose, config.motion_timeout), config.motion_timeout,
                                cancel, "motion_timeout")
        if accepted is not True:
            raise RuntimeError("motion_command_rejected")
        while True:
            remaining = deadline - time.monotonic()
            reached = bounded_call(lambda: interfaces.is_pose_reached(remaining), remaining,
                                   cancel, "motion_timeout")
            if reached is True:
                return
            if reached is not False:
                raise RuntimeError("invalid_motion_feedback")
            if cancel.wait(min(config.poll_interval, max(0.0, deadline - time.monotonic()))):
                raise RuntimeError("cancelled")

    def gripper(method):
        if bounded_call(lambda: method(config.gripper_timeout), config.gripper_timeout,
                        cancel, "gripper_timeout") is not True:
            raise RuntimeError("gripper_command_failed")

    try:
        for pose in (approach_pose, grasp_pose, place_ready, place_drop):
            validate_pose(pose, config)
        if (approach_pose.position[:2] != grasp_pose.position[:2]
                or approach_pose.orientation != grasp_pose.orientation
                or approach_pose.position[2] <= grasp_pose.position[2]):
            raise ValueError("invalid_vertical_approach")
        if (place_ready.position[:2] != place_drop.position[:2]
                or place_ready.orientation != place_drop.orientation
                or place_ready.position[2] <= place_drop.position[2]):
            raise ValueError("invalid_vertical_place")
        step = "moving_to_approach"; on_state(step); motion(interfaces.move_to_pose, approach_pose)
        step = "opening_gripper"; on_state(step); gripper(interfaces.open_gripper)
        step = "moving_to_grasp"; on_state(step); motion(interfaces.move_linear, grasp_pose)
        step = "closing_gripper"; on_state(step); gripper(interfaces.close_gripper)
        step = "lifting_after_grasp"; on_state(step); motion(interfaces.move_linear, approach_pose)
        step = "moving_to_place_ready"; on_state(step); motion(interfaces.move_to_pose, place_ready)
        step = "moving_to_place"; on_state(step); motion(interfaces.move_linear, place_drop)
        step = "holding_before_release"; on_state(step)
        deadline = time.monotonic() + config.place_dwell_time
        while time.monotonic() < deadline:
            if cancel.wait(min(config.poll_interval, max(0.0, deadline - time.monotonic()))):
                raise RuntimeError("cancelled")
        step = "releasing_gripper"; on_state(step); gripper(interfaces.open_gripper)
        if config.place_retreat:
            step = "retreating_from_place"; on_state(step); motion(interfaces.move_linear, place_ready)
        if callable(getattr(interfaces, "finish", None)):
            interfaces.finish()
        return ExecutionResult("success")
    except Exception as exc:
        stop_error = stop_safely(interfaces, config.stop_timeout) if command_attempted else ""
        return ExecutionResult("failed", step, str(exc), stop_error)


def run_task(point, stamp_ns, now_ns, interfaces, config, on_state=None, cancel=None):
    """Preflight never stops the arm: no task-owned motion has started yet."""
    cancel = cancel if cancel is not None else Event()
    on_state = on_state if on_state is not None else lambda state: None
    step = "preflight"
    try:
        validate_target(config.base_frame, point, stamp_ns, now_ns(), config)
        grasp_pose = build_grasp_pose(point, config)
        approach_pose = build_approach_pose(grasp_pose, config)
        ready = bounded_call(lambda: interfaces.check_ready(config.motion_timeout),
                             config.motion_timeout, cancel, "interface_readiness_timeout")
        if ready is not True:
            raise RuntimeError("interfaces_not_ready")
        step = "planning"
        on_state(step)
        planned = bounded_call(lambda: interfaces.prepare_plans(approach_pose, grasp_pose),
                               3 * config.planning_timeout, cancel, "planning_timeout")
        if planned is not True:
            raise RuntimeError("planning_failed")
        validate_target(config.base_frame, point, stamp_ns, now_ns(), config)
        if cancel.is_set():
            raise RuntimeError("cancelled")
        if config.plan_only:
            return ExecutionResult("planned")
        claim_target(config.execution_journal, config.base_frame, point, stamp_ns)
    except Exception as exc:
        # Includes duplicate target, missing TF, inactive controller and stale
        # input. Do not stop someone else's robot task.
        # Interrupt any planning call that outlived its outer timeout. No motion
        # has been dispatched, so do not stop someone else's controller.
        cancel.set()
        return ExecutionResult("failed", step, str(exc))
    return execute_grasp(approach_pose, grasp_pose, interfaces, config, on_state, cancel)


def run_batch_task(poses, stamp_ns, now_ns, interfaces, config, on_state=None, cancel=None, on_poses=None):
    """Plan and execute a frozen PoseArray from nearest to farthest."""
    if not poses:
        return ExecutionResult("no_targets", object_count=0, completed_count=0)
    cancel = cancel if cancel is not None else Event()
    on_state = on_state if on_state is not None else lambda state: None
    step = "preflight"
    index, completed, targets = -1, 0, []
    try:
        if not config.place_retreat:
            raise ValueError("batch_requires_place_retreat")
        original_targets = list(poses)
        targets = []
        for pose in original_targets:
            validate_pose(pose, config)
            validate_target(pose.frame_id, pose.position, stamp_ns, now_ns(), config)
            adjusted = apply_grasp_offset(pose, config)
            build_approach_pose(adjusted, config)
            targets.append(adjusted)
        # Journal identity stays tied to the original observation, including
        # its historical stable distance ordering, regardless of calibration.
        original_targets.sort(key=lambda p: math.hypot(*p.position[:2]))
        # Defensive sorting also covers PoseArrays sent by another publisher.
        targets.sort(key=lambda p: math.hypot(*p.position[:2]))
        if callable(getattr(interfaces, "begin_batch", None)):
            interfaces.begin_batch()
        ready = drop = None
        claimed = False
        for index, grasp_pose in enumerate(targets):
            if cancel.is_set():
                raise RuntimeError("cancelled")
            step = f"readiness_{index}"
            approach_pose = build_approach_pose(grasp_pose, config)
            available = bounded_call(lambda: interfaces.check_ready(config.motion_timeout),
                                     config.motion_timeout, cancel, "interface_readiness_timeout")
            if available is not True:
                raise RuntimeError("interfaces_not_ready")
            if ready is None:
                ready = (interfaces.resolve_place_pose()
                         if callable(getattr(interfaces, "resolve_place_pose", None)) else None)
                ready = ready if ready is not None else build_place_ready_pose(config)
                drop = build_place_drop_pose(ready, config)
                validate_pose(ready, config); validate_pose(drop, config)
            if on_poses:
                on_poses(approach_pose, grasp_pose, ready, drop)
            step = f"planning_{index}"; on_state(step)
            planned = bounded_call(lambda ap=approach_pose, gp=grasp_pose:
                                   interfaces.prepare_plans(ap, gp, ready, drop, ready),
                                   7 * config.planning_timeout, cancel, "planning_timeout")
            if planned is not True:
                raise RuntimeError("planning_failed")
            if cancel.is_set():
                raise RuntimeError("cancelled")
            if config.plan_only:
                continue
            if not claimed:
                validate_target(grasp_pose.frame_id, grasp_pose.position, stamp_ns, now_ns(), config)
                claim_batch(config.execution_journal, original_targets, stamp_ns)
                claimed = True
            result = execute_grasp_place(approach_pose, grasp_pose, ready, drop, interfaces,
                                         config, lambda state: on_state(f"object_{index}:{state}"), cancel)
            if result.status != "success":
                return replace(result, object_index=index, object_count=len(targets), completed_count=completed)
            completed += 1
        return ExecutionResult("planned" if config.plan_only else "success",
                               object_count=len(targets), completed_count=completed)
    except Exception as exc:
        # Cancel outstanding service workers on timeout as well as explicit
        # cancellation. No automatic release, retry, or next-object motion.
        cancel.set()
        return ExecutionResult("failed", step, str(exc), object_index=index,
                               object_count=len(targets), completed_count=completed)
    finally:
        if callable(getattr(interfaces, "end_batch", None)):
            interfaces.end_batch()

"""ROS-free scheduling of grouped contacts; retries never dispatch motion."""

from dataclasses import dataclass, field
import math
from threading import Event
import time

from rim_locator.candidate_protocol import validate_candidate_batch

from .grasp import (Pose, apply_grasp_offset, bounded_call, build_approach_pose,
                    build_place_drop_pose, build_place_ready_pose, claim_candidate_object,
                    execute_grasp_place, validate_pose, validate_target)
from .planning_errors import CandidatePlanRejected


@dataclass(frozen=True)
class CandidateResult:
    status: str
    task_id: str = ""
    observation_id: str = ""
    source_stamp_ns: int = 0
    object_id: int = -1
    candidate_id: int = -1
    object_count: int = 0
    completed_count: int = 0
    candidate_attempt_count: int = 0
    candidate_failures: list = field(default_factory=list)
    command_attempted: bool = False
    failed_step: str = ""
    reason: str = ""
    stop_error: str = ""


def run_candidate_group_task(batch, now_ns, interfaces, config, *, on_state=None,
                             on_poses=None, cancel=None, expected_task_id=None,
                             expected_observation_id=None, source_not_before_ns=0,
                             task_deadline_monotonic_ns=0):
    """Execute one object, or preview a sorted snapshot without moving anything.

    The same cancel Event belongs to the real adapter. A timeout or unexpected
    failure ends this task permanently; clearing that Event is never recovery.
    """
    cancel = cancel if cancel is not None else Event()
    on_state = on_state or (lambda event: None)
    context = dict(task_id="", observation_id="", source_stamp_ns=0,
                   object_count=0, object_id=-1, candidate_id=-1,
                   candidate_attempt_count=0, completed_count=0,
                   candidate_failures=[], command_attempted=False)
    step, started_batch = "validating", False
    task_deadline = (task_deadline_monotonic_ns / 1e9
                     if task_deadline_monotonic_ns else math.inf)

    def remaining(limit):
        if cancel.is_set():
            raise RuntimeError("cancelled")
        available = task_deadline - time.monotonic()
        if available <= 0:
            raise TimeoutError("task_deadline_exceeded")
        return min(limit, available)

    def event(status, **extra):
        snapshot = dict(context, candidate_failures=list(context["candidate_failures"]))
        on_state(dict(status=status, **snapshot, **extra))

    def result(status, **extra):
        return CandidateResult(status=status, **context, **extra)

    def discard():
        interfaces.discard_prepared_plans()

    try:
        batch = validate_candidate_batch(batch, expected_task_id=expected_task_id,
            expected_observation_id=expected_observation_id, expected_frame_id=config.base_frame,
            expected_tool_frame=config.tool_frame, source_not_before_ns=source_not_before_ns)
        context.update(task_id=batch["task_id"], observation_id=batch["observation_id"],
                       source_stamp_ns=batch["source_stamp_ns"], object_count=len(batch["objects"]))
        if not context["task_id"] or not context["observation_id"]:
            raise ValueError("candidate_context_required")
        if not config.plan_only and len(batch["objects"]) != 1:
            raise ValueError("execution_requires_single_object_group")
        if not config.place_retreat:
            raise ValueError("candidate_task_requires_place_retreat")
        remaining(config.motion_timeout)
        if not batch["objects"]:
            return result("no_targets")

        def pose_of(candidate):
            return Pose(tuple(candidate["position"]), tuple(candidate["orientation"]),
                        batch["frame_id"], batch["tool_frame"])

        def order(group):
            # Ranking uses the original preferred contact plus the existing
            # offset, even if that contact later fails a workspace check.
            position = group["candidates"][0]["position"]
            return math.hypot(position[0] + config.grasp_offset["x"],
                              position[1] + config.grasp_offset["y"])

        groups = sorted(batch["objects"], key=order)
        # Preserve the existing virtual endpoint only between preview objects.
        interfaces.begin_batch()
        started_batch = True
        for group in groups:
            context.update(object_id=group["object_id"], candidate_id=-1)
            step = "readiness"
            validate_target(batch["frame_id"], group["candidates"][0]["position"],
                            batch["source_stamp_ns"], now_ns(), config)
            timeout = remaining(config.motion_timeout)
            if bounded_call(lambda: interfaces.check_ready(timeout), timeout, cancel,
                            "interface_readiness_timeout") is not True:
                raise RuntimeError("interfaces_not_ready")
            ready = interfaces.resolve_place_pose()
            ready = ready if ready is not None else build_place_ready_pose(config)
            drop = build_place_drop_pose(ready, config)
            validate_pose(ready, config)
            validate_pose(drop, config)
            deadline = min(task_deadline, time.monotonic() + 7 * config.planning_timeout)
            selected = False
            for candidate in group["candidates"]:
                remaining(config.planning_timeout)
                if time.monotonic() >= deadline:
                    raise TimeoutError("candidate_planning_deadline")
                context["candidate_id"] = candidate["candidate_id"]
                step = "planning"
                discard()
                validate_target(batch["frame_id"], candidate["position"], batch["source_stamp_ns"],
                                now_ns(), config)
                context["candidate_attempt_count"] += 1
                event("planning_candidate")
                try:
                    try:
                        grasp = apply_grasp_offset(pose_of(candidate), config)
                        approach = build_approach_pose(grasp, config)
                    except ValueError as exc:
                        # An otherwise valid contact outside configured bounds
                        # is infeasible. Schema/frame/orientation faults remain fatal.
                        if str(exc) != "pose_outside_workspace":
                            raise
                        raise CandidatePlanRejected(str(exc), stage="workspace") from exc
                    planned = bounded_call(
                        lambda: interfaces.prepare_plans(approach, grasp, ready, drop, ready,
                                                         deadline=deadline),
                        deadline - time.monotonic(), cancel, "candidate_planning_deadline")
                    if planned is not True:
                        raise RuntimeError("invalid_planning_result")
                except CandidatePlanRejected as exc:
                    remaining(config.planning_timeout)
                    discard()
                    failure = dict(object_id=group["object_id"], candidate_id=candidate["candidate_id"],
                                   stage=exc.stage, reason=str(exc))
                    context["candidate_failures"].append(failure)
                    event("candidate_rejected")
                    continue
                remaining(config.planning_timeout)
                validate_target(batch["frame_id"], grasp.position, batch["source_stamp_ns"],
                                now_ns(), config)
                if on_poses:
                    on_poses(approach, grasp, ready, drop)
                selected = True
                if config.plan_only:
                    event("candidate_planned")
                    break
                step = "claiming"
                remaining(config.motion_timeout)
                validate_target(batch["frame_id"], grasp.position, batch["source_stamp_ns"],
                                now_ns(), config)
                claim_candidate_object(config.execution_journal, batch, group["object_id"], candidate)
                remaining(config.motion_timeout)
                step = "executing"
                context["command_attempted"] = True
                executed = execute_grasp_place(approach, grasp, ready, drop, interfaces, config,
                    on_state=lambda state: event(state), cancel=cancel)
                if executed.status == "success":
                    context["completed_count"] = 1
                return result(executed.status, failed_step=executed.failed_step,
                              reason=executed.reason, stop_error=executed.stop_error)
            if not selected:
                remaining(config.planning_timeout)
                discard()
                return result("candidates_exhausted", failed_step="planning",
                              reason="all_candidates_rejected")
        return result("planned")
    except Exception as exc:
        cancel.set()
        # Discard also invalidates a late planning worker's commit token.
        if not context["command_attempted"]:
            try:
                discard()
            except Exception:
                pass
        return result("failed", failed_step=step, reason=str(exc))
    finally:
        if started_batch:
            interfaces.end_batch()


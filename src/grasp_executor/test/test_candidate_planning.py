"""Test adapter transaction boundaries with real ROS message types, no nodes."""
from copy import deepcopy
from dataclasses import replace
from threading import Event, Thread
from types import SimpleNamespace as NS
import time

import pytest

from grasp_executor.grasp import (build_grasp_pose, build_approach_pose, build_place_ready_pose,
                                  build_place_drop_pose)
from grasp_executor.moveit_ros import Segment
from grasp_executor.planning_errors import CandidatePlanRejected
from test_grasp import completed, trajectory
from test_ik_selection import arm, state


def adapter():
    result = arm()
    result.measured = state([0.]*6)
    result._state = lambda **kwargs: deepcopy(result.measured)
    result._scene = result._valid = lambda *args: None
    result.model_id = 'offline'
    result.preview_state = None
    result.displayed = []
    result.display = NS(publish=result.displayed.append)
    result.starts = []
    result.deadlines = []
    result.fail_place = False
    result.invalidate = False
    result.active = None
    result.plans = ['old_plan']
    result.next_segment = 0

    def path(initial):
        value = initial.joint_state.position[0]
        message = trajectory()
        message.joint_trajectory.points[0].positions = [value]*6
        message.joint_trajectory.points[-1].positions = [value+.1]*6
        return message

    def pose_plan(initial, goal, deadline, validate=None):
        result.starts.append(list(initial.joint_state.position))
        result.deadlines.append(deadline)
        if result.fail_place and goal == build_place_ready_pose(result.config):
            raise CandidatePlanRejected('injected_place_unreachable')
        if result.invalidate:
            result.discard_prepared_plans()
        return validate(path(initial))

    result._pose_plan = pose_plan
    result._linear_plan = lambda initial, goal, deadline: path(initial)
    result._validate_and_time = lambda name, path, initial, goal, linear, deadline: Segment(
        name, path, initial, goal, linear, [])
    return result


def prepare(a, deadline=None):
    grasp = build_grasp_pose((.3, .1, .2), a.config)
    approach = build_approach_pose(grasp, a.config)
    ready = build_place_ready_pose(a.config)
    return a.prepare_plans(approach, grasp, ready, build_place_drop_pose(ready, a.config), ready,
                           deadline=deadline)


def test_place_rejection_discards_old_plan_without_preview_advance():
    a = adapter()
    a.preview_state = state([.7]*6)
    a.fail_place = True
    with pytest.raises(CandidatePlanRejected) as caught:
        prepare(a)
    assert caught.value.stage == 'place_ready'
    assert not a.plans and not a.displayed and a.next_segment == 0
    assert list(a.preview_state.joint_state.position) == [.7]*6
    a.fail_place = False
    assert prepare(a)
    assert a.starts[2] == [.7]*6  # Candidate B starts at the same virtual state.
    assert len(a.plans) == 6 and len(a.displayed) == 1
    assert a.preview_state.joint_state.position == pytest.approx([1.3]*6)


def test_execution_always_reads_measured_state_between_objects():
    a = adapter()
    a.config = replace(a.config, plan_only=False)
    a.preview_state = state([2.]*6)
    prepare(a)
    assert a.starts[0] == [0.]*6
    a.measured = state([.4]*6)
    prepare(a)
    assert a.starts[2] == [.4]*6


def test_shared_deadline_and_expired_budget_do_not_commit():
    a = adapter()
    deadline = time.monotonic() + .5
    prepare(a, deadline)
    assert all(value <= deadline for value in a.deadlines)
    with pytest.raises(TimeoutError, match='candidate_planning_deadline'):
        prepare(a, time.monotonic() - 1)
    assert not a.plans


def test_invalidation_prevents_late_worker_commit():
    a = adapter()
    a.invalidate = True
    with pytest.raises(RuntimeError, match='planning_generation_invalidated'):
        prepare(a)
    assert not a.plans and not a.displayed and a.preview_state is None


def test_cancellation_prevents_late_worker_commit():
    a = adapter()
    original = a._pose_plan
    def call(*args, **kwargs):
        value = original(*args, **kwargs)
        a.cancel.set()
        return value
    a._pose_plan = call
    with pytest.raises(RuntimeError, match='cancelled'):
        prepare(a)
    assert not a.plans and not a.displayed and a.preview_state is None


def test_discard_does_not_abandon_active_action():
    a = adapter()
    a.active = object()
    with pytest.raises(RuntimeError, match='cannot_discard_active_trajectory'):
        a.discard_prepared_plans()
    assert a.plans == ['old_plan']
    a.result_future = completed(NS(status=4))
    a.discard_prepared_plans()
    assert a.plans == [] and a.active is None


def test_current_start_collision_is_fatal_not_contact_rejection():
    a = adapter()
    def invalid(*args):
        raise RuntimeError('collision_or_invalid_state:robot/table')
    a._valid = invalid
    with pytest.raises(RuntimeError) as caught:
        prepare(a)
    assert not isinstance(caught.value, CandidatePlanRejected)
    assert not a.starts and not a.plans


@pytest.mark.parametrize('code', [-5, -6, -10, -17, 999])
def test_ik_service_faults_cannot_be_misreported_as_no_solution(code):
    from grasp_executor.grasp import build_grasp_pose
    a = arm()
    a._call = lambda *args: NS(error_code=NS(val=code))
    with pytest.raises(RuntimeError) as caught:
        a._ik_solutions(state([0.]*6), build_grasp_pose((.3,.1,.2), a.config), time.monotonic()+1)
    assert not isinstance(caught.value, CandidatePlanRejected)


def test_outer_timeout_invalidates_late_worker_without_trying_contact_b(tmp_path):
    from grasp_executor.candidates import run_candidate_group_task
    from test_candidate_execution import batch
    a = adapter()
    a.config = replace(a.config, planning_timeout=.02, execution_journal=str(tmp_path))
    entered, release, exited = Event(), Event(), Event()
    original = a._pose_plan
    def slow(*args, **kwargs):
        entered.set()
        assert release.wait(1)
        return original(*args, **kwargs)
    a._pose_plan = slow
    attempts = []
    def prepare_call(*args, **kwargs):
        attempts.append(True)
        try:
            return a.prepare_plans(*args, **kwargs)
        finally:
            exited.set()
    interface = NS(check_ready=lambda timeout: True, resolve_place_pose=lambda: None,
                   begin_batch=a.begin_batch, end_batch=a.end_batch,
                   discard_prepared_plans=a.discard_prepared_plans, prepare_plans=prepare_call)
    try:
        result = run_candidate_group_task(batch(), lambda: 2_000_000_000,
                                         interface, a.config, cancel=a.cancel)
        assert entered.is_set() and result.status == 'failed'
        assert result.reason == 'candidate_planning_deadline' and a.cancel.is_set()
        assert not a.plans and len(attempts) == 1 and not list(tmp_path.glob('*.json'))
    finally:
        release.set()
        assert exited.wait(1)
    assert not a.plans and not a.displayed and a.preview_state is None

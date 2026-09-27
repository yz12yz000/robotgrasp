"""Candidate fallback and observation claims without any robot connection."""
from copy import deepcopy
from dataclasses import replace
import math
from pathlib import Path
from threading import Event
import time

import pytest

from grasp_executor.candidates import run_candidate_group_task
from grasp_executor.config import GraspConfig, load_config
from grasp_executor.planning_errors import CandidatePlanRejected
from test_place import RecordingInterfaces


def batch(groups=1, candidates=2):
    return dict(schema_version=1, task_id='task', observation_id='task/round1',
                source_stamp_ns=1_000_000_000, frame_id='base_link', tool_frame='grasp_center',
                objects=[dict(object_id=i, distance=math.hypot(.3 + .2*i, .1),
                    candidates=[dict(candidate_id=j, position=[.3 + .2*i + .01*j, .1, .2],
                        orientation=[0., 1., 0., 0.], geometry=dict(wall_count=30, wall_residual=.001))
                        for j in range(candidates)]) for i in range(groups)], rejected=[])


class Candidates(RecordingInterfaces):
    def __init__(self, failures=(), fail_motion=0, fail_gripper=None):
        super().__init__(fail_motion)
        self.failures = list(failures)
        self.discards = 0
        self.prepared = []
        self.deadlines = []
        self.fail_gripper = fail_gripper
        self.gripper_calls = 0

    def begin_batch(self):
        self.events.append('begin')

    def end_batch(self):
        self.events.append('end')

    def resolve_place_pose(self):
        return None

    def discard_prepared_plans(self):
        self.discards += 1
        self.prepared = []

    def prepare_plans(self, *poses, deadline=None):
        self.plans.append(poses)
        self.deadlines.append(deadline)
        failure = self.failures.pop(0) if self.failures else None
        if failure:
            raise failure
        self.prepared = poses
        return True

    def gripper(self, event):
        self.gripper_calls += 1
        self.events.append(event)
        if self.fail_gripper == self.gripper_calls:
            raise RuntimeError('injected_gripper_failure')
        return True

    def open_gripper(self, timeout):
        return self.gripper('open')

    def close_gripper(self, timeout):
        return self.gripper('close')


def run(tmp_path, interface, value=None, plan_only=False, **kwargs):
    config = GraspConfig(plan_only=plan_only, input_mode='candidate_groups',
                         execution_journal=str(tmp_path), place_dwell_time=.001)
    return run_candidate_group_task(batch() if value is None else value,
        lambda: 2_000_000_000, interface, config, **kwargs)


def rejection(stage='descend'):
    return CandidatePlanRejected('cartesian_path_incomplete:fraction=.032,code=1', stage=stage)


def test_first_rejected_second_executes_only_once(tmp_path):
    interface = Candidates([rejection()])
    cancel = Event()
    result = run(tmp_path, interface, cancel=cancel)
    assert result.status == 'success' and result.completed_count == 1
    assert result.candidate_id == 1 and result.candidate_attempt_count == 2
    assert result.candidate_failures[0]['stage'] == 'descend'
    assert len(interface.moves) == 6 and len(interface.plans) == 2
    assert interface.moves[1].position == (.31, .1, .2)
    assert interface.events.count('close') == 1
    assert len(list(tmp_path.glob('object_*.json'))) == 1
    assert not cancel.is_set()
    assert interface.deadlines[0] == interface.deadlines[1]


def test_all_rejected_have_no_commands_claim_or_global_cancel(tmp_path):
    interface = Candidates([rejection(), rejection('place_ready')])
    cancel = Event()
    result = run(tmp_path, interface, cancel=cancel)
    assert result.status == 'candidates_exhausted' and not result.command_attempted
    assert result.completed_count == 0 and len(result.candidate_failures) == 2
    assert not interface.moves and not interface.gripper_calls and not interface.prepared
    assert not list(tmp_path.glob('*.json')) and not cancel.is_set()


@pytest.mark.parametrize('failure', [RuntimeError('cancelled'), RuntimeError('unknown'),
    RuntimeError('stale_robot_tf_feedback'), RuntimeError('planning_start_invalid:collision'),
    RuntimeError('robot_moved_during_planning'), TimeoutError('moveit_response_timeout')])
def test_environment_and_timeout_are_not_retried(tmp_path, failure):
    interface, cancel = Candidates([failure]), Event()
    result = run(tmp_path, interface, cancel=cancel)
    assert result.status == 'failed' and result.reason == str(failure)
    assert len(interface.plans) == 1 and not interface.moves and not interface.prepared
    assert not list(tmp_path.glob('*.json')) and cancel.is_set()


@pytest.mark.parametrize('motion', range(1, 7))
def test_every_motion_failure_stops_without_switching_contact(tmp_path, motion):
    interface = Candidates(fail_motion=motion)
    result = run(tmp_path, interface)
    assert result.status == 'failed' and result.command_attempted
    assert len(interface.plans) == 1 and len(interface.moves) == motion
    assert 'stop' in interface.events and interface.events.count('close') <= 1
    assert len(list(tmp_path.glob('*.json'))) == 1


@pytest.mark.parametrize('gripper', [1, 2, 3])
def test_gripper_failure_keeps_claim_and_never_retries(tmp_path, gripper):
    interface = Candidates(fail_gripper=gripper)
    result = run(tmp_path, interface)
    assert result.status == 'failed' and result.completed_count == 0
    assert len(interface.plans) == 1 and interface.gripper_calls == gripper
    assert 'stop' in interface.events and len(list(tmp_path.glob('*.json'))) == 1


def test_contact_reorder_cannot_bypass_object_claim(tmp_path):
    assert run(tmp_path, Candidates()).status == 'success'
    changed = batch()
    contacts = changed['objects'][0]['candidates']
    contacts.reverse()
    for index, candidate in enumerate(contacts):
        candidate['candidate_id'] = index
        candidate['geometry']['wall_count'] = 900
    changed['objects'][0]['distance'] = math.hypot(*contacts[0]['position'][:2])
    interface = Candidates()
    result = run(tmp_path, interface, changed)
    assert result.reason == 'object_already_claimed' and not interface.moves
    assert len(list(tmp_path.glob('*.json'))) == 1


def test_only_selected_object_is_claimed(tmp_path):
    one = batch(groups=2)
    one['objects'] = [one['objects'][0]]
    assert run(tmp_path, Candidates(), one).status == 'success'
    other = batch(groups=2)
    other['objects'] = [other['objects'][1]]
    assert run(tmp_path, Candidates(), other).status == 'success'
    assert len(list(tmp_path.glob('*.json'))) == 2


def test_execute_rejects_multiple_groups_before_readiness(tmp_path):
    interface = Candidates()
    result = run(tmp_path, interface, batch(groups=2))
    assert result.reason == 'execution_requires_single_object_group'
    assert not interface.events and not interface.plans and not interface.moves


def test_preview_nearest_first_without_commands_or_claims(tmp_path):
    value = batch(groups=2)
    value['objects'].reverse()
    saved = deepcopy(value)
    interface = Candidates([rejection(), None, None])
    result = run(tmp_path, interface, value, plan_only=True)
    assert result.status == 'planned' and result.completed_count == 0
    assert result.candidate_attempt_count == 3 and result.object_count == 2
    assert [poses[1].position[0] for poses in interface.plans] == [.3, .31, .5]
    assert not interface.moves and not interface.gripper_calls and not list(tmp_path.glob('*.json'))
    assert value == saved


def test_time_and_identity_validation_never_plan(tmp_path):
    for options in (dict(expected_observation_id='other'), dict(source_not_before_ns=2_000_000_000),
                    dict(task_deadline_monotonic_ns=1)):
        interface = Candidates()
        result = run(tmp_path, interface, **options)
        assert result.status == 'failed' and not interface.plans and not interface.moves


def test_expiry_after_planning_discards_without_claim(tmp_path):
    clock = [2_000_000_000]
    interface = Candidates()
    original = interface.prepare_plans
    def prepare(*args, **kwargs):
        answer = original(*args, **kwargs)
        clock[0] = 900_000_000_000
        return answer
    interface.prepare_plans = prepare
    result = run_candidate_group_task(batch(), lambda: clock[0], interface,
        GraspConfig(plan_only=False, execution_journal=str(tmp_path)))
    assert result.reason == 'stale_target' and not interface.moves and not interface.prepared
    assert not list(tmp_path.glob('*.json'))


def test_fixed_height_and_offsets_once(tmp_path):
    value, interface = batch(candidates=1), Candidates()
    config = GraspConfig(plan_only=False, execution_journal=str(tmp_path), place_dwell_time=.001,
                         grasp_offset=dict(x=.01, y=0., z=.02))
    result = run_candidate_group_task(value, lambda: 2_000_000_000, interface, config)
    assert result.status == 'success'
    approach, grasp, lift, ready, drop, retreat = interface.moves
    assert grasp.position == pytest.approx((.31, .1, .22))
    assert approach.position == pytest.approx((.31, .1, .32))
    assert approach == lift and ready == retreat
    assert ready.position[2] - drop.position[2] == pytest.approx(.20)
    assert grasp.orientation == (0., 1., 0., 0.)
    for name in ('config.json', 'place_config.json'):
        assert load_config(Path(__file__).parents[1] / 'config' / name).approach_height == .10


def test_cancel_after_preview_before_claim_has_no_journal(tmp_path):
    interface, cancel = Candidates(), Event()
    result = run(tmp_path, interface, cancel=cancel, on_poses=lambda *args: cancel.set())
    assert result.status == 'failed' and result.reason == 'cancelled'
    assert not interface.moves and not list(tmp_path.glob('*.json'))


def test_grasp_node_rejects_wrong_round_and_multiple_execution_groups(monkeypatch):
    import json
    from types import SimpleNamespace as NS
    from grasp_executor import main
    for value in (batch(groups=2), dict(batch(), observation_id='old_round')):
        reports, started = [], []
        node = NS(state='waiting_target', task_id='task', observation_id='task/round1',
                  source_not_before_ns=0, config=GraspConfig(plan_only=False),
                  report=lambda status, **details: reports.append(dict(status=status, **details)),
                  run_candidate_job=lambda value: None, worker=None)
        monkeypatch.setattr(main, 'Thread', lambda **kwargs: NS(start=lambda: started.append(True)))
        main.GraspNode.on_candidate_groups(node, NS(data=json.dumps(value)))
        assert reports[-1]['status'] == 'failed' and not started
        assert not reports[-1]['command_attempted']


@pytest.mark.parametrize('height', [1.45, 1.6])
def test_workspace_rejects_only_this_contact_then_tries_next(tmp_path, height):
    value, interface = batch(), Candidates()
    value['objects'][0]['candidates'][0]['position'][2] = height
    result = run(tmp_path, interface, value)
    assert result.status == 'success' and result.candidate_id == 1
    assert result.candidate_attempt_count == 2
    assert result.candidate_failures == [dict(object_id=0, candidate_id=0,
        stage='workspace', reason='pose_outside_workspace')]
    assert len(interface.plans) == 1 and len(interface.moves) == 6
    assert interface.moves[1].position == (.31, .1, .2)
    assert len(list(tmp_path.glob('*.json'))) == 1


def test_workspace_rejection_does_not_reorder_object_by_fallback_distance(tmp_path):
    value, interface = batch(groups=2), Candidates()
    # Nearest object's preferred contact is outside Z bounds; its fallback is
    # farther in XY than the second object, but object priority must not change.
    value['objects'][0]['candidates'][0]['position'][2] = 1.45
    value['objects'][0]['candidates'][1]['position'][0] = .7
    result = run(tmp_path, interface, value, plan_only=True)
    assert result.status == 'planned'
    assert [poses[1].position[0] for poses in interface.plans] == [.7, .5]
    assert result.candidate_attempt_count == 3


def test_offset_workspace_failure_can_use_later_contact(tmp_path):
    value, interface = batch(), Candidates()
    value['objects'][0]['candidates'][0]['position'][2] = 1.3
    config = GraspConfig(plan_only=True, grasp_offset=dict(x=.01, y=0., z=.2))
    result = run_candidate_group_task(value, lambda: 2_000_000_000, interface, config)
    assert result.status == 'planned' and result.candidate_id == 1
    assert len(interface.plans) == 1
    assert interface.plans[0][1].position == pytest.approx((.32, .1, .4))


def test_workspace_rejections_cannot_mask_stale_observation(tmp_path):
    value, interface = batch(), Candidates()
    value['objects'][0]['candidates'][0]['position'][2] = 1.45
    result = run_candidate_group_task(value, lambda: 900_000_000_000, interface, GraspConfig())
    assert result.status == 'failed' and result.reason == 'stale_target'
    assert not interface.plans and not result.candidate_failures

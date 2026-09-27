"""Bounded recovery policy tests without ROS or a device."""
import json
import pytest
from place_rounds import RoundBudget, RoundLimits, recoverable_failure, select_candidate_groups
import test_live_place_pipeline as runner


def test_no_progress_limit_not_keyed_by_object_ids():
    budget = RoundBudget(100., RoundLimits())
    for i in range(3):
        budget.begin_round(i)
        assert budget.retry_observation(i) is (i < 2)
    assert budget.observation_count == 3 and budget.reobservations_without_progress == 2
    assert budget.empty_result() == (False, 'failed', 'remaining_unknown')


def test_success_resets_retry_count_but_not_global_budgets():
    budget = RoundBudget(10., RoundLimits(max_observation_rounds=3))
    budget.begin_round(1)
    budget.retry_observation(2)
    budget.complete_object()
    assert budget.reobservations_without_progress == 0 and budget.observation_count == 1
    with pytest.raises(TimeoutError): budget.begin_round(10)


@pytest.mark.parametrize('value', [True, -1, 1.5, '2'])
def test_invalid_limits(value):
    with pytest.raises(ValueError): RoundLimits(max_reobservations=value)


def test_unknown_failure_cannot_be_recovered_alongside_geometry_failure():
    statuses = {'grasp_executor': {'status': 'candidates_exhausted', 'command_attempted': False},
                'yolo_vision': {'status': 'failed', 'reason': 'input_timeout'}}
    assert not recoverable_failure(statuses)
    assert not recoverable_failure({'grasp_executor': {'status': 'candidates_exhausted'}})
    assert not recoverable_failure({'grasp_executor': {'status': 'failed', 'reason': 'ik_failure'}})


def test_selection_preserves_candidate_order_and_applies_offset_to_object_order_only():
    def group(oid, x):
        return {'object_id': oid, 'candidates': [
            {'candidate_id': 0, 'position': [x, 0., .2], 'geometry': {'score': 0}},
            {'candidate_id': 1, 'position': [0., 0., .2], 'geometry': {'score': 100}}]}
    batch = {'objects': [group(1, .2), group(2, .5)], 'rejected': []}
    original = json.dumps(batch)
    selected = select_candidate_groups(batch, {'x': -.5, 'y': 0., 'z': 0.}, True)
    assert selected['objects'][0]['object_id'] == 2
    assert [c['candidate_id'] for c in selected['objects'][0]['candidates']] == [0, 1]
    assert selected['objects'][0]['candidates'][0]['position'][0] == .5
    assert json.dumps(batch) == original


def test_task_mutex_is_independent_of_namespace(monkeypatch, tmp_path):
    monkeypatch.setattr(runner, 'task_lock_path', lambda: tmp_path/'lock')
    with runner.TaskLock():
        with pytest.raises(RuntimeError, match='another_place_task'):
            with runner.TaskLock(): pass
    with runner.TaskLock(): pass


def test_cleanup_stops_descendant_after_launch_leader_exits():
    import os
    import signal
    import subprocess
    import sys
    child_script = 'import time; time.sleep(60)'
    parent_script = (
        'import subprocess, sys; '
        'child = subprocess.Popen([sys.executable, "-c", sys.argv[1]], '
        'stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL); '
        'print(child.pid, flush=True)')
    leader = subprocess.Popen([sys.executable, '-c', parent_script, child_script],
        start_new_session=True, stdout=subprocess.PIPE, text=True)
    try:
        child_pid = int(leader.communicate(timeout=5)[0].strip())
        assert child_pid != leader.pid and leader.returncode == 0
        assert runner.process_group_alive(leader.pid)
        assert runner.stop_owned_processes({'already_exited_launch': leader}) == []
        assert not runner.process_group_alive(leader.pid)
    finally:
        try:
            os.killpg(leader.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        leader.wait(timeout=5)


def test_isolated_ros_fixtures_cannot_probe_concurrently(tmp_path):
    from isolated_ros import IsolatedRosTestLock
    path = tmp_path / 'ros-fixture.lock'
    with IsolatedRosTestLock(path):
        with pytest.raises(RuntimeError, match='another_isolated_ros_test'):
            IsolatedRosTestLock(path)
    with IsolatedRosTestLock(path):
        pass

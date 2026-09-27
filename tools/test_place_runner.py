"""Exercise the actual runner with fake ROS, processes and a virtual clock."""
import json
import signal
import sys
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import test_live_place_pipeline as runner


def run_fake(monkeypatch, tmp_path, scenes, execute=False, extra=(), new_run=False,
             overlay=None, busy=(), stale_callbacks=False, cleanup_failure=False):
    clock, subscriptions, processes, commands, selected = [1.], {}, [], [], []
    root = Path(runner.__file__).resolve().parents[1]
    override = tmp_path / 'vision.json'
    override.write_text(json.dumps(overlay or {'multi_instance': False}))
    original_paths = [root / f'src/{name}/config/place_config.json'
                      for name in ('yolo_vision', 'rim_locator', 'grasp_executor')] + [override]
    originals = {p: p.read_bytes() for p in original_paths}
    monkeypatch.chdir(root / 'New_run' if new_run else root)
    states, old_callbacks, publishers = {}, [], []

    def ros_now(): return int((1000. + clock[0]) * 1e9)

    def send_status(state, name, value, **details):
        payload = {'status': value, 'task_id': state['task_id'],
                   'observation_id': state['observation_id'], **details}
        subscriptions[state['namespace'] + '/' + name + '/status'](NS(data=json.dumps(payload)))

    def batch_for(state):
        scene = state['scene']
        positions = scene.get('positions', [[.3 + .1*i, .1, .2] for i in range(scene.get('count', 1))])
        ids = scene.get('ids', list(range(len(positions))))
        return {'schema_version': 1, 'task_id': state['task_id'], 'observation_id': state['observation_id'],
                'source_stamp_ns': ros_now(), 'frame_id': 'base_link', 'tool_frame': 'grasp_center',
                'objects': [{'object_id': oid, 'distance': runner.math.hypot(*pos[:2]),
                    'candidates': [{'candidate_id': j, 'position': [pos[0], pos[1]+j*.005, pos[2]],
                        'orientation': [0., 1., 0., 0.], 'geometry': {}} for j in range(2)]}
                    for oid, pos in zip(ids, positions)], 'rejected': []}

    class Publisher:
        def __init__(self, topic, qos):
            self.topic, self.qos, self.alive = topic, qos, True
            publishers.append(self)
        def publish(self, message):
            assert self.alive
            payload = json.loads(message.data)
            state = states[payload['observation_id']]
            assert state.get('executor_ready')
            assert self.qos.depth == 1
            assert self.qos.durability == runner.DurabilityPolicy.TRANSIENT_LOCAL
            selected.append(deepcopy(payload))
            scene = state['scene']
            if scene.get('executor_error'):
                send_status(state, 'grasp_executor', 'failed', reason=scene['executor_error'],
                    command_attempted=scene.get('command_attempted', False), object_count=len(payload['objects']),
                    completed_count=0, candidate_attempt_count=1, source_stamp_ns=payload['source_stamp_ns'])
                return
            if scene.get('exhausted'):
                send_status(state, 'grasp_executor', 'candidates_exhausted', reason='all_candidates_failed',
                    command_attempted=False, completed_count=0, object_count=len(payload['objects']),
                    candidate_attempt_count=2, source_stamp_ns=payload['source_stamp_ns'])
                return
            preview_count = scene.get('previews', len(payload['objects']))
            preview_topic = state['namespace'] + '/task_preview'
            if scene.get('intermediate'):
                subscriptions[preview_topic](NS(trajectory=[NS(joint_trajectory=NS(points=[1, 2]))]))
            for _ in range(preview_count):
                subscriptions[preview_topic](NS(trajectory=[NS(joint_trajectory=NS(points=[1, 2]))] * 6))
            send_status(state, 'grasp_executor', 'success' if execute else 'planned',
                object_count=len(payload['objects']), completed_count=1 if execute else 0,
                candidate_attempt_count=len(payload['objects'])*2,
                candidate_failures=[{'candidate_id': 0, 'reason': 'test_rejected'}],
                command_attempted=execute, source_stamp_ns=payload['source_stamp_ns'])

    class Node:
        def __init__(self, *args): pass
        def get_node_names_and_namespaces(self): return list(busy)
        def get_clock(self): return NS(now=lambda: NS(nanoseconds=ros_now()))
        def create_subscription(self, kind, topic, callback, qos):
            subscriptions[topic] = callback
            return topic
        def create_publisher(self, kind, topic, qos): return Publisher(topic, qos)
        def destroy_subscription(self, token):
            old_callbacks.append(subscriptions.pop(token))
        def destroy_publisher(self, publisher): publisher.alive = False
        def destroy_node(self): pass

    class Process:
        def __init__(self, command, **kwargs):
            self.pid, self.returncode = 1000 + len(processes), None
            processes.append(self)
            commands.append(command)
            self.command = command
            if command[0].endswith('run_grasp_position_control.sh'):
                assert execute
                self.returncode = 0
                return
            params = dict(arg.split(':=', 1) for arg in command if ':=' in arg)
            observation_id = params['observation_id']
            if observation_id not in states:
                index = int(observation_id.rsplit('_', 1)[-1]) - 1
                states[observation_id] = {**params, 'scene': scenes[min(index, len(scenes)-1)],
                                         'observed': False, 'started': clock[0]}
            state = states[observation_id]
            if command[0].endswith('run_rim_locator.sh'):
                send_status(state, 'rim_locator', 'waiting_input')
            elif command[0].endswith('run_yolo_vision.sh'):
                state['vision_started'] = True
                send_status(state, 'yolo_vision', 'ready')
            elif command[0].endswith('run_grasp_executor.sh'):
                assert state['observed']
                assert int(params['supervisor_pid']) == runner.os.getpid()
                assert int(params['supervisor_start_ticks']) == runner.process_start_ticks(runner.os.getpid())
                assert params['task_preview_topic'] == state['namespace'] + '/task_preview'
                assert int(params['source_not_before_ns']) < ros_now()
                assert int(params['task_deadline_monotonic_ns']) > int(clock[0] * 1e9)
                state['executor_ready'] = True
                send_status(state, 'grasp_executor', 'waiting_target')
        def poll(self): return self.returncode
        def wait(self, timeout):
            if self.returncode is None:
                raise runner.subprocess.TimeoutExpired(self.command, timeout)
            return self.returncode

    def spin(node, timeout_sec):
        clock[0] += max(.05, timeout_sec)
        for state in list(states.values()):
            if not state.get('vision_started') or state['observed']:
                continue
            scene = state['scene']
            if clock[0] - state['started'] < scene.get('observe_after', .1):
                continue
            state['observed'] = True
            if stale_callbacks:
                for callback in old_callbacks:
                    callback(NS(data='not valid JSON'))  # Retired closures must do nothing.
            if scene.get('vision_error'):
                send_status(state, 'yolo_vision', 'failed', reason=scene['vision_error'],
                            source_stamp_ns=ros_now(), detected_count=1,
                            rejected=[{'object_id': 0, 'reason': 'insufficient_pointcloud'}])
                continue
            if scene.get('count') == 0:
                send_status(state, 'yolo_vision', 'no_targets', detected_count=0, object_count=0,
                            source_stamp_ns=ros_now())
                continue
            batch = batch_for(state)
            state['batch'] = batch
            count = len(batch['objects'])
            extra_rejected = [{'object_id': 99, 'reason': 'insufficient_pointcloud'}] if scene.get('rejected') else []
            stamp = batch['source_stamp_ns']
            send_status(state, 'yolo_vision', 'success', detected_count=count+len(extra_rejected),
                        object_count=count, objects=[{'object_id': g['object_id']} for g in batch['objects']],
                        rejected=extra_rejected, source_stamp_ns=stamp)
            if scene.get('rim_error'):
                rejected = [{'object_id': g['object_id'], 'reason': scene['rim_error']} for g in batch['objects']]
                batch['objects'], batch['rejected'] = [], rejected
                subscriptions[state['namespace']+'/grasp_candidates'](NS(data=json.dumps(batch)))
                send_status(state, 'rim_locator', 'failed', reason='pose_not_found:no_valid_object',
                            rejected=rejected, source_stamp_ns=stamp)
                continue
            if scene.get('old_candidate'):
                batch['source_stamp_ns'] = int(state.get('source_not_before_ns', ros_now())) - 1
            if scene.get('foreign_candidate'):
                foreign = deepcopy(batch)
                foreign['observation_id'] = 'foreign/round_0001'
                subscriptions[state['namespace']+'/grasp_candidates'](NS(data=json.dumps(foreign)))
            subscriptions[state['namespace']+'/grasp_candidates'](NS(data=json.dumps(batch)))
            send_status(state, 'rim_locator', 'success', object_count=count, source_stamp_ns=stamp, rejected=[])

    def kill(pid, sig):
        clock[0] += .001  # Process shutdown takes time before the next source barrier.
        process = next(p for p in processes if p.pid == pid)
        if cleanup_failure and sig == signal.SIGINT:
            return
        process.returncode = 0

    monkeypatch.setattr(runner, 'Node', Node)
    monkeypatch.setattr(runner, 'task_lock_path', lambda: tmp_path / 'task.lock')
    monkeypatch.setattr(runner.rclpy, 'init', lambda **kwargs: None)
    monkeypatch.setattr(runner.rclpy, 'shutdown', lambda: None)
    monkeypatch.setattr(runner.rclpy, 'spin_once', spin)
    monkeypatch.setattr(runner.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(runner.subprocess, 'Popen', Process)
    monkeypatch.setattr(runner.os, 'killpg', kill)
    monkeypatch.setattr(runner, 'process_group_alive',
                        lambda pid: next(p for p in processes if p.pid == pid).returncode is None)
    monkeypatch.setattr(runner.time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0] + seconds))
    output = tmp_path / 'output'
    monkeypatch.setattr(sys, 'argv', ['test', '--output-dir', str(output), '--vision-config',
        runner.os.path.relpath(override, Path.cwd()), '--timeout', '120'] + (['--execute'] if execute else []) + list(extra))
    code = runner.main()
    result = json.loads((output/'result.json').read_text())
    assert all(p.read_bytes() == data for p, data in originals.items())
    assert all(p.poll() is not None for p in processes)
    assert all(not p.alive for p in publishers)
    return code, result, commands, selected


@pytest.mark.parametrize('count', [0, 1, 2, 3])
@pytest.mark.parametrize('execute', [False, True])
@pytest.mark.parametrize('new_run', [False, True])
def test_observed_count_drives_fresh_rounds(monkeypatch, tmp_path, count, execute, new_run):
    scenes = ([{'count': n} for n in range(count, -1, -1)] if execute else [{'count': count}])
    code, result, commands, selected = run_fake(monkeypatch, tmp_path, scenes, execute, new_run=new_run)
    assert code == 0 and result['passed']
    assert result['completed_count'] == (count if execute else 0)
    assert result['initial_detected_count'] == count
    assert len(result['preview_batches']) == count
    assert result['observation_count'] == (count+1 if execute else 1)
    assert len(selected) == (count if execute else bool(count))
    if execute:
        assert all(len(batch['objects']) == 1 for batch in selected)
    else:
        assert not any('position_control' in command[0] for command in commands)
    for index, current in enumerate(result['rounds'], 1):
        assert f'round_{index:04d}' in current['namespace']
        assert current['source_not_before_ns'] > 0
        configs = current['config_snapshots']
        g = json.loads(Path(configs['grasp']).read_text())
        assert g['input_mode'] == 'candidate_groups' and g['approach_height'] == .10
        assert g['plan_only'] is (not execute)
        assert g['input_topic'].startswith(current['namespace'])
        assert g['task_preview_topic'].startswith(current['namespace'])


def test_next_object_uses_new_position_and_reassigned_id(monkeypatch, tmp_path):
    scenes = [{'positions': [[.3,.1,.2], [.5,.1,.2]], 'ids': [4, 9]},
              {'positions': [[.62,.05,.21]], 'ids': [0]}, {'count': 0}]
    code, result, _, selected = run_fake(monkeypatch, tmp_path, scenes, True, stale_callbacks=True)
    assert code == 0 and result['completed_count'] == 2
    assert selected[0]['objects'][0]['object_id'] == 4
    assert selected[1]['objects'][0]['object_id'] == 0
    assert selected[1]['objects'][0]['candidates'][0]['position'] == [.62,.05,.21]
    assert selected[0]['source_stamp_ns'] < selected[1]['source_stamp_ns']
    assert selected[0]['observation_id'] != selected[1]['observation_id']


def test_exhausted_candidates_have_bounded_reobservation(monkeypatch, tmp_path):
    code, result, _, selected = run_fake(monkeypatch, tmp_path, [{'exhausted': True}], True)
    assert code == 1 and len(selected) == result['observation_count'] == 3
    assert result['completed_count'] == 0 and result['error'] == 'reobservation_budget_exhausted'
    assert result['candidate_attempt_count'] == 6


@pytest.mark.parametrize('scene', [
    {'vision_error': 'no_valid_depth_for_detected_targets'},
    {'rim_error': 'pose_not_found:insufficient_wall_points'}])
def test_geometry_failure_can_reobserve_and_then_complete(monkeypatch, tmp_path, scene):
    code, result, _, _ = run_fake(monkeypatch, tmp_path, [scene, {'count': 1}, {'count': 0}], True)
    assert code == 1 and result['completed_count'] == 1 and result['observation_count'] == 3
    assert result['reobservations_without_progress'] == 0
    assert result['outcome'] == 'partial' and result['error'] == 'remaining_unknown'


@pytest.mark.parametrize('scene', [
    {'vision_error': 'input_timeout'}, {'rim_error': 'invalid_transform'},
    {'executor_error': 'controller_not_ready'},
    {'executor_error': 'motion_timeout', 'command_attempted': True}])
def test_nonrecoverable_errors_never_start_second_round(monkeypatch, tmp_path, scene):
    code, result, _, _ = run_fake(monkeypatch, tmp_path, [scene, {'count': 0}], True)
    assert code == 1 and result['observation_count'] == 1 and result['completed_count'] == 0


def test_failed_target_disappearing_is_remaining_unknown(monkeypatch, tmp_path):
    code, result, _, _ = run_fake(monkeypatch, tmp_path, [{'exhausted': True}, {'count': 0}], True)
    assert code == 1 and result['error'] == 'remaining_unknown'


def test_prior_unresolved_geometry_is_not_erased_by_another_success(monkeypatch, tmp_path):
    scenes = [{'count': 1, 'rejected': True}, {'count': 1}, {'count': 0}]
    code, result, _, _ = run_fake(monkeypatch, tmp_path, scenes, True)
    assert code == 1 and result['outcome'] == 'partial' and result['completed_count'] == 2
    assert result['error'] == 'remaining_unknown'


def test_preview_does_not_loop_and_reports_rejected_targets(monkeypatch, tmp_path):
    code, result, _, _ = run_fake(monkeypatch, tmp_path, [{'count': 2, 'rejected': True}])
    assert code == 1 and result['outcome'] == 'partial' and result['observation_count'] == 1
    assert result['completed_count'] == 0


def test_missing_complete_preview_is_failure(monkeypatch, tmp_path):
    code, result, _, _ = run_fake(monkeypatch, tmp_path, [{'count': 2, 'previews': 1, 'intermediate': True}])
    assert code == 1 and result['error'] == 'preview_batch_mismatch'


def test_foreign_candidate_and_retired_callbacks_cannot_advance_round(monkeypatch, tmp_path):
    code, result, _, _ = run_fake(monkeypatch, tmp_path,
        [{'count': 1, 'foreign_candidate': True}, {'count': 0}], True, stale_callbacks=True)
    assert code == 0 and result['completed_count'] == 1


def test_old_candidate_is_rejected_before_executor(monkeypatch, tmp_path):
    code, result, commands, _ = run_fake(monkeypatch, tmp_path, [{'old_candidate': True}], True)
    assert code == 1 and not any('run_grasp_executor' in c[0] for c in commands)


@pytest.mark.parametrize('limit,expected', [('--max-completed-objects','max_completed_objects_reached'),
                                          ('--max-observation-rounds','max_observation_rounds_reached')])
def test_total_limits_do_not_reset_on_success(monkeypatch, tmp_path, limit, expected):
    code, result, _, _ = run_fake(monkeypatch, tmp_path, [{'count': 1}], True, extra=[limit, '1'])
    assert code == 1 and result['completed_count'] == 1 and result['error'] == expected


def test_zero_reobservations_and_global_deadline(monkeypatch, tmp_path):
    code, result, _, _ = run_fake(monkeypatch, tmp_path, [{'exhausted': True}], True,
                                extra=['--max-reobservations','0'])
    assert code == 1 and result['observation_count'] == 1


def test_no_frames_times_out_without_controller(monkeypatch, tmp_path):
    code, result, commands, _ = run_fake(monkeypatch, tmp_path, [{'observe_after': 1000}], True,
                                       extra=['--timeout', '3'])
    assert code == 1 and result['error'] == 'pipeline_timeout'
    assert not any('position_control' in c[0] for c in commands)


def test_cleanup_failure_prevents_next_observation(monkeypatch, tmp_path):
    code, result, _, _ = run_fake(monkeypatch, tmp_path, [{'count': 1}], True, cleanup_failure=True)
    assert code == 1 and result['observation_count'] == 1 and result['completed_count'] == 1
    assert 'forced_shutdown' in result['error']


@pytest.mark.parametrize('fault', ['single','multi','string_bool','unknown','bad_json','missing','timeout','limit'])
def test_invalid_config_fails_before_ros_or_processes(monkeypatch, tmp_path, fault):
    def forbidden(*args, **kwargs): pytest.fail('invalid input must fail before ROS or processes')
    monkeypatch.setattr(runner.rclpy, 'init', forbidden)
    monkeypatch.setattr(runner.subprocess, 'Popen', forbidden)
    config = tmp_path/'override.json'
    if fault == 'string_bool': config.write_text('{"multi_instance":"false"}')
    if fault == 'unknown': config.write_text('{"unknown":true}')
    if fault == 'bad_json': config.write_text('{')
    args = ['test', '--output-dir', str(tmp_path/'out'), '--execute']
    args += (['--'+fault] if fault in ('single','multi') else ['--timeout','nan'] if fault == 'timeout'
             else ['--max-reobservations','-1'] if fault == 'limit' else ['--vision-config',str(config)])
    monkeypatch.setattr(sys, 'argv', args)
    with pytest.raises(SystemExit) as error: runner.main()
    assert error.value.code == 2 and not (tmp_path/'out').exists()


@pytest.mark.parametrize('symlink', [False, True])
def test_snapshot_cannot_overwrite_source_config(monkeypatch, tmp_path, symlink):
    output = tmp_path/'out'; (output/'round_0001').mkdir(parents=True)
    snapshot = output/'round_0001/effective_vision_config.json'
    config = tmp_path/'source.json' if symlink else snapshot
    original = '{"multi_instance":false}'; config.write_text(original)
    if symlink: snapshot.symlink_to(config)
    monkeypatch.setattr(sys, 'argv', ['test','--output-dir',str(output),'--vision-config',str(config)])
    with pytest.raises(SystemExit) as error: runner.main()
    assert error.value.code == 2 and config.read_text() == original
    assert not (output/'round_0001/effective_rim_config.json').exists()


def test_unassociated_success_does_not_erase_prior_exhausted_object(monkeypatch, tmp_path):
    scenes = [{'exhausted': True, 'ids': [4]}, {'count': 1, 'ids': [0]}, {'count': 0}]
    code, result, _, _ = run_fake(monkeypatch, tmp_path, scenes, True)
    assert code == 1 and result['completed_count'] == 1
    assert result['outcome'] == 'partial' and result['error'] == 'remaining_unknown'
    assert result['reobservations_without_progress'] == 0

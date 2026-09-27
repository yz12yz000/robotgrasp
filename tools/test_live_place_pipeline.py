#!/usr/bin/env python3
"""Plan one observation, or execute one object then observe again.

The default is plan-only: no controller switching, arm motion or gripper command.
Execution uses the existing six-stage task. Only pre-motion candidate exhaustion
and explicit geometry failures may request a bounded fresh observation.
"""
import argparse
from dataclasses import asdict, replace
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from threading import Event
import uuid

import rclpy
from moveit_msgs.msg import DisplayTrajectory
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from yolo_vision.config import load_effective_config
from rim_locator.config import load_config as load_rim_config
from rim_locator.candidate_protocol import decode_candidate_batch, encode_candidate_batch
from grasp_executor.config import load_config as load_grasp_config
from grasp_executor.process_guard import process_start_ticks
from place_rounds import (RoundBudget, RoundLimits, context_matches,
                          recoverable_failure, select_candidate_groups)


def write_json(path, value, source_paths=()):
    path = Path(path)
    if path.resolve() in source_paths:
        raise ValueError('output_overwrites_source_config:' + str(path))
    # A task/round result is immutable once written. Never replace older evidence.
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)


def task_lock_path():
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    return Path('/tmp') / f'robot-grasp-{os.getuid()}-domain-{domain}.place-task.lock'


class TaskLock:
    def __enter__(self):
        fd = os.open(task_lock_path(), os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            os.close(fd)
            raise RuntimeError('another_place_task_is_running')
        self.fd = fd
        return self

    def __exit__(self, *args):
        os.close(self.fd)


def process_group_alive(pgid):
    """Linux process groups include ros2 launch descendants, even after its exit.

    Zombies cannot send commands and may wait for an external parent to reap them.
    Read /proc rather than considering such zombies a live worker forever.
    """
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields = path.read_text().rsplit(')', 1)[1].split()
            if int(fields[2]) == pgid and fields[0] not in ('Z', 'X'):
                return True
        except (FileNotFoundError, ProcessLookupError):
            continue
    return False


def stop_owned_processes(processes):
    errors = []
    pending = dict(processes)
    for sig, allowance in ((signal.SIGINT, 20.), (signal.SIGTERM, 3.), (signal.SIGKILL, 3.)):
        if sig == signal.SIGTERM:
            errors.extend('forced_shutdown:' + name for name in pending)
        for name, process in pending.items():
            # The session leader can exit before its descendants; always signal
            # the group that this round created, never unrelated ROS processes.
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                pass
            except OSError as exc:
                errors.append('process_signal_failed:' + name + ':' + str(exc))
        deadline = time.monotonic() + allowance
        while pending:
            for name, process in list(pending.items()):
                process.poll()  # reap the child leader if it has exited
                try:
                    alive = process_group_alive(process.pid)
                except OSError as exc:
                    errors.append('process_group_check_failed:' + name + ':' + str(exc))
                    alive = True
                if not alive:
                    pending.pop(name)
            if not pending or time.monotonic() >= deadline:
                break
            time.sleep(.05)
        if not pending:
            break
    errors.extend('process_shutdown_unconfirmed:' + name for name in pending)
    return errors


def round_configs(vision, rim, grasp, namespace, timeout, execute):
    return {
        'vision': replace(vision, output_topic=namespace + '/object_cloud'),
        'rim': replace(rim, input_topic=namespace + '/object_cloud',
                       output_topic=namespace + '/rim_point', poses_topic=namespace + '/grasp_poses',
                       candidates_topic=namespace + '/grasp_candidates',
                       input_timeout=max(rim.input_timeout, timeout + 35.)),
        'grasp': replace(grasp, input_mode='candidate_groups', input_topic=namespace + '/selected_candidates',
                         task_preview_topic=namespace + '/task_preview', plan_only=not execute,
                         input_timeout=max(grasp.input_timeout, timeout + 35.)),
    }


class ObservationRound:
    """Own one namespace, one set of processes, and one immutable observation."""
    def __init__(self, node, root, output, task_id, number, configs, deadline,
                 source_not_before_ns, execute, interrupted, source_paths):
        self.node, self.root, self.output = node, root, output
        self.task_id, self.observation_id = task_id, f'{task_id}/round_{number:04d}'
        self.namespace = f'/place_demo_{task_id}/round_{number:04d}'
        self.deadline, self.barrier = deadline, source_not_before_ns
        self.execute, self.interrupted = execute, interrupted
        self.configs, self.source_paths = configs, source_paths
        self.statuses, self.history, self.previews, self.intermediate = {}, [], [], []
        self.processes, self.logs, self.subscriptions = {}, [], []
        self.batch, self.selected, self.callback_error = None, None, None
        self.supervisor_pid = os.getpid()
        self.supervisor_start_ticks = process_start_ticks(self.supervisor_pid)
        self.executor_started = False
        self.publisher = None
        self.active = True
        self.result = {'task_id': task_id, 'observation_id': self.observation_id,
                       'namespace': self.namespace, 'source_not_before_ns': self.barrier,
                       'task_deadline_monotonic_ns': int(deadline * 1e9),
                       'supervisor_pid': self.supervisor_pid, 'supervisor_start_ticks': self.supervisor_start_ticks,
                       'mode': 'execute' if execute else 'plan_only', 'completed_count': 0}
        output.mkdir(parents=True, exist_ok=False)
        self.snapshots = {name: str(output / f'effective_{name}_config.json') for name in configs}
        for name, config in configs.items():
            write_json(self.snapshots[name], asdict(config), source_paths)
        self.result['config_snapshots'] = self.snapshots
        self.qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def attach(self):
        def subscribe(kind, topic, callback, qos=None):
            def guarded(message):
                if not self.active:
                    return
                try:
                    callback(message)
                except Exception as exc:
                    self.callback_error = str(exc)
            self.subscriptions.append(self.node.create_subscription(kind, topic, guarded, qos or self.qos))
        for name in ('yolo_vision', 'rim_locator', 'grasp_executor'):
            subscribe(String, self.namespace + '/' + name + '/status',
                      lambda message, name=name: self.on_status(name, message))
        subscribe(String, self.configs['rim'].candidates_topic, self.on_candidates)
        subscribe(DisplayTrajectory, self.configs['grasp'].task_preview_topic, self.on_preview,
                  QoSProfile(depth=10))
        if self.configs['vision'].publish_debug:
            subscribe(Image, self.namespace + '/yolo_vision/overlay', self.save_image)
        if self.configs['rim'].publish_debug:
            subscribe(PointCloud2, self.namespace + '/rim_locator/base_cloud', self.save_cloud)
        self.publisher = self.node.create_publisher(String, self.configs['grasp'].input_topic, self.qos)

    def on_status(self, name, message):
        value = json.loads(message.data)
        if not context_matches(value, self.task_id, self.observation_id):
            return  # Foreign retained callbacks must never advance this round.
        stamp = value.get('source_stamp_ns')
        if stamp is not None:
            if type(stamp) is not int or stamp <= self.barrier:
                raise ValueError('status_before_observation_barrier')
            if self.batch is not None and stamp != self.batch['source_stamp_ns']:
                raise ValueError('status_observation_stamp_mismatch')
        if self.statuses.get(name) != value:
            print(name, json.dumps(value, ensure_ascii=False), flush=True)
            self.history.append({'node': name, 'status': value})
        self.statuses[name] = value

    def on_candidates(self, message):
        # Ignore correctly formed messages belonging to another observation.
        payload = decode_candidate_batch(message.data)
        if not context_matches(payload, self.task_id, self.observation_id):
            return
        payload = decode_candidate_batch(message.data, expected_task_id=self.task_id,
            expected_observation_id=self.observation_id, expected_frame_id=self.configs['grasp'].base_frame,
            expected_tool_frame=self.configs['grasp'].tool_frame, source_not_before_ns=self.barrier)
        if self.batch is not None:
            if payload != self.batch:
                raise ValueError('observation_changed_after_publication')
            return
        self.batch = payload
        write_json(self.output / 'candidates.json', payload, self.source_paths)

    def on_preview(self, message):
        preview = {'segments': len(message.trajectory),
                   'points': [len(t.joint_trajectory.points) for t in message.trajectory]}
        (self.previews if preview['segments'] == 6 else self.intermediate).append(preview)

    def save_image(self, message):
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if stamp <= self.barrier or 'detection_overlay' in self.result:
            return
        try:
            from PIL import Image as PillowImage
            if message.encoding not in ('bgr8', 'rgb8'):
                raise ValueError('unsupported_debug_image_encoding')
            image = PillowImage.frombytes('RGB', (message.width, message.height), bytes(message.data),
                'raw', 'BGR' if message.encoding == 'bgr8' else 'RGB', message.step, 1)
            path = self.output / 'detection_overlay.png'
            image.save(path)
            self.result['detection_overlay'] = str(path)
        except Exception as exc:
            self.result.setdefault('debug_capture_errors', []).append(str(exc))

    def save_cloud(self, message):
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if stamp <= self.barrier or 'base_cloud' in self.result:
            return
        try:
            import numpy as np
            from rim_locator.pointcloud import read_xyz_object_ids
            points, ids = read_xyz_object_ids(message)
            path = self.output / 'base_cloud.npz'
            np.savez_compressed(path, points=points, object_ids=ids,
                                frame_id=message.header.frame_id, stamp_ns=stamp)
            self.result['base_cloud'] = str(path)
        except Exception as exc:
            self.result.setdefault('debug_capture_errors', []).append(str(exc))

    def spin(self):
        if self.interrupted.is_set():
            raise RuntimeError('interrupted')
        if time.monotonic() >= self.deadline:
            raise TimeoutError('pipeline_timeout')
        rclpy.spin_once(self.node, timeout_sec=min(.05, max(0., self.deadline-time.monotonic())))
        if self.callback_error:
            raise RuntimeError('invalid_round_message:' + self.callback_error)
        for name, process in self.processes.items():
            if name != 'position_control' and process.poll() is not None:
                raise RuntimeError('child_process_exited:' + name)

    def start(self, name, *extra):
        path = self.root / ('run_grasp_position_control.sh' if name == 'position_control' else f'run_{name}.sh')
        command = [str(path), *map(str, extra)]
        if name != 'position_control':
            config_key = {'rim_locator': 'rim', 'yolo_vision': 'vision', 'grasp_executor': 'grasp'}[name]
            command += [f'config_path:={self.snapshots[config_key]}', f'namespace:={self.namespace}',
                        f'task_id:={self.task_id}', f'observation_id:={self.observation_id}']
            if name in ('yolo_vision', 'grasp_executor'):
                command.append(f'source_not_before_ns:={self.barrier}')
            if name == 'grasp_executor':
                command.append(f'task_deadline_monotonic_ns:={int(self.deadline*1e9)}')
                command.append(f'task_preview_topic:={self.configs["grasp"].task_preview_topic}')
                command += [f'supervisor_pid:={self.supervisor_pid}',
                            f'supervisor_start_ticks:={self.supervisor_start_ticks}']
        log = (self.output / (name + '.log')).open('x')
        self.logs.append(log)
        process = subprocess.Popen(command, cwd=self.root, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        self.processes[name] = process
        return process

    def wait_ready(self, name, status):
        startup_deadline = min(self.deadline, time.monotonic() + 35.)
        while self.statuses.get(name, {}).get('status') != status:
            if any(s.get('status') == 'failed' for s in self.statuses.values()):
                raise RuntimeError('pipeline_startup_failed')
            if time.monotonic() >= startup_deadline:
                raise TimeoutError('pipeline_startup_timeout:' + name)
            self.spin()

    def prepare_controller(self):
        process = self.start('position_control')
        stop = min(self.deadline, time.monotonic() + 240.)
        while process.poll() is None:
            if time.monotonic() >= stop:
                raise TimeoutError('controller_preparation_timeout')
            self.spin()
        if process.returncode != 0:
            raise RuntimeError('controller_preparation_failed')

    def run(self, controller_prepared):
        self.attach()
        self.start('rim_locator')
        self.wait_ready('rim_locator', 'waiting_input')
        self.start('yolo_vision')
        while True:
            self.spin()
            failed = any(s.get('status') in ('failed', 'candidates_exhausted') for s in self.statuses.values())
            if failed:
                # DDS does not order different topics. Keep the all-rejected
                # protocol message before retiring this round's subscriptions.
                rim_status = self.statuses.get('rim_locator', {})
                if rim_status.get('reason') == 'pose_not_found:no_valid_object' and self.batch is None:
                    delivery_deadline = min(self.deadline, time.monotonic() + 1.)
                    while self.batch is None and time.monotonic() < delivery_deadline:
                        self.spin()
                    if self.batch is None:
                        raise RuntimeError('missing_rejected_candidate_batch')
                self.result['kind'] = 'retry' if recoverable_failure(self.statuses) else 'failed'
                self.result['error'] = next((s.get('reason') or s['status'] for s in self.statuses.values()
                    if s.get('status') in ('failed', 'candidates_exhausted')), 'pipeline_failed')
                return controller_prepared
            vision = self.statuses.get('yolo_vision', {})
            if vision.get('status') == 'no_targets':
                if vision.get('detected_count') != 0 or self.batch is not None or self.executor_started:
                    raise RuntimeError('inconsistent_empty_observation')
                if type(vision.get('source_stamp_ns')) is not int:
                    raise RuntimeError('missing_observation_stamp')
                self.result['kind'] = 'empty'
                return controller_prepared
            if (not self.executor_started and self.batch is not None
                    and vision.get('status') == 'success'
                    and self.statuses.get('rim_locator', {}).get('status') == 'success'):
                if not self.batch['objects']:
                    raise RuntimeError('empty_localized_batch')
                stamps = [vision.get('source_stamp_ns'), self.statuses['rim_locator'].get('source_stamp_ns')]
                if any(stamp != self.batch['source_stamp_ns'] for stamp in stamps):
                    raise RuntimeError('perception_stamp_mismatch')
                accepted_ids = {obj['object_id'] for obj in vision.get('objects', [])}
                located_ids = {obj['object_id'] for obj in self.batch['objects'] + self.batch['rejected']}
                if accepted_ids != located_ids or len(accepted_ids) != vision.get('object_count'):
                    raise RuntimeError('perception_object_mismatch')
                self.selected = select_candidate_groups(self.batch, self.configs['grasp'].grasp_offset, self.execute)
                write_json(self.output / 'selected_candidates.json', self.selected, self.source_paths)
                if self.execute and not controller_prepared:
                    self.prepare_controller()
                    controller_prepared = True
                self.start('grasp_executor', '--execute' if self.execute else '--plan-only')
                self.executor_started = True
                self.wait_ready('grasp_executor', 'waiting_target')
                self.publisher.publish(String(data=encode_candidate_batch(self.selected)))
            status = self.statuses.get('grasp_executor', {})
            if status.get('status') in ('success', 'planned'):
                expected = 'success' if self.execute else 'planned'
                count = len(self.selected['objects']) if self.selected else 0
                if status['status'] != expected or status.get('object_count') != count:
                    raise RuntimeError('executor_result_mismatch')
                expected_completed = 1 if self.execute else 0
                if status.get('completed_count') != expected_completed:
                    raise RuntimeError('completed_count_mismatch')
                self.result['completed_count'] = expected_completed
                # Status and DisplayTrajectory are independent topics; allow delivery to settle.
                drain_until = min(self.deadline, time.monotonic() + .5)
                while time.monotonic() < drain_until:
                    self.spin()
                if len(self.previews) != count:
                    raise RuntimeError('preview_batch_mismatch')
                self.result['kind'] = 'completed' if self.execute else 'planned'
                return controller_prepared

    def close(self):
        self.active = False
        errors = stop_owned_processes(self.processes)
        for subscription in self.subscriptions:
            try:
                self.node.destroy_subscription(subscription)
            except Exception as exc:
                errors.append('subscription_cleanup_failed:' + str(exc))
        if self.publisher is not None:
            try:
                self.node.destroy_publisher(self.publisher)
            except Exception as exc:
                errors.append('publisher_cleanup_failed:' + str(exc))
        for log in self.logs:
            try:
                log.close()
            except OSError as exc:
                errors.append('log_close_failed:' + str(exc))
        executor = self.statuses.get('grasp_executor', {})
        self.result.update(statuses=self.statuses, history=self.history,
            detected_count=self.statuses.get('yolo_vision', {}).get('detected_count'),
            target_count=len(self.batch['objects']) if self.batch else 0,
            selected_object_ids=[g['object_id'] for g in self.selected['objects']] if self.selected else [],
            preview_batches=self.previews, intermediate_previews=self.intermediate,
            candidate_attempt_count=executor.get('candidate_attempt_count', 0),
            rejected={name: self.statuses.get(name, {}).get('rejected', [])
                      for name in ('yolo_vision', 'rim_locator')})
        try:
            write_json(self.output / 'candidate_attempts.json', executor.get('candidate_failures', []), self.source_paths)
        except OSError as exc:
            errors.append('candidate_log_failed:' + str(exc))
        if errors:
            self.result.update(kind='failed', cleanup_errors=errors, error=';'.join(errors))
        try:
            write_json(self.output / 'result.json', self.result, self.source_paths)
        except OSError as exc:
            errors.append('round_result_write_failed:' + str(exc))
            self.result.update(kind='failed', cleanup_errors=errors, error=';'.join(errors))
        return errors


def main(execute=False):
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', default=('debug_output/place_execution/' if execute else
                         'debug_output/place_validation/') + time.strftime('%Y%m%d_%H%M%S'))
    parser.add_argument('--timeout', type=float, default=None)
    parser.add_argument('--vision-config', default=None)
    parser.add_argument('--execute', action='store_true', default=execute)
    parser.add_argument('--max-reobservations', type=int, default=2)
    parser.add_argument('--max-observation-rounds', type=int, default=30)
    parser.add_argument('--max-completed-objects', type=int, default=20)
    args = parser.parse_args()
    execute = args.execute
    timeout = args.timeout if args.timeout is not None else (900. if execute else 360.)
    output = Path(args.output_dir).expanduser().resolve()
    vision_default = root / 'src/yolo_vision/config/place_config.json'
    rim_path = root / 'src/rim_locator/config/place_config.json'
    grasp_path = root / 'src/grasp_executor/config/place_config.json'
    override = Path(args.vision_config).expanduser().resolve() if args.vision_config else None
    try:
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be positive and finite')
        limits = RoundLimits(args.max_reobservations, args.max_observation_rounds, args.max_completed_objects)
        vision, rim, grasp = load_effective_config(vision_default, override), load_rim_config(rim_path), load_grasp_config(grasp_path)
        sources = {p.resolve() for p in (vision_default, rim_path, grasp_path, override) if p is not None}
        first_output = output / 'round_0001'
        protected_outputs = [output / 'task_config.json', output / 'result.json',
                             *[first_output / f'effective_{name}_config.json' for name in ('vision', 'rim', 'grasp')]]
        if any(p.resolve() in sources for p in protected_outputs):
            raise ValueError('snapshot_overwrites_source_config')
        if any(p.exists() for p in (output / 'result.json', output / 'task_config.json', first_output)):
            raise ValueError('output_directory_already_used')
    except (OSError, ValueError, TypeError) as exc:
        parser.error('invalid pipeline config: ' + str(exc))
    task_id = uuid.uuid4().hex
    budget = RoundBudget(time.monotonic() + timeout, limits)
    result = {'task_id': task_id, 'mode': 'execute' if execute else 'plan_only',
              'hardware_motion_requested': execute, 'selection': 'all_detected',
              'vision_config_sources': {'base': str(vision_default), 'override': str(override) if override else None},
              'pick_region': vision.inference_roi, 'placement_region_separation_verified': False,
              'grasp_verified': False, 'placement_verified': False, 'rounds': [],
              'passed': False, 'outcome': 'failed', 'completed_count': 0}
    interrupted, previous = Event(), {}
    node, initialized, controller_prepared = None, False, False
    owns_output = False
    try:
        with TaskLock():
            output.mkdir(parents=True, exist_ok=True)
            write_json(output / 'task_config.json', {'task_id': task_id, 'timeout': timeout,
                'limits': asdict(limits), 'vision': asdict(vision), 'rim': asdict(rim),
                'grasp': asdict(replace(grasp, plan_only=not execute)),
                'vision_config_sources': result['vision_config_sources']}, sources)
            owns_output = True
            print('抓放每次只处理最近一只；完成退回后重新观测。预览只规划一份快照。',
                  '取物 ROI:', vision.inference_roi, flush=True)
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            initialized = True
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                previous[sig] = signal.getsignal(sig)
                signal.signal(sig, lambda signum, frame: interrupted.set())
            node = Node('live_place_pipeline_test')
            discovery_until = min(budget.deadline, time.monotonic() + 1.)
            while time.monotonic() < discovery_until:
                if interrupted.is_set():
                    raise RuntimeError('interrupted')
                rclpy.spin_once(node, timeout_sec=.05)
            busy = [namespace.rstrip('/') + '/' + name for name, namespace in node.get_node_names_and_namespaces()
                    if name in ('yolo_vision', 'rim_locator', 'grasp_executor')]
            if busy:
                raise RuntimeError('existing project nodes: ' + ', '.join(sorted(busy)))
            while True:
                if interrupted.is_set():
                    raise RuntimeError('interrupted')
                number = budget.begin_round(time.monotonic())
                barrier = node.get_clock().now().nanoseconds
                if type(barrier) is not int or barrier <= 0:
                    raise RuntimeError('invalid_observation_clock')
                if result['rounds']:
                    last = result['rounds'][-1]
                    prior_stamps = [last['source_not_before_ns']] + [
                        status['source_stamp_ns'] for status in last['statuses'].values()
                        if type(status.get('source_stamp_ns')) is int]
                    if barrier <= max(prior_stamps):
                        raise RuntimeError('observation_clock_did_not_advance')
                namespace = f'/place_demo_{task_id}/round_{number:04d}'
                configs = round_configs(vision, rim, grasp, namespace, timeout, execute)
                observation = ObservationRound(node, root, output / f'round_{number:04d}', task_id, number,
                    configs, budget.deadline, barrier, execute, interrupted, sources)
                try:
                    controller_prepared = observation.run(controller_prepared)
                except Exception as exc:
                    observation.result.update(kind='failed', error=str(exc))
                finally:
                    observation.close()
                current = observation.result
                result['rounds'].append(current)
                if 'initial_detected_count' not in result and current['detected_count'] is not None:
                    result['initial_detected_count'] = current['detected_count']
                # Keep these first-observation aliases for existing diagnostics readers.
                if number == 1:
                    result.update(config_snapshots=current['config_snapshots'],
                        effective_vision_config=current['config_snapshots']['vision'],
                        detected_count=current['detected_count'], target_count=current['target_count'])
                kind = current['kind']
                has_rejections = any(current['rejected'].values())
                if current['completed_count']:
                    budget.complete_object(has_rejections)
                if kind == 'completed':
                    continue
                if kind == 'empty':
                    result['passed'], result['outcome'], result['error'] = budget.empty_result()
                    break
                if kind == 'planned':
                    result.update(passed=not has_rejections, outcome='planned' if not has_rejections else 'partial',
                                  error='' if not has_rejections else 'rejected_targets')
                    break
                if kind == 'retry' and budget.retry_observation(time.monotonic()):
                    continue
                result['error'] = ('reobservation_budget_exhausted' if kind == 'retry' else current.get('error', 'pipeline_failed'))
                result['outcome'] = ('partial' if budget.completed_count or
                    (not execute and current['preview_batches']) else 'failed')
                break
    except Exception as exc:
        result.update(passed=False, error=str(exc), outcome='partial' if budget.completed_count else 'failed')
    finally:
        result.update(completed_count=budget.completed_count, observation_count=budget.observation_count,
            reobservations_without_progress=budget.reobservations_without_progress,
            unresolved_rejections=budget.unresolved_rejections,
            candidate_attempt_count=sum(r.get('candidate_attempt_count', 0) for r in result['rounds']),
            preview_batches=[p for r in result['rounds'] for p in r['preview_batches']],
            intermediate_previews=[p for r in result['rounds'] for p in r['intermediate_previews']])
        result['stop_reason'] = result.get('error', '')
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        if owns_output and not (output / 'result.json').exists():
            write_json(output / 'result.json', result, sources)
    print('Result:', output / 'result.json', flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main('--execute' in os.sys.argv))

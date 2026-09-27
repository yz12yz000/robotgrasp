#!/usr/bin/env python3
"""Exercise actual rim nodes and retained grouped messages in an isolated DDS domain.

Only rim_locator Python processes and synthetic base_link point clouds are used.
No model inference, camera, robot driver, MoveIt, arm or gripper is started.
Source ROS and this workspace's setup before running. Results stay under /tmp.
"""
from dataclasses import asdict, replace
import json
import os
from pathlib import Path
import random
import signal
import subprocess
import sys
import tempfile
import time
import uuid
from isolated_ros import IsolatedRosTestLock


def main():
    test_lock = IsolatedRosTestLock()
    # Domain/transport isolation precedes every ROS import and context creation.
    previous_domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    domain = random.SystemRandom().choice([value for value in range(150, 220)
                                          if value != previous_domain])
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    for name in ('ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                 'FASTDDS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI'):
        os.environ.pop(name, None)
    root = Path(__file__).resolve().parents[1]
    source_path = str(root / 'src/rim_locator')
    sys.path.insert(0, source_path)
    os.environ['PYTHONPATH'] = source_path + os.pathsep + os.environ.get('PYTHONPATH', '')

    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseArray
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header, String
    from rim_locator.candidate_protocol import decode_candidate_batch
    from rim_locator.config import load_config
    from rim_locator.pointcloud import make_cloud

    directory = Path(tempfile.mkdtemp(prefix='observation-transport-test-'))
    result = {'passed': False, 'hardware_connected': False, 'camera_started': False,
              'model_inference_started': False, 'domain': domain, 'checks': []}
    print('Test logs:', directory, 'ROS_DOMAIN_ID:', domain, flush=True)
    processes, logs, subscriptions = [], [], []
    node, initialized = None, False
    statuses, received = {}, {}
    session = uuid.uuid4().hex
    rounds = []

    def check(condition, name):
        if not condition:
            raise AssertionError(name)
        result['checks'].append(name)
        print('PASS', name, flush=True)

    def spin_until(predicate, description, timeout=20.):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError(description)
            rclpy.spin_once(node, timeout_sec=.05)
            if any(process.poll() is not None for process in processes):
                raise RuntimeError('rim_process_exited:' + description)
            failures = [status for status in statuses.values() if status.get('status') == 'failed']
            if failures:
                raise RuntimeError('rim_failed:' + json.dumps(failures, ensure_ascii=False))

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        spin_until(lambda: time.monotonic() >= deadline, 'delivery_settle', seconds + 2.)

    def on_status(index, message):
        value = json.loads(message.data)
        current = rounds[index]
        if (value.get('task_id') != current['task_id']
                or value.get('observation_id') != current['observation_id']):
            raise RuntimeError('status_context_mismatch')
        statuses[index] = value

    def launch(index):
        current = rounds[index]
        log = (directory / f'rim_{index + 1}.log').open('x')
        logs.append(log)
        command = [sys.executable, '-u', '-c', 'from rim_locator.main import main; main()',
                   '--ros-args', '-r', '__ns:=' + current['namespace'],
                   '-p', 'config_path:=' + str(current['config_path']),
                   '-p', 'task_id:=' + current['task_id'],
                   '-p', 'observation_id:=' + current['observation_id']]
        processes.append(subprocess.Popen(command, cwd=root, stdout=log, stderr=subprocess.STDOUT,
                                          env=os.environ.copy(), start_new_session=True))

    def synthetic_cloud(objects, stamp):
        angle, depth = np.meshgrid(np.linspace(0., 2. * np.pi, 240, endpoint=False),
                                   np.linspace(0., .05, 40))
        clouds, ids = [], []
        for object_id, center in objects:
            cloud = np.c_[(center[0] + .085 * np.cos(angle)).ravel(),
                          (center[1] + .085 * np.sin(angle)).ravel(), (.30 - depth).ravel()]
            clouds.append(cloud)
            ids.append(np.full(len(cloud), object_id, dtype=np.uint32))
        return make_cloud(Header(frame_id='base_link', stamp=stamp), np.concatenate(clouds), np.concatenate(ids))

    def late_subscribe_outputs(index):
        current = rounds[index]
        received[index] = {'candidate': [], 'pose': []}
        subscriptions.append(node.create_subscription(String, current['config'].candidates_topic,
            received[index]['candidate'].append, qos))
        subscriptions.append(node.create_subscription(PoseArray, current['config'].poses_topic,
            received[index]['pose'].append, qos))
        spin_until(lambda: bool(received[index]['candidate']) and bool(received[index]['pose']),
                   f'late_retained_outputs_{index + 1}')

    def validate_outputs(index, expected_ids, stamp):
        current, outputs = rounds[index], received[index]
        batch = decode_candidate_batch(outputs['candidate'][0].data,
            expected_task_id=current['task_id'], expected_observation_id=current['observation_id'],
            expected_frame_id='base_link', expected_tool_frame=current['config'].tool_frame)
        poses = outputs['pose'][0]
        source_ns = stamp.sec * 10**9 + stamp.nanosec
        check(batch['source_stamp_ns'] == source_ns and poses.header.stamp == stamp
              and statuses[index]['source_stamp_ns'] == source_ns,
              f'round_{index + 1}_preserves_sensor_stamp')
        check([group['object_id'] for group in batch['objects']] == expected_ids,
              f'round_{index + 1}_keeps_object_identity_and_near_to_far_order')
        check(len(poses.poses) == len(batch['objects']) == len(expected_ids)
              and not batch['rejected'], f'round_{index + 1}_pose_array_counts_objects')
        for group, pose in zip(batch['objects'], poses.poses):
            check(1 < len(group['candidates']) <= current['config'].max_grasp_candidates,
                  f'round_{index + 1}_object_{group["object_id"]}_has_grouped_candidates')
            first = group['candidates'][0]
            np.testing.assert_allclose(first['position'], [pose.position.x, pose.position.y, pose.position.z])
            np.testing.assert_allclose(first['orientation'],
                [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        (directory / f'round_{index + 1}_candidates.json').write_text(
            json.dumps(batch, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        return batch

    try:
        rclpy.init()
        initialized = True
        node = Node('observation_transport_test')
        spin_for(1.5)
        others = [name for name in node.get_node_names() if name != node.get_name()]
        if others:
            raise RuntimeError('test_domain_occupied:rerun_to_choose_another_domain:' + ','.join(others))
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        base = load_config(root / 'src/rim_locator/config/place_config.json')
        for index in range(2):
            namespace = f'/observation_transport_{session}/round_{index + 1:04d}'
            config = replace(base, input_topic=namespace + '/object_cloud',
                output_topic=namespace + '/rim_point', poses_topic=namespace + '/grasp_poses',
                candidates_topic=namespace + '/grasp_candidates', publish_debug=False,
                input_timeout=30.)
            current = dict(namespace=namespace, task_id=f'{session}_task_{index + 1}',
                observation_id=f'{session}_observation_{index + 1}', config=config,
                config_path=directory / f'rim_{index + 1}_config.json')
            current['config_path'].write_text(json.dumps(asdict(config), indent=2), encoding='utf-8')
            current['publisher'] = node.create_publisher(PointCloud2, config.input_topic, qos)
            rounds.append(current)
            subscriptions.append(node.create_subscription(String, namespace + '/rim_locator/status',
                lambda message, index=index: on_status(index, message), qos))

        launch(0)
        spin_until(lambda: statuses.get(0, {}).get('status') == 'waiting_input', 'first_rim_ready')
        # A cloud on the second namespace must not consume the first node's
        # one-shot observation. This cached sample is replaced before node 2 starts.
        rounds[1]['publisher'].publish(synthetic_cloud([(3, (.45, -.10))], node.get_clock().now().to_msg()))
        spin_for(.35)
        check(statuses[0]['status'] == 'waiting_input', 'foreign_namespace_cloud_does_not_consume_round')

        first_stamp = node.get_clock().now().to_msg()
        rounds[0]['publisher'].publish(synthetic_cloud([(17, (.60, .10)), (3, (.35, .10))], first_stamp))
        spin_until(lambda: statuses.get(0, {}).get('status') == 'success', 'first_rim_success')
        # Attach after success: both outputs must be retained and complete.
        late_subscribe_outputs(0)
        first_batch = validate_outputs(0, [3, 17], first_stamp)
        check(len(received[0]['candidate']) == len(received[0]['pose']) == 1,
              'late_subscriber_receives_one_retained_batch')

        second_stamp = node.get_clock().now().to_msg()
        second_cloud = synthetic_cloud([(3, (.45, -.10))], second_stamp)
        rounds[1]['publisher'].publish(second_cloud)
        # Publish input before creating its consumer as the production pipeline
        # also relies on durable, one-shot transport between processes.
        launch(1)
        spin_until(lambda: statuses.get(1, {}).get('status') == 'success', 'second_rim_retained_input')
        late_subscribe_outputs(1)
        second_batch = validate_outputs(1, [3], second_stamp)
        check(second_batch['source_stamp_ns'] > first_batch['source_stamp_ns']
              and second_batch['task_id'] != first_batch['task_id']
              and second_batch['observation_id'] != first_batch['observation_id'],
              'new_observation_can_reuse_frame_local_object_id')
        check(len(received[0]['candidate']) == len(received[0]['pose']) == 1,
              'second_round_does_not_republish_first_round_outputs')

        rounds[0]['publisher'].publish(second_cloud)
        rounds[1]['publisher'].publish(second_cloud)
        spin_for(.35)
        check(all(len(received[index]['candidate']) == len(received[index]['pose']) == 1
                  for index in range(2)), 'one_shot_nodes_ignore_additional_input')
        result['passed'] = True
    except BaseException as exc:
        result['error'] = f'{type(exc).__name__}:{exc}'
        print('FAIL', result['error'], flush=True)
    finally:
        cleanup_errors = []
        for process in processes:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        for process in processes:
            for timeout, escalation in ((10., signal.SIGTERM), (3., signal.SIGKILL), (2., None)):
                try:
                    process.wait(timeout=timeout)
                    break
                except subprocess.TimeoutExpired:
                    if escalation is None:
                        cleanup_errors.append('rim_process_shutdown_unconfirmed')
                    else:
                        try:
                            os.killpg(process.pid, escalation)
                        except ProcessLookupError:
                            pass
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()
        for log in logs:
            log.close()
        if cleanup_errors:
            result.update(passed=False, cleanup_errors=cleanup_errors)
        result['statuses'] = statuses
        (directory / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
        test_lock.close()
    print('Result:', directory / 'result.json', flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Observe all targets once, then grasp/place each one from near to far.

The default mode only plans and publishes MoveIt trajectories.  It never sends
arm or gripper commands.  ``--execute`` is reserved for the separate wrapper
after the operator has enabled the robot position controller.
"""

import argparse
from dataclasses import asdict, replace
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from threading import Event

import rclpy
from geometry_msgs.msg import PoseArray
from moveit_msgs.msg import DisplayTrajectory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from yolo_vision.config import load_effective_config
from rim_locator.config import load_config as load_rim_config
from grasp_executor.config import load_config as load_grasp_config


def main(execute=False):
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', default=('debug_output/place_execution/' if execute else
                                                 'debug_output/place_validation/') + time.strftime('%Y%m%d_%H%M%S'))
    parser.add_argument('--timeout', type=float, default=900. if execute else 360.)
    parser.add_argument('--vision-config', default=None)
    parser.add_argument('--execute', action='store_true', default=execute)
    args = parser.parse_args()
    execute = args.execute
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be a positive finite number')
    place_config = root / 'src/grasp_executor/config/place_config.json'
    rim_config = root / 'src/rim_locator/config/place_config.json'
    vision_default = root / 'src/yolo_vision/config/place_config.json'
    vision_override = Path(args.vision_config).expanduser().resolve() if args.vision_config else None
    output = Path(args.output_dir).expanduser().resolve()
    try:
        vision = load_effective_config(vision_default, vision_override)
        rim = load_rim_config(rim_config)
        grasp = load_grasp_config(place_config)
        # Consumers start before vision. Only extend their first-input wait.
        rim = replace(rim, input_timeout=max(rim.input_timeout, args.timeout + 35.))
        grasp = replace(grasp, input_timeout=max(grasp.input_timeout, args.timeout + 35.))
        snapshots = {name: str(output / f'effective_{name}_config.json')
                     for name in ('vision', 'rim', 'grasp')}
        source_paths = {path.resolve() for path in
                        (vision_default, rim_config, place_config, vision_override) if path is not None}
        # Reusing an output directory must not overwrite an input configuration,
        # including an input snapshot or a destination symlink to a source.
        for path in snapshots.values():
            if Path(path).resolve() in source_paths:
                raise ValueError(f'snapshot_overwrites_source_config:{path}')
        output.mkdir(parents=True, exist_ok=True)
        for name, config in (('vision', vision), ('rim', rim), ('grasp', grasp)):
            path = Path(snapshots[name])
            path.write_text(json.dumps(asdict(config), ensure_ascii=False, indent=2), encoding='utf-8')
    except (OSError, ValueError, TypeError) as exc:
        parser.error(f'invalid pipeline config: {exc}')
    sources = {'base': str(vision_default),
               'override': str(vision_override) if vision_override else None}
    print('观察当前视野：0 个不动作，N 个按近到远逐个抓放。',
          'ROI:', vision.inference_roi, '配置来源:', json.dumps(sources, ensure_ascii=False), flush=True)

    rclpy.init()
    interrupted = Event()
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    for sig in previous:
        signal.signal(sig, lambda signum, frame: interrupted.set())
    node = Node('live_place_pipeline_test')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    preview_qos = QoSProfile(depth=10)
    statuses, history, previews, poses, intermediate_previews = {}, [], [], [], []
    processes, logs = [], []
    result = {'mode': 'execute' if execute else 'plan_only',
              'hardware_motion_requested': bool(execute),
              'selection': 'all_detected', 'vision_config_sources': sources,
              'effective_vision_config': snapshots['vision'], 'config_snapshots': snapshots}

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if interrupted.is_set():
                raise RuntimeError('interrupted')
            rclpy.spin_once(node, timeout_sec=.05)

    def report_status(key, message):
        value = json.loads(message.data)
        if statuses.get(key) != value:
            print(key, json.dumps(value, ensure_ascii=False), flush=True)
            history.append({'node': key, 'status': value})
        statuses[key] = value

    def on_pose(message):
        poses.append({'frame': message.header.frame_id, 'count': len(message.poses),
                      'stamp': message.header.stamp.sec + message.header.stamp.nanosec / 1e9})

    def on_preview(message):
        preview = {'segments': len(message.trajectory), 'points': [len(t.joint_trajectory.points)
                    for t in message.trajectory]}
        # MoveIt's planning pipeline also publishes each individual segment on
        # this topic. Only our complete six-stage batches count as task previews.
        (previews if preview['segments'] == 6 else intermediate_previews).append(preview)

    def save_detection_image(message):
        try:
            from PIL import Image as PillowImage
            if message.encoding not in ('bgr8', 'rgb8'):
                raise ValueError('unsupported_debug_image_encoding')
            image = PillowImage.frombytes('RGB', (message.width, message.height), bytes(message.data),
                                          'raw', 'BGR' if message.encoding == 'bgr8' else 'RGB', message.step, 1)
            image.save(output / 'detection_overlay.png')
            result['detection_overlay'] = str(output / 'detection_overlay.png')
        except Exception as exc:
            result.setdefault('debug_capture_errors', []).append(str(exc))

    def save_base_cloud(message):
        try:
            import numpy as np
            from rim_locator.pointcloud import read_xyz_object_ids
            points, ids = read_xyz_object_ids(message)
            path = output / 'base_cloud.npz'
            np.savez_compressed(path, points=points, object_ids=ids,
                                frame_id=message.header.frame_id,
                                stamp_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec)
            result['base_cloud'] = str(path)
        except Exception as exc:
            result.setdefault('debug_capture_errors', []).append(str(exc))

    def start(name, *command):
        log = (output / (name + '.log')).open('w')
        logs.append(log)
        processes.append(subprocess.Popen([str(x) for x in command], cwd=root,
                                          stdout=log, stderr=subprocess.STDOUT,
                                          start_new_session=True))

    try:
        for key in ('grasp_executor', 'rim_locator', 'yolo_vision'):
            node.create_subscription(String, '/' + key + '/status',
                                     lambda msg, key=key: report_status(key, msg), qos)
        node.create_subscription(PoseArray, '/rim_locator/grasp_poses', on_pose, qos)
        node.create_subscription(DisplayTrajectory, '/display_planned_path', on_preview, preview_qos)
        if vision.publish_debug:
            node.create_subscription(Image, '/yolo_vision/overlay', save_detection_image, qos)
        if rim.publish_debug:
            node.create_subscription(PointCloud2, '/rim_locator/base_cloud', save_base_cloud, qos)
        spin_for(1.0)
        busy = set(node.get_node_names()) & {'grasp_executor', 'rim_locator', 'yolo_vision'}
        if busy:
            raise RuntimeError('existing project nodes: ' + ', '.join(sorted(busy)))
        start('rim_locator', root / 'run_rim_locator.sh', f"config_path:={snapshots['rim']}")
        deadline = time.monotonic() + 35
        while statuses.get('rim_locator', {}).get('status') != 'waiting_input':
            if time.monotonic() > deadline:
                raise TimeoutError('pipeline_startup_timeout')
            if any(v.get('status') == 'failed' for v in statuses.values()):
                raise RuntimeError('pipeline_startup_failed')
            spin_for(.1)
        yolo_args = [f"config_path:={snapshots['vision']}"]
        start('yolo_vision', root / 'run_yolo_vision.sh', *yolo_args)
        deadline = time.monotonic() + args.timeout
        executor_started = False
        while time.monotonic() < deadline:
            spin_for(.1)
            if any(v.get('status') == 'failed' for v in statuses.values()):
                break
            if statuses.get('yolo_vision', {}).get('status') == 'no_targets':
                print('当前视野检测到 0 个目标，本次结束，不启动抓取。', flush=True)
                break
            if (not executor_started and poses
                    and statuses.get('yolo_vision', {}).get('status') == 'success'
                    and statuses.get('rim_locator', {}).get('status') == 'success'):
                count = poses[0]['count']
                if count <= 0:
                    raise RuntimeError('empty_localized_batch')
                print(f'检测到 {statuses.get("yolo_vision", {}).get("detected_count", count)} 个目标，'
                      f'可定位 {count} 个，按近到远依次处理。', flush=True)
                if execute:
                    subprocess.run([str(root / 'run_grasp_position_control.sh')], cwd=root,
                                   check=True, timeout=min(240., max(.1, deadline-time.monotonic())))
                if interrupted.is_set():
                    raise RuntimeError('interrupted')
                if time.monotonic() >= deadline:
                    raise TimeoutError('pipeline_timeout')
                start('grasp_executor', root / 'run_grasp_executor.sh',
                      '--execute' if execute else '--plan-only', f"config_path:={snapshots['grasp']}")
                executor_started = True
            if statuses.get('grasp_executor', {}).get('status') in ('planned', 'success'):
                break
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('child_process_exited')
        # Status and preview are different topics; drain pending callbacks.
        spin_for(.5)
        result['target_batches'] = poses
        result['preview_batches'] = previews
        result['intermediate_previews'] = intermediate_previews
        detected = statuses.get('yolo_vision', {})
        result['detected_count'] = detected.get('detected_count')
        result['target_count'] = poses[0]['count'] if poses else 0
        result['completed_count'] = statuses.get('grasp_executor', {}).get('completed_count', 0)
        result['rejected'] = {name: statuses.get(name, {}).get('rejected', [])
                              for name in ('yolo_vision', 'rim_locator')}
        empty = detected.get('status') == 'no_targets' and detected.get('detected_count') == 0
        failures = {name: value for name, value in statuses.items() if value.get('status') == 'failed'}
        complete = (statuses.get('yolo_vision', {}).get('status') == 'success'
                            and statuses.get('rim_locator', {}).get('status') == 'success'
                            and statuses.get('grasp_executor', {}).get('status') == ('success' if execute else 'planned')
                            and len(poses) == 1 and len(previews) == poses[0]["count"]
                            and all(p['segments'] == 6 for p in previews)
                            and (not execute or result['completed_count'] == poses[0]['count']))
        result['passed'] = ((empty and not failures and not executor_started and not poses and not previews)
                            or (complete and result['detected_count'] == result['target_count']
                                and not any(result['rejected'].values())))
        result['outcome'] = ('no_targets' if empty and result['passed'] else 'completed' if result['passed']
                             else 'partial' if complete else 'failed')
        if not result['passed']:
            result['error'] = ('rejected_targets' if complete else
                               'pipeline_failed' if failures else
                               'preview_batch_mismatch' if statuses.get('grasp_executor', {}).get('status')
                               in ('planned', 'success') else 'pipeline_timeout')
            result['failures'] = failures
    except Exception as exc:
        result['error'] = str(exc)
        result['passed'] = False
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                result['forced_shutdown'] = True
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        result['statuses'], result['history'] = statuses, history
        (output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    print('Result:', output / 'result.json', flush=True)
    return 0 if result.get('passed') else 1


if __name__ == '__main__':
    raise SystemExit(main('--execute' in os.sys.argv))

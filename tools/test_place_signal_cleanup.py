#!/usr/bin/env python3
"""Real runner SIGTERM/SIGHUP cleanup in empty DDS; never publish a target."""
import json
import os
from pathlib import Path
import random
import signal
import subprocess
import sys
import tempfile
import time

from isolated_ros import IsolatedRosTestLock


def main():
    with IsolatedRosTestLock():
        return run()


def run():
    old_domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    domain = random.SystemRandom().choice([x for x in range(150, 220) if x != old_domain])
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1', PYTHONNOUSERSITE='1')
    for key in ('ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                'FASTDDS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI'):
        os.environ.pop(key, None)
    import rclpy
    from rclpy.node import Node
    from test_live_place_pipeline import process_group_alive, stop_owned_processes
    root = Path(__file__).resolve().parents[1]
    output = Path(tempfile.mkdtemp(prefix='place-signal-cleanup-'))
    result = dict(passed=False, domain=domain, hardware_started=False, targets_published=0, checks=[])
    print('Test logs:', output, 'ROS_DOMAIN_ID:', domain, flush=True)
    rclpy.init(domain_id=domain)
    node = Node('place_signal_cleanup_test')
    process, stream = None, None
    try:
        until = time.monotonic() + 1.5
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        if any(name != node.get_name() for name in node.get_node_names()):
            raise RuntimeError('chosen_test_domain_occupied')
        for sig in (signal.SIGTERM, signal.SIGHUP):
            task_output = output / sig.name
            log = output / (sig.name + '.log')
            stream = log.open('x')
            process = subprocess.Popen([str(root / 'run_pipeline_place_test.sh'), '--output-dir',
                str(task_output), '--timeout', '45'], cwd=root, stdin=subprocess.DEVNULL,
                stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 30.
            while 'yolo_vision {"status": "ready"' not in log.read_text():
                if process.poll() is not None:
                    raise RuntimeError('runner_exited_before_signal:' + log.read_text()[-2000:])
                if time.monotonic() >= deadline:
                    raise TimeoutError('runner_did_not_wait_for_observation')
                rclpy.spin_once(node, timeout_sec=.05)
            process.send_signal(sig)
            process.wait(timeout=27.)
            stream.close()
            stream = None
            outcome = json.loads((task_output / 'result.json').read_text())
            assert outcome['completed_count'] == 0 and outcome['passed'] is False
            assert outcome['stop_reason'] == 'interrupted', outcome
            assert all(not item.get('cleanup_errors') for item in outcome['rounds'])
            namespace = outcome['rounds'][0]['namespace']
            deadline = time.monotonic() + 3.
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                if not any(ns == namespace for name, ns in node.get_node_names_and_namespaces()):
                    break
            assert not any(ns == namespace for name, ns in node.get_node_names_and_namespaces())
            assert not process_group_alive(process.pid)
            result['checks'].append(dict(signal=sig.name, returncode=process.returncode,
                                         stop_reason=outcome['stop_reason'], project_nodes_cleaned=True))
            print('PASS', sig.name, 'records interruption and cleans every owned node', flush=True)
            process = None
        result['passed'] = True
    except BaseException as exc:
        result['error'] = f'{type(exc).__name__}:{exc}'
        print('FAIL', result['error'], flush=True)
    finally:
        if process is not None:
            stop_owned_processes({'signal_test_runner': process})
        if stream is not None:
            stream.close()
        node.destroy_node()
        rclpy.shutdown()
        (output / 'result.json').write_text(json.dumps(result, indent=2))
    print('Result:', output / 'result.json', flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

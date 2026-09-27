#!/usr/bin/env python3
"""Verify real launch -> node -> topic contracts without delivering any targets.

Runs only ros2 launch for rim_locator and a plan-only grasp_executor in an empty
localhost DDS domain. No camera, model, hardware driver, MoveIt server, cloud or
candidate publisher is started. Parameter services and ROS graph/status provide
the evidence. Source ROS and the built workspace setup before running.
"""
from dataclasses import asdict, replace
import json
import os
from pathlib import Path
import random
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import uuid


def group_alive(pgid):
    """Count live descendants even when their ros2 launch leader has exited."""
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields = path.read_text().rsplit(')', 1)[1].split()
            if int(fields[2]) == pgid and fields[0] not in ('Z', 'X'):
                return True
        except (FileNotFoundError, ProcessLookupError):
            pass
    return False


def stop_groups(processes):
    pending = dict(processes)
    escalated, errors = [], []
    for sig, allowance in ((signal.SIGINT, 4.), (signal.SIGTERM, 1.), (signal.SIGKILL, 1.)):
        if sig != signal.SIGINT and pending:
            escalated.extend(pending)
        for name, process in pending.items():
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                pass
            except OSError as exc:
                errors.append(f'signal_failed:{name}:{exc}')
        deadline = time.monotonic() + allowance
        while pending:
            for name, process in list(pending.items()):
                process.poll()
                if not group_alive(process.pid):
                    pending.pop(name)
            if not pending or time.monotonic() >= deadline:
                break
            time.sleep(.03)
        if not pending:
            break
    errors.extend('process_group_still_alive:' + name for name in pending)
    return {'errors': errors, 'escalated': sorted(set(escalated)),
            'returncodes': {name: process.poll() for name, process in processes.items()},
            'all_groups_stopped': not pending}


def _run_locked():
    started = time.monotonic()
    deadline = started + 24.  # At most another six seconds for all process groups.
    previous_domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    domain = random.SystemRandom().choice([value for value in range(150, 220) if value != previous_domain])
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['PYTHONNOUSERSITE'] = '1'
    for name in ('ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                 'FASTDDS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI'):
        os.environ.pop(name, None)
    root = Path(__file__).resolve().parents[1]
    sources = [str(root / 'src' / package) for package in ('rim_locator', 'grasp_executor')]
    sys.path[:0] = sources
    os.environ['PYTHONPATH'] = os.pathsep.join([*sources, os.environ.get('PYTHONPATH', '')])
    directory = Path(tempfile.mkdtemp(prefix='place-launch-contract-'))
    os.environ['ROS_LOG_DIR'] = str(directory / 'ros_logs')
    result = {'passed': False, 'domain': domain, 'localhost_only': True,
              'hardware_started': False, 'camera_started': False, 'moveit_server_started': False,
              'observations_published': 0, 'targets_published': 0, 'checks': [], 'commands': {}}
    processes, logs, statuses, parameters = {}, [], {}, {}
    node, initialized = None, False
    supervisor = None
    print('Test logs:', directory, 'ROS_DOMAIN_ID:', domain, flush=True)

    def check(condition, description):
        if not condition:
            raise AssertionError(description)
        result['checks'].append(description)
        print('PASS', description, flush=True)

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rcl_interfaces.msg import ParameterType
        from rcl_interfaces.srv import GetParameters
        from std_msgs.msg import String
        from rim_locator.config import load_config as load_rim_config
        from grasp_executor.config import load_config as load_grasp_config
        from grasp_executor.process_guard import process_start_ticks

        rclpy.init(domain_id=domain)
        initialized = True
        node = Node('place_launch_contract_test_' + uuid.uuid4().hex[:8])
        result['actual_context_domain'] = node.context.get_domain_id()
        check(node.context.get_domain_id() == domain, 'ROS_context_uses_exact_isolated_domain')

        def spin_until(predicate, description, allow_node_failure=False):
            while not predicate():
                if time.monotonic() >= deadline:
                    raise TimeoutError('test_deadline:' + description)
                rclpy.spin_once(node, timeout_sec=min(.05, max(0., deadline - time.monotonic())))
                exited = [name for name, process in processes.items() if process.poll() is not None]
                if exited:
                    raise RuntimeError('launch_exited:' + ','.join(exited))
                failed = {name: status for name, status in statuses.items() if status.get('status') == 'failed'}
                if failed and not allow_node_failure:
                    raise RuntimeError('node_failed:' + json.dumps(failed))

        settle = time.monotonic() + 1.25
        spin_until(lambda: time.monotonic() >= settle, 'initial_discovery')
        others = [(name, namespace) for name, namespace in node.get_node_names_and_namespaces()
                  if name != node.get_name()]
        check(not others, 'chosen_domain_has_no_other_nodes')
        session = uuid.uuid4().hex
        namespace = '/place_launch_contract_' + session
        task_id = 'task_' + session
        observation_id = task_id + '/round_0007'
        source_not_before_ns = node.get_clock().now().nanoseconds - 1_000_000_000
        task_deadline_monotonic_ns = int((started + 30.) * 1e9)
        preview_topic = namespace + '/actual_launch_preview'
        result.update(namespace=namespace, task_id=task_id, observation_id=observation_id,
                      source_not_before_ns=source_not_before_ns,
                      task_deadline_monotonic_ns=task_deadline_monotonic_ns,
                      task_preview_topic=preview_topic)
        rim = replace(load_rim_config(root / 'src/rim_locator/config/place_config.json'),
                      input_topic=namespace + '/object_cloud', output_topic=namespace + '/rim_point',
                      poses_topic=namespace + '/grasp_poses', candidates_topic=namespace + '/grasp_candidates',
                      input_timeout=60., publish_debug=False)
        grasp = replace(load_grasp_config(root / 'src/grasp_executor/config/place_config.json'),
                        input_mode='candidate_groups', input_topic=namespace + '/selected_candidates',
                        task_preview_topic=namespace + '/config_preview_must_be_overridden',
                        plan_only=False, input_timeout=60., publish_debug=False)
        supervisor = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
        supervisor_ticks = process_start_ticks(supervisor.pid)
        configs = {'rim_locator': rim, 'grasp_executor': grasp}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions = []
        ros2 = shutil.which('ros2')
        if ros2 is None:
            raise RuntimeError('ros2_not_on_path:source_ROS_and_workspace_setup')
        for name, config in configs.items():
            path = directory / (name + '_config.json')
            path.write_text(json.dumps(asdict(config), indent=2))
            subscriptions.append(node.create_subscription(String, namespace + '/' + name + '/status',
                lambda message, name=name: statuses.__setitem__(name, json.loads(message.data)), qos))
            command = [ros2, 'launch', name, name + '.launch.py',
                       'config_path:=' + str(path), 'namespace:=' + namespace,
                       'task_id:=' + task_id, 'observation_id:=' + observation_id, 'use_sim_time:=false']
            if name == 'grasp_executor':
                command.extend(['plan_only:=true', f'source_not_before_ns:={source_not_before_ns}',
                                f'task_deadline_monotonic_ns:={task_deadline_monotonic_ns}',
                                'task_preview_topic:=' + preview_topic,
                                f'supervisor_pid:={supervisor.pid}',
                                f'supervisor_start_ticks:={supervisor_ticks}'])
            result['commands'][name] = command
            log = (directory / (name + '.log')).open('x')
            logs.append(log)
            processes[name] = subprocess.Popen(command, cwd=root, env=os.environ.copy(),
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        spin_until(lambda: statuses.get('rim_locator', {}).get('status') == 'waiting_input'
                   and statuses.get('grasp_executor', {}).get('status') == 'waiting_target', 'both_nodes_ready')
        check(all(status.get('task_id') == task_id and status.get('observation_id') == observation_id
                  for status in statuses.values()), 'private_statuses_preserve_exact_task_and_observation')
        check(statuses['grasp_executor']['plan_only'] is True, 'launch_overrides_config_to_plan_only')
        node_names = set(node.get_node_names_and_namespaces())
        check(('rim_locator', namespace) in node_names and ('grasp_executor', namespace) in node_names,
              'both_real_launch_nodes_have_requested_namespace')
        check(('rim_locator', '/') not in node_names and ('grasp_executor', '/') not in node_names,
              'launch_does_not_create_unscoped_pipeline_nodes')

        def read_parameters(name, names):
            client = node.create_client(GetParameters, namespace + '/' + name + '/get_parameters')
            try:
                spin_until(client.service_is_ready, name + '_parameter_service')
                future = client.call_async(GetParameters.Request(names=names))
                spin_until(future.done, name + '_parameter_response')
                response = future.result()
                if response is None or len(response.values) != len(names):
                    raise RuntimeError('parameter_response_missing:' + name)
                values = {}
                for key, value in zip(names, response.values):
                    if value.type == ParameterType.PARAMETER_STRING:
                        values[key] = value.string_value
                    elif value.type == ParameterType.PARAMETER_INTEGER:
                        values[key] = value.integer_value
                    elif value.type == ParameterType.PARAMETER_BOOL:
                        values[key] = value.bool_value
                    else:
                        raise RuntimeError('unexpected_parameter_type:' + key + ':' + str(value.type))
                parameters[name] = values
                return values
            finally:
                node.destroy_client(client)

        common_names = ['task_id', 'observation_id', 'config_path', 'use_sim_time']
        rim_parameters = read_parameters('rim_locator', common_names)
        grasp_parameters = read_parameters('grasp_executor', common_names + [
            'source_not_before_ns', 'task_deadline_monotonic_ns', 'task_preview_topic', 'plan_only', 'supervisor_pid', 'supervisor_start_ticks'])
        for name, values in parameters.items():
            check(values['task_id'] == task_id and values['observation_id'] == observation_id,
                  name + '_parameter_identity_matches_launch')
            check(values['config_path'] == str(directory / (name + '_config.json'))
                  and values['use_sim_time'] is False, name + '_config_and_clock_parameters_match')
        check(type(grasp_parameters['source_not_before_ns']) is int
              and grasp_parameters['source_not_before_ns'] == source_not_before_ns,
              'nanosecond_capture_boundary_survives_launch_as_exact_integer')
        check(type(grasp_parameters['task_deadline_monotonic_ns']) is int
              and grasp_parameters['task_deadline_monotonic_ns'] == task_deadline_monotonic_ns,
              'monotonic_task_deadline_survives_launch_as_exact_integer')
        check(grasp_parameters['plan_only'] is True, 'actual_plan_only_parameter_is_true')
        check(grasp_parameters['task_preview_topic'] == preview_topic,
              'actual_preview_parameter_uses_private_launch_argument')

        def endpoints(topic, publishers, owner):
            method = node.get_publishers_info_by_topic if publishers else node.get_subscriptions_info_by_topic
            return [item for item in method(topic) if item.node_name == owner and item.node_namespace == namespace]

        expected = [
            (rim.input_topic, False, 'rim_locator', 'sensor_msgs/msg/PointCloud2'),
            (rim.candidates_topic, True, 'rim_locator', 'std_msgs/msg/String'),
            (rim.poses_topic, True, 'rim_locator', 'geometry_msgs/msg/PoseArray'),
            (rim.output_topic, True, 'rim_locator', 'geometry_msgs/msg/PointStamped'),
            (grasp.input_topic, False, 'grasp_executor', 'std_msgs/msg/String'),
            (preview_topic, True, 'grasp_executor', 'moveit_msgs/msg/DisplayTrajectory'),
        ]
        spin_until(lambda: all(endpoints(topic, publishers, owner) for topic, publishers, owner, _ in expected),
                   'private_topic_graph_discovery')
        result['endpoints'] = []
        for topic, publishers, owner, topic_type in expected:
            info = endpoints(topic, publishers, owner)
            check(len(info) == 1 and info[0].topic_type == topic_type,
                  owner + ('_publishes_' if publishers else '_subscribes_') + topic.rsplit('/', 1)[1] + '_correct_type')
            result['endpoints'].append({'topic': topic, 'publisher': publishers, 'node_name': owner,
                'namespace': info[0].node_namespace, 'type': info[0].topic_type,
                'durability': int(info[0].qos_profile.durability),
                'reliability': int(info[0].qos_profile.reliability), 'depth': info[0].qos_profile.depth})
            if topic in (rim.input_topic, rim.candidates_topic, rim.poses_topic, grasp.input_topic):
                check(info[0].qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
                      and info[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE,
                      owner + '_' + topic.rsplit('/', 1)[1] + '_uses_reliable_retained_transport')
        check(not endpoints('/display_planned_path', True, 'grasp_executor')
              and not endpoints(grasp.task_preview_topic, True, 'grasp_executor'),
              'preview_publisher_is_not_redirected_to_default_or_config_topic')
        check(not endpoints(rim.poses_topic, False, 'grasp_executor')
              and not endpoints(rim.output_topic, False, 'grasp_executor'),
              'group_mode_does_not_subscribe_legacy_pose_or_point_inputs')
        check(node.count_publishers(rim.input_topic) == node.count_publishers(grasp.input_topic) == 0,
              'test_has_no_cloud_or_target_publishers')
        check(node.count_publishers('/joint_states') == 0, 'no_robot_or_synthetic_joint_publisher_started')
        # A graph lists action clients as well as servers. Check server
        # readiness instead of treating executor's idle client as a server.
        from rclpy.action import ActionClient
        from moveit_msgs.action import ExecuteTrajectory
        from moveit_msgs.srv import GetMotionPlan
        from controller_manager_msgs.srv import SwitchController
        action_probe = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
        planning_probe = node.create_client(GetMotionPlan, '/plan_kinematic_path')
        controller_probe = node.create_client(SwitchController, '/controller_manager/switch_controller')
        try:
            check(not action_probe.server_is_ready() and not planning_probe.service_is_ready()
                  and not controller_probe.service_is_ready(),
                  'no_motion_action_planning_or_controller_service_started')
        finally:
            action_probe.destroy()
            node.destroy_client(planning_probe)
            node.destroy_client(controller_probe)
        check(statuses['rim_locator']['status'] == 'waiting_input'
              and statuses['grasp_executor']['status'] == 'waiting_target',
              'nodes_remain_idle_without_any_observation_or_target')
        check(grasp_parameters['supervisor_pid'] == supervisor.pid
              and grasp_parameters['supervisor_start_ticks'] == supervisor_ticks,
              'supervisor_identity_survives_real_launch')
        supervisor.kill()
        supervisor.wait(timeout=2.)
        spin_until(lambda: statuses.get('grasp_executor', {}).get('reason') == 'supervisor_lost',
                   'idle_executor_detects_supervisor_death', allow_node_failure=True)
        check(statuses['grasp_executor']['status'] == 'failed'
              and statuses['grasp_executor'].get('command_attempted') is False,
              'supervisor_death_fails_idle_executor_without_any_motion')
        result['passed'] = True
    except BaseException as exc:
        result['error'] = f'{type(exc).__name__}:{exc}'
        print('FAIL', result['error'], flush=True)
    finally:
        if supervisor is not None and supervisor.poll() is None:
            supervisor.terminate()
            supervisor.wait(timeout=2.)
        cleanup = stop_groups(processes)
        result['cleanup'] = cleanup
        if cleanup['errors'] or not cleanup['all_groups_stopped']:
            result['passed'] = False
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()
        for log in logs:
            log.close()
        result.update(statuses=statuses, parameters=parameters, elapsed_seconds=time.monotonic() - started)
        (directory / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print('Result:', directory / 'result.json', flush=True)
    return 0 if result['passed'] else 1


def main():
    from isolated_ros import IsolatedRosTestLock
    with IsolatedRosTestLock():
        return _run_locked()


if __name__ == '__main__':
    raise SystemExit(main())

"""Observation barriers and launch context without ROS nodes or sensor devices."""
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header

from test_retry import FakeNode, result
from yolo_vision import main as vision
from yolo_vision.sensor_input import SerializedSensor


BARRIER = 99_000_000_000


def sensor(kind, stamp_ns):
    return kind(header=Header(frame_id='camera', stamp=Time(
        sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)),
        height=3, width=4)


@pytest.mark.parametrize('old_input', ['rgb', 'depth', 'both'])
@pytest.mark.parametrize('at_barrier', [False, True])
def test_old_pair_never_decodes_or_locks_observation(monkeypatch, old_input, at_barrier):
    node = FakeNode(monkeypatch, [result()])
    node.source_not_before_ns = BARRIER
    old_stamp = BARRIER if at_barrier else BARRIER - 10_000_000
    new_stamp = BARRIER + 10_000_000
    rgb = sensor(Image, old_stamp if old_input in ('rgb', 'both') else new_stamp)
    depth = sensor(PointCloud2, old_stamp if old_input in ('depth', 'both') else new_stamp)
    frames = [SerializedSensor(serialize_message(message), type(message)) for message in (rgb, depth)]
    with monkeypatch.context() as block_decode:
        block_decode.setattr(SerializedSensor, 'decode', lambda self: pytest.fail('old image decoded'))
        node.on_pair(*frames)
    assert node.state == 'ready' and not node.reports
    assert node.calls == 0 and not node.messages and node.wait_started == 0.
    node.on_pair(sensor(Image, new_stamp), sensor(PointCloud2, new_stamp))
    assert node.state == 'success' and node.calls == 1
    assert node.messages[0].header.stamp == sensor(Image, new_stamp).header.stamp


def test_repeated_old_pairs_cannot_extend_observation_timeout(monkeypatch):
    node = FakeNode(monkeypatch, [])
    node.source_not_before_ns = BARRIER
    for elapsed in (1., 5., 9.9):
        node.clock = elapsed
        node.on_pair(sensor(Image, BARRIER), sensor(PointCloud2, BARRIER))
        assert node.state == 'ready' and node.wait_started == 0.
    node.clock = 10.
    node.on_pair(sensor(Image, BARRIER), sensor(PointCloud2, BARRIER))
    assert node.state == 'failed' and node.reports[-1]['reason'] == 'input_timeout'
    assert not node.calls and not node.messages


@pytest.mark.parametrize('reason, now, stamp', [
    ('source_clock_before_observation_barrier', BARRIER - 1, BARRIER + 1),
    ('source_timestamp_stale_or_future', BARRIER + 20_000_000_000, BARRIER + 1),
    ('source_timestamp_stale_or_future', BARRIER, BARRIER + 1_000_000_000),
])
def test_barrier_does_not_replace_clock_and_age_guards(monkeypatch, reason, now, stamp):
    node = FakeNode(monkeypatch, [])
    node.source_not_before_ns = BARRIER
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=now))
    node.on_pair(sensor(Image, stamp), sensor(PointCloud2, stamp))
    assert node.state == 'failed' and node.reports[-1]['reason'] == reason
    assert node.calls == 0 and not node.messages


@pytest.mark.parametrize('info_delta, accepted', [(-20_000_000, True), (-100_000_000, False)])
def test_camera_info_uses_sync_window_not_image_barrier(monkeypatch, info_delta, accepted):
    node = FakeNode(monkeypatch, [result()])
    node.source_not_before_ns = BARRIER
    node.config = replace(node.config, depth_image_topic='/depth')
    stamp = BARRIER + 10_000_000
    info = sensor(CameraInfo, stamp + info_delta)
    monkeypatch.setattr(vision, 'read_aligned_depth_xyz', lambda depth, calibration: node.xyz)
    node.on_pair(sensor(Image, stamp), sensor(Image, stamp), info)
    if accepted:
        assert node.state == 'success' and node.calls == 1
        assert node.messages[0].header.stamp == sensor(Image, stamp).header.stamp
    else:
        assert node.state == 'failed'
        assert node.reports[-1]['reason'] == 'depth_camera_info_not_synchronized'
        assert node.calls == 0 and not node.messages


@pytest.mark.parametrize('status', ['ready', 'processing', 'success', 'no_targets', 'failed'])
def test_every_status_preserves_runtime_observation_identity(status):
    messages, logs = [], []
    node = NS(task_id='task', observation_id='task/round_0002', source_not_before_ns=BARRIER,
              status_pub=NS(publish=messages.append), get_logger=lambda: NS(info=logs.append))
    vision.VisionNode.report(node, status, source_stamp_ns=BARRIER + 1)
    value = json.loads(messages[0].data)
    assert value['status'] == node.state == status
    assert value['task_id'] == 'task' and value['observation_id'] == 'task/round_0002'
    assert value['source_not_before_ns'] == BARRIER
    assert value['source_stamp_ns'] == BARRIER + 1 and logs == [messages[0].data]


@pytest.mark.parametrize('parameters, reason', [
    ({'task_id': 'task'}, 'incomplete_observation_identity'),
    ({'observation_id': 'round'}, 'incomplete_observation_identity'),
    ({'task_id': ' '}, 'invalid_observation_parameter:task_id'),
    ({'source_not_before_ns': -1}, 'invalid_observation_parameter:source_not_before_ns'),
    ({'source_not_before_ns': True}, 'invalid_observation_parameter:source_not_before_ns'),
])
def test_bad_runtime_parameters_fail_before_model_or_sensors(monkeypatch, parameters, reason):
    messages = []
    monkeypatch.setattr(vision.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(vision.Node, 'create_publisher', lambda *a, **kw: NS(publish=messages.append))
    monkeypatch.setattr(vision.Node, 'declare_parameter', lambda self, name, default: NS(
        value=parameters.get(name, default)))
    monkeypatch.setattr(vision.Node, 'get_logger', lambda self: NS(info=lambda payload: None))
    monkeypatch.setattr(vision, 'get_package_share_directory', lambda name: pytest.fail('invalid context reached model setup'))
    node = vision.VisionNode()
    assert node.state == 'failed'
    value = json.loads(messages[0].data)
    assert value['failed_step'] == 'initializing' and value['reason'] == reason


def test_launch_forwards_namespace_and_typed_runtime_context(monkeypatch):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, GroupAction
    from launch.utilities import perform_substitutions
    from launch_ros.utilities import evaluate_parameters, normalize_parameters
    import launch_ros.actions

    captured = {}
    def fake_node(**kwargs):
        captured.update(kwargs)
        return GroupAction(actions=[])
    monkeypatch.setattr(launch_ros.actions, 'Node', fake_node)
    path = Path(__file__).parents[1] / 'launch/yolo_vision.launch.py'
    spec = importlib.util.spec_from_file_location('vision_launch_context_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    declarations = {entry.name: entry for entry in description.entities if isinstance(entry, DeclareLaunchArgument)}
    context = LaunchContext()
    assert perform_substitutions(context, declarations['namespace'].default_value) == ''
    assert perform_substitutions(context, declarations['task_id'].default_value) == ''
    assert perform_substitutions(context, declarations['observation_id'].default_value) == ''
    assert perform_substitutions(context, declarations['source_not_before_ns'].default_value) == '0'
    context.launch_configurations.update(namespace='/task/round_0002', task_id='task',
        observation_id='task/round_0002', source_not_before_ns=str(BARRIER),
        use_sim_time='false', config_path='/tmp/vision_context_config.json')
    assert captured['namespace'].perform(context) == '/task/round_0002'
    values = evaluate_parameters(context, normalize_parameters(captured['parameters']))[0]
    assert values['task_id'] == 'task' and values['observation_id'] == 'task/round_0002'
    assert values['source_not_before_ns'] == BARRIER and type(values['source_not_before_ns']) is int
    assert values['use_sim_time'] is False

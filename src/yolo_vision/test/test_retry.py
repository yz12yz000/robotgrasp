"""Real callback code with fake clock/bridge/model/publishers; no ROS graph."""
from types import SimpleNamespace as NS
import numpy as np
import pytest
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from yolo_vision.config import VisionConfig
from yolo_vision import main as vision


class FakeNode:
    on_pair = vision.VisionNode.on_pair
    check_timeout = vision.VisionNode.check_timeout
    deadline_expired = vision.VisionNode.deadline_expired
    camera_details = vision.VisionNode.camera_details

    def __init__(self, monkeypatch, outcomes):
        self.config = VisionConfig(expected_width=4, expected_height=3,
                                   min_points=2, input_timeout=10., publish_debug=False)
        self.state, self.wait_started = 'ready', 0.
        self.clock, self.calls, self.messages, self.reports = 0., 0, [], []
        self.outcomes = iter(outcomes)
        self.bridge = NS(imgmsg_to_cv2=lambda *a, **k: np.zeros((3, 4, 3), dtype=np.uint8))
        self.segmenter = NS(predict_all=lambda bgr: self.predict())
        self.publisher = NS(publish=self.messages.append)
        self.xyz = np.ones((3, 4, 3))
        monkeypatch.setattr(vision.time, 'monotonic', lambda: self.clock)
        monkeypatch.setattr(vision, 'read_xyz', lambda cloud: self.xyz)

    def report(self, state, **details):
        self.state = state
        self.reports.append({'state': state, **details})

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=100_000_000_000))

    def predict(self):
        self.calls += 1
        result = next(self.outcomes)
        if callable(result):
            result = result()
        if isinstance(result, Exception):
            raise result
        return result


def result(mask=None):
    return [NS(mask=np.ones((3, 4), dtype=bool) if mask is None else mask, confidence=.9)]


def pair():
    header = Header(frame_id='camera', stamp=Time(sec=99, nanosec=42))
    return [NS(header=header, height=3, width=4) for _ in range(2)]


@pytest.mark.parametrize('count', [0, 1, 2, 3])
def test_observed_count_freezes_and_preserves_stamp(monkeypatch, count):
    targets=[]
    for index in range(count):
        mask=np.zeros((3,4),dtype=bool)
        mask[index,:]=True
        targets += result(mask)
    node = FakeNode(monkeypatch, [targets])
    frames = pair()
    node.on_pair(*frames)
    assert node.state == ('success' if count else 'no_targets')
    assert node.reports[-1]['detected_count'] == node.reports[-1]['object_count'] == count
    assert len(node.messages) == (1 if count else 0)
    if count:
        from rim_locator.pointcloud import read_xyz_object_ids
        assert node.messages[0].header.stamp == frames[1].header.stamp
        _, ids=read_xyz_object_ids(node.messages[0])
        assert set(ids)==set(range(count))
    node.on_pair(*frames)
    assert node.calls == 1


@pytest.mark.parametrize('expiration', ['timer', 'new_frame', 'after_empty', 'after_success'])
def test_total_deadline(monkeypatch, expiration):
    node = FakeNode(monkeypatch, [result()])
    if expiration.startswith('after'):
        def late():
            node.clock = 10.
            return result() if expiration == 'after_success' else []
        node.outcomes = iter([late])
        node.on_pair(*pair())
    else:
        node.clock = 10.
        node.check_timeout() if expiration == 'timer' else node.on_pair(*pair())
        assert node.calls == 0
    assert node.state == 'failed' and node.reports[-1]['reason'] == 'input_timeout'
    assert not node.messages


def test_no_input_deadline(monkeypatch):
    node = FakeNode(monkeypatch, [])
    node.clock = 10.
    node.check_timeout()
    assert node.state == 'failed' and not node.calls


def test_detected_but_no_valid_depth_is_failure_not_zero_objects(monkeypatch):
    node = FakeNode(monkeypatch, [result()])
    node.xyz[:] = 0.
    node.on_pair(*pair())
    assert node.state == 'failed'
    assert node.reports[-1]['reason'] == 'no_valid_depth_for_detected_targets'
    assert not node.messages


@pytest.mark.parametrize('bad, reason', [
    (np.ones((3, 4), dtype=int), 'mask_must_be_boolean'),
    (np.ones((1, 4), dtype=bool), 'dimension_mismatch'),
    (ValueError('model_failed'), 'model_failed')])
def test_format_or_model_errors_are_terminal(monkeypatch, bad, reason):
    node = FakeNode(monkeypatch, [bad if isinstance(bad, Exception) else result(bad)])
    node.on_pair(*pair())
    assert node.state == 'failed' and node.reports[-1]['reason'] == reason
    assert not node.messages


def test_partial_insufficient_depth_keeps_other_instance(monkeypatch):
    mask = np.zeros((3, 4), dtype=bool)
    mask[:, :2] = True
    node = FakeNode(monkeypatch, [result(mask) + result(~mask)])
    node.xyz[:, :2] = 0.
    node.on_pair(*pair())
    assert node.state == 'success'
    assert node.reports[-1]['objects'][0]['object_id'] == 1
    assert node.reports[-1]['rejected'] == [{'object_id': 0, 'reason': 'insufficient_pointcloud',
                                           'mask_pixels': 6, 'valid_depth_points': 0,
                                           'valid_depth_ratio': 0.0}]


@pytest.mark.parametrize('fault, reason', [
    ('empty', 'empty_frame_id'), ('frame', 'depth_not_registered_to_rgb_frame'),
    ('sync', 'rgb_depth_not_synchronized'), ('stale', 'source_timestamp_stale_or_future'),
    ('shape', 'dimension_mismatch')])
def test_existing_input_guards_still_fail(monkeypatch, fault, reason):
    node = FakeNode(monkeypatch, [result()])
    image, cloud = pair()
    image.header = Header(frame_id='camera', stamp=Time(sec=99, nanosec=42))
    if fault == 'empty': cloud.header.frame_id = ''
    if fault == 'frame': image.header.frame_id = 'other'
    if fault == 'sync': image.header.stamp.sec = 98
    if fault == 'stale': image.header.stamp.sec = cloud.header.stamp.sec = 1
    if fault == 'shape': image.width = 3
    node.on_pair(image, cloud)
    assert node.state == 'failed' and node.reports[-1]['reason'] == reason
    assert node.calls == 0 and not node.messages


@pytest.mark.parametrize('during_debug', [False, True])
def test_deadline_during_output_processing_never_publishes(monkeypatch, during_debug):
    from dataclasses import replace
    node = FakeNode(monkeypatch, [result()])
    debug_messages = []
    if during_debug:
        node.config = replace(node.config, publish_debug=True)
        def convert(*args, **kwargs):
            node.clock = 10.
            return NS(header=None)
        node.bridge.cv2_to_imgmsg = convert
        node.mask_pub = node.overlay_pub = NS(publish=debug_messages.append)
    else:
        def read(cloud):
            node.clock = 10.
            return node.xyz
        monkeypatch.setattr(vision, 'read_xyz', read)
    node.on_pair(*pair())
    assert node.state == 'failed' and node.reports[-1]['reason'] == 'input_timeout'
    assert not node.messages and not debug_messages


def test_bad_second_mask_does_not_publish_partial_batch(monkeypatch):
    node = FakeNode(monkeypatch, [result() + result(np.ones((3, 4), dtype=int))])
    node.on_pair(*pair())
    assert node.state == 'failed' and node.reports[-1]['reason'] == 'mask_must_be_boolean'
    assert not node.messages

from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
from cv_bridge import CvBridge

from yolo_vision.diagnostics import detection_debug
from test_retry import FakeNode, result, pair


def test_debug_overlap_ids_match_cloud_confidence_policy():
    bgr = np.full((60, 80, 3), 80, dtype=np.uint8)
    first = np.zeros((60, 80), bool)
    second = first.copy()
    first[20:50, 10:45] = True
    second[30:55, 30:65] = True
    xyz = np.ones((60, 80, 3))
    xyz[40, 40] = 0
    mask, image = detection_debug(bgr, [NS(mask=first, confidence=.9), NS(mask=second, confidence=.8)], xyz)
    assert mask[40, 40] == 1  # First/highest confidence owns overlapping pixels.
    assert mask[50, 50] == 2
    assert np.array_equal(image[40, 40], [0, 0, 255])
    assert not np.shares_memory(image, bgr)
    assert np.all(bgr == 80)


def test_failed_depth_still_publishes_detection_debug_and_counts(monkeypatch):
    node = FakeNode(monkeypatch, [result()])
    node.config = replace(node.config, publish_debug=True)
    node.xyz[:] = 0.
    masks, overlays = [], []
    node.bridge.cv2_to_imgmsg = CvBridge().cv2_to_imgmsg
    node.mask_pub = NS(publish=masks.append)
    node.overlay_pub = NS(publish=overlays.append)
    node.on_pair(*pair())
    assert node.state == 'failed' and not node.messages
    assert len(masks) == len(overlays) == 1
    assert masks[0].header.stamp == pair()[0].header.stamp
    report = node.reports[-1]
    assert report['detected_count'] == 1 and report['object_count'] == 0
    assert report['rejected'][0]['valid_depth_ratio'] == 0.

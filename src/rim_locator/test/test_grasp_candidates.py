"""Multi-contact output preserves observed geometry, ordering and object groups."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from rim_locator import rim_detection as geometry
from rim_locator.candidate_protocol import decode_candidate_batch
from rim_locator.config import RimConfig, load_config
from test_multi_pose import wall_cloud, bowl_points


def test_every_candidate_uses_supported_wall_and_first_matches_single_api():
    points = wall_cloud(tilt_deg=20.)
    config = RimConfig(orientation_mode='wall_normal', max_tool_tilt_deg=0.)
    first = geometry.locate_grasp_pose(points, config)
    candidates = geometry.locate_grasp_candidates(points, config)
    assert len(candidates) == config.max_grasp_candidates
    np.testing.assert_array_equal(candidates[0].point, first.point)
    np.testing.assert_array_equal(candidates[0].orientation, first.orientation)
    assert candidates[0].diagnostics == first.diagnostics
    for index, result in enumerate(candidates):
        assert result.diagnostics['wall_count'] >= config.min_wall_points
        assert result.diagnostics['wall_span'] >= config.min_wall_span
        assert result.diagnostics['wall_residual'] <= config.max_wall_residual
        assert result.diagnostics['normal_change_deg'] <= config.max_normal_change_deg
        assert result.diagnostics['tool_tilt_deg'] == 0.
        assert np.linalg.norm(result.orientation) == pytest.approx(1.)
        assert all(np.linalg.norm(result.point - previous.point) >= config.candidate_min_separation
                   for previous in candidates[:index])
    one = geometry.locate_grasp_candidates(points, replace(config, max_grasp_candidates=1))
    assert len(one) == 1
    np.testing.assert_array_equal(one[0].point, first.point)


def test_stable_dedup_keeps_search_order_and_ignores_quality_ranking(monkeypatch):
    points = wall_cloud()
    config = RimConfig(orientation_mode='wall_normal')
    first = geometry.locate_grasp_pose(points, config)
    # The first contact is intentionally farther away than subsequent contacts.
    positions = [np.array([.5, .1, .4]), np.array([.501, .1, .4]),
                 np.array([.3, .1, .4]), np.array([.304, .1, .4])]
    def supported(*args):
        for index, point in enumerate(positions):
            yield point, first.normal, first.local_points, first.wall_points, {'wall_count': 20 + index * 100}
    monkeypatch.setattr(geometry, '_supported_walls', supported)
    candidates = geometry.locate_grasp_candidates(points, config)
    assert len(candidates) == 3
    for actual, expected in zip(candidates, (positions[0], positions[2], positions[3])):
        np.testing.assert_array_equal(actual.point, expected)
    assert [item.diagnostics['wall_count'] for item in candidates] == [20, 220, 320]


def test_missing_wall_returns_no_invented_candidates():
    with pytest.raises(geometry.PoseNotFound, match='insufficient_wall_points'):
        geometry.locate_grasp_candidates(bowl_points(), RimConfig(orientation_mode='wall_normal'))


@pytest.mark.parametrize('fixture', ['two_bowls_frame01.npz', 'two_bowls_frame02.npz'])
def test_recorded_clouds_offer_distinct_supported_candidates(fixture):
    data = np.load(Path(__file__).parent / 'data' / fixture)
    config = RimConfig(orientation_mode='wall_normal', max_tool_tilt_deg=0.)
    for key in ('points_0', 'points_1'):
        candidates = geometry.locate_grasp_candidates(data[key], config)
        assert 1 < len(candidates) <= config.max_grasp_candidates
        assert all(result.diagnostics['wall_count'] >= 20 for result in candidates)
        expected = geometry.locate_grasp_pose(data[key], config)
        np.testing.assert_array_equal(candidates[0].point, expected.point)


def node_for_finish(config):
    outputs = NS(groups=[], poses=[], points=[], reports=[])
    node = NS(config=config, task_id='task', observation_id='task/round_0001',
              candidates_publisher=NS(publish=outputs.groups.append),
              poses_publisher=NS(publish=outputs.poses.append),
              publisher=NS(publish=outputs.points.append),
              get_logger=lambda: NS(warning=lambda message: None),
              report=lambda state, **details: outputs.reports.append((state, details)))
    return node, outputs


def test_ros_grouping_preserves_legacy_one_pose_per_bowl_and_stamp():
    from builtin_interfaces.msg import Time
    from rim_locator.main import RimNode
    node, outputs = node_for_finish(RimConfig(orientation_mode='wall_normal', publish_debug=False))
    far, near = wall_cloud((.6, .1)), wall_cloud((.35, .1))
    points = np.r_[far, near]
    ids = np.r_[np.full(len(far), 9), np.full(len(near), 7)]
    stamp = Time(sec=123, nanosec=456)
    RimNode.finish(node, points, ids, stamp)
    batch = decode_candidate_batch(outputs.groups[0].data, expected_task_id='task',
                                   expected_observation_id='task/round_0001')
    assert batch['source_stamp_ns'] == 123_000_000_456
    assert [group['object_id'] for group in batch['objects']] == [7, 9]
    assert len(batch['objects']) == len(outputs.poses[0].poses) == 2
    assert all(len(group['candidates']) > 1 for group in batch['objects'])
    assert outputs.poses[0].header.stamp == stamp
    for pose, group in zip(outputs.poses[0].poses, batch['objects']):
        assert [pose.position.x, pose.position.y, pose.position.z] == group['candidates'][0]['position']
    assert outputs.reports[0][1]['object_count'] == 2
    assert all(item['grasp_candidate_count'] == 8 for item in outputs.reports[0][1]['objects'])


def test_ros_all_rejected_still_publishes_geometry_outcome_with_context():
    from builtin_interfaces.msg import Time
    from rim_locator.main import RimNode
    node, outputs = node_for_finish(RimConfig(orientation_mode='wall_normal', publish_debug=False))
    points = bowl_points()
    RimNode.finish(node, points, np.full(len(points), 4), Time(sec=222))
    batch = decode_candidate_batch(outputs.groups[0].data)
    assert batch['objects'] == [] and batch['rejected'][0]['object_id'] == 4
    assert batch['rejected'][0]['geometry']['searched_contacts'] > 1
    assert batch['source_stamp_ns'] == 222_000_000_000
    assert batch['task_id'] == 'task' and batch['observation_id'] == 'task/round_0001'
    assert outputs.poses == outputs.points == []
    assert outputs.reports[0][0] == 'failed'
    assert outputs.reports[0][1]['source_stamp_ns'] == batch['source_stamp_ns']


@pytest.mark.parametrize('values', [dict(max_grasp_candidates=value) for value in (0, 33, True, 1.5)] +
                         [dict(candidate_min_separation=value) for value in (0., -1., True, float('nan'))])
def test_candidate_limits_are_validated(values):
    with pytest.raises(ValueError, match='invalid_config'):
        RimConfig(**values)


def test_shipped_configs_use_bounded_candidates_and_explicit_tool_frame():
    for name in ('config.json', 'place_config.json'):
        config = load_config(Path(__file__).parents[1] / 'config' / name)
        assert config.max_grasp_candidates == 8
        assert config.candidate_min_separation == .004
        assert config.tool_frame == 'grasp_center'
        assert config.candidates_topic == '/rim_locator/grasp_candidates'

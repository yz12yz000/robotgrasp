"""Disconnected contamination regression without any point-cloud library."""
from dataclasses import replace
import math
import time
import numpy as np
import pytest
from rim_locator.config import RimConfig
from rim_locator.rim_detection import filter_component, locate_grasp_pose, locate_rim
from test_multi_pose import wall_cloud, bowl_points


def contamination():
    rng = np.random.default_rng(123)
    return np.array([.20, .05, .47]) + rng.normal(0, .001, (80, 3))


def test_9600_wall_points_and_80_high_near_outliers():
    clean = wall_cloud()
    assert len(clean) == 9600
    polluted = np.concatenate([clean, contamination()])
    config = RimConfig(orientation_mode='wall_normal')
    expected = locate_grasp_pose(clean, config)
    actual = locate_grasp_pose(polluted, config)
    assert np.linalg.norm(actual.point-expected.point) <= .002
    angle = math.degrees(math.acos(np.clip(np.dot(actual.normal, expected.normal), -1., 1.)))
    assert angle <= 2.
    np.testing.assert_array_equal(filter_component(polluted, config), clean)
    # The supported-contact search can now recover even without the optional
    # component filter. It must recover the bowl, never the high outlier patch.
    recovered = locate_grasp_pose(polluted, replace(config, filter_disconnected_points=False))
    assert np.linalg.norm(recovered.point-expected.point) <= .002
    assert math.degrees(math.acos(np.clip(recovered.normal @ expected.normal, -1., 1.))) <= 2.


@pytest.mark.parametrize('wall_angle', [0., 20.])
def test_clean_input_no_coordinate_or_pose_drift(wall_angle):
    points = wall_cloud(tilt_deg=wall_angle)
    config = RimConfig(orientation_mode='wall_normal')
    np.testing.assert_array_equal(filter_component(points, config), points)
    for locate in (locate_rim, locate_grasp_pose):
        old = locate(points, replace(config, filter_disconnected_points=False))
        new = locate(points, config)
        for name in ('point', 'candidates', 'local_points'):
            np.testing.assert_array_equal(getattr(old, name), getattr(new, name))
        if locate is locate_grasp_pose:
            np.testing.assert_array_equal(old.orientation, new.orientation)


def test_tilted_rim_preserves_real_high_edge():
    points = bowl_points(tilt=math.tan(math.radians(20)))
    config = RimConfig(near_side_ratio=.25)
    np.testing.assert_array_equal(filter_component(points, config), points)
    before = locate_grasp_pose(points, replace(config, filter_disconnected_points=False))
    after = locate_grasp_pose(np.concatenate([points, contamination() + [0, 0, .2]]), config)
    np.testing.assert_array_equal(after.point, before.point)
    assert after.candidates[:, 2].max() == points[:, 2].max()


def test_nonuniform_noisy_sampling_and_small_random_holes():
    rng = np.random.default_rng(20260923)
    points = wall_cloud(tilt_deg=20.)
    points = points[rng.random(len(points)) > .12]
    points += rng.normal(0, .00015, points.shape)
    points = np.concatenate([points, points[points[:, 0] > .4]])
    config = RimConfig(orientation_mode='wall_normal')
    np.testing.assert_array_equal(filter_component(points, config), points)
    assert abs(math.degrees(math.asin(abs(locate_grasp_pose(points, config).normal[2])))-20) < 2


def test_components_are_weighted_by_points_not_voxels():
    dense = np.repeat([[.1, .1, .1]], 100, axis=0)
    sparse = np.c_[np.arange(40)*.006 + 1., np.zeros(40), np.zeros(40)]
    np.testing.assert_array_equal(filter_component(np.r_[dense, sparse], RimConfig()), dense)


@pytest.mark.parametrize('points', [np.r_[wall_cloud(), wall_cloud((.8,.8))],
    np.c_[np.arange(40)*.1, np.zeros(40), np.zeros(40)]])
def test_ambiguous_or_fragmented_cloud_is_rejected(points):
    with pytest.raises(ValueError, match='pose_not_found:ambiguous_components'):
        locate_grasp_pose(points, RimConfig())


def test_nonfinite_and_small_clouds():
    points = wall_cloud()
    np.testing.assert_array_equal(filter_component(np.r_[points, [[np.nan, 0, 0], [0, np.inf, 0]]], RimConfig()), points)
    for points, error in [(np.empty((0,3)), 'empty_cloud'), (np.zeros((5,3)), 'insufficient_points'), (np.zeros(3), 'invalid_points_shape')]:
        with pytest.raises(ValueError, match=error):
            filter_component(points, RimConfig())


@pytest.mark.parametrize('values', [dict(component_voxel_size=v) for v in (0, -1, float('inf'), float('nan'), True, '0.008')]
    + [dict(min_component_ratio=v) for v in (0.5, 1.1, float('nan'), False, '0.6')]
    + [dict(filter_disconnected_points=v) for v in (0, 'true', None)])
def test_filter_config_validation(values):
    with pytest.raises(ValueError, match='invalid_config'):
        RimConfig(**values)


def test_large_cloud_linear_storage(capsys):
    points = np.tile(wall_cloud(), (32, 1))
    occupied = len(np.unique(np.floor(points/.008), axis=0))
    started = time.perf_counter()
    kept = filter_component(points, RimConfig())
    elapsed = time.perf_counter()-started
    np.testing.assert_array_equal(kept, points)
    with capsys.disabled():
        print(f'\ncomponent benchmark: points={len(points)}, voxels={occupied}, seconds={elapsed:.3f}')


def test_object_ids_stamp_and_good_instance_survive_fragmented_instance():
    from types import SimpleNamespace as NS
    from builtin_interfaces.msg import Time
    from rim_locator.main import RimNode
    stamp = Time(sec=321, nanosec=987)
    good = np.r_[wall_cloud(), contamination()]
    bad = np.r_[wall_cloud((.8,.8)), wall_cloud((1.2,1.2))]
    points = np.r_[good, bad]
    ids = np.r_[np.full(len(good), 7), np.full(len(bad), 9)]
    poses, reports = [], []
    node = NS(config=RimConfig(orientation_mode='wall_normal', publish_debug=False),
              poses_publisher=NS(publish=poses.append), publisher=NS(publish=lambda m: None),
              report=lambda state, **kw: reports.append(kw))
    RimNode.finish(node, points, ids, stamp)
    assert poses[0].header.stamp == stamp and len(poses[0].poses) == 1
    assert reports[0]['ordered_object_ids'] == [7]
    assert reports[0]['rejected'] == [{'object_id': 9, 'reason': 'pose_not_found:ambiguous_components'}]

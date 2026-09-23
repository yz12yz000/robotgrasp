"""Supported wall search, including recorded depth rather than ideal circles only."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from rim_locator.config import RimConfig
from rim_locator import rim_detection as geometry
from test_multi_pose import wall_cloud, bowl_points


@pytest.mark.parametrize('frame', ['frame01', 'frame02'])
def test_recorded_two_bowls_both_have_supported_contacts(frame):
    data = np.load(Path(__file__).parent / 'data' / f'two_bowls_{frame}.npz')
    config = RimConfig(orientation_mode='wall_normal')
    distances = []
    for index in (0, 1):
        points = data[f'points_{index}']
        result = geometry.locate_grasp_pose(points, config)
        distances.append(np.hypot(*result.point[:2]))
        evidence = result.diagnostics
        assert evidence['searched_contacts'] > 1  # The old sole contact was invalid.
        assert evidence['wall_count'] >= 20
        assert evidence['corroborating_count'] > evidence['wall_count']
        assert evidence['normal_change_deg'] <= 12
        assert evidence['wall_tilt_deg'] <= 40
        assert evidence['wall_residual'] <= .0025
        assert evidence['wall_span'] >= .006
        assert evidence['anchor_plane_distance'] <= .006
        assert np.min(np.linalg.norm(points-result.point, axis=1)) < .003
        assert result.point[2] >= np.percentile(points[:, 2], 98)-config.max_height_band
        assert np.linalg.norm(result.orientation) == pytest.approx(1.)
        shuffled = geometry.locate_grasp_pose(points[np.random.default_rng(37).permutation(len(points))], config)
        np.testing.assert_allclose(shuffled.point, result.point, atol=1e-12)
        np.testing.assert_allclose(shuffled.normal, result.normal, atol=1e-10)
    assert distances[0] < distances[1]


def test_vertical_rim_pinch_keeps_measured_contact_and_horizontal_closing_axis():
    from grasp_executor.cartesian import rotate
    points=wall_cloud(tilt_deg=20.)
    config=RimConfig(orientation_mode='wall_normal')
    measured=geometry.locate_grasp_pose(points,config)
    vertical=geometry.locate_grasp_pose(points,replace(config,max_tool_tilt_deg=0.))
    np.testing.assert_array_equal(vertical.point,measured.point)
    np.testing.assert_array_equal(vertical.wall_points,measured.wall_points)
    np.testing.assert_allclose(rotate(vertical.orientation,(0.,0.,1.)),(0.,0.,-1.),atol=1e-12)
    np.testing.assert_allclose(vertical.normal[:2],measured.normal[:2]/np.linalg.norm(measured.normal[:2]),atol=1e-12)
    assert vertical.diagnostics['wall_tilt_deg'] == pytest.approx(20., abs=1.)
    assert vertical.diagnostics['tool_tilt_deg'] == 0
    with pytest.raises(ValueError,match='insufficient_wall_points'):
        geometry.locate_grasp_pose(bowl_points(),replace(config,max_tool_tilt_deg=0.))


def test_recorded_white_bowl_depth_tail_does_not_move_contact_above_rim():
    config = RimConfig(orientation_mode='wall_normal', max_tool_tilt_deg=0.)
    contacts = []
    for number in (1, 2):
        data = np.load(Path(__file__).parent / 'data' / f'white_green_depth_tail_{number}.npz')
        points = data['points_1']
        assert points[:, 2].max() > .055  # Both observed clouds contain the streak.
        white = geometry.locate_grasp_pose(points, config)
        green = geometry.locate_grasp_pose(data['points_0'], config)
        assert .015 < white.point[2] < .030
        assert white.diagnostics['wall_count'] >= 50
        assert white.diagnostics['normal_change_deg'] < 5.
        assert np.hypot(*green.point[:2]) < np.hypot(*white.point[:2])
        contacts.append(white.point)
    # Before bounding the upper candidate tail these two frames differed by
    # 3.6 cm in contact height; projection onto the image did not reveal it.
    assert np.linalg.norm(contacts[0]-contacts[1]) < .003


@pytest.mark.parametrize('value',[-1,90,True,float('nan'),'0'])
def test_tool_tilt_limit_validation(value):
    with pytest.raises(ValueError,match='max_tool_tilt_deg'):
        RimConfig(max_tool_tilt_deg=value)


def test_missing_nearest_wall_searches_another_observed_rim():
    points = wall_cloud()
    config = RimConfig(orientation_mode='wall_normal')
    nearest = geometry.locate_grasp_pose(points, config)
    # Remove only the wall below the original contact, leaving that rim visible.
    near = np.linalg.norm(points[:, :2]-nearest.point[:2], axis=1) < .045
    points = points[~(near & (points[:, 2] < .394))]
    result = geometry.locate_grasp_pose(points, config)
    assert result.diagnostics['searched_contacts'] > 1
    assert np.linalg.norm(result.point-nearest.point) > .02
    assert result.point[2] > .395
    assert result.diagnostics['wall_count'] >= config.min_wall_points
    assert abs(result.normal[2]) < .02


@pytest.mark.parametrize('points', [bowl_points(), wall_cloud(tilt_deg=65.),
    np.c_[np.linspace(.2, .4, 100), np.full(100, .2), np.full(100, .4)]])
def test_missing_or_unsafe_wall_never_falls_back_to_vertical(points):
    with pytest.raises(ValueError, match='pose_not_found'):
        geometry.locate_grasp_pose(points, RimConfig(orientation_mode='wall_normal'))


def test_search_reports_every_failed_neighborhood_and_is_bounded():
    config = RimConfig(orientation_mode='wall_normal', rim_search_max_candidates=4)
    with pytest.raises(geometry.PoseNotFound, match='insufficient_wall_points') as caught:
        geometry.locate_grasp_pose(bowl_points(), config)
    evidence = caught.value.diagnostics
    assert 1 < evidence['searched_contacts'] <= 5
    assert sum(evidence['rejected_neighborhoods'].values()) == 3*evidence['searched_contacts']


def test_two_scales_must_agree_and_have_additional_support(monkeypatch):
    config = RimConfig(orientation_mode='wall_normal')
    points = wall_cloud()
    result = geometry.locate_rim(points, config)
    monkeypatch.setattr(geometry, '_rim_contacts', lambda *args: [(result.point, result.local_points)])

    def inconsistent(local, point, outward, radius, config):
        angle = {0.5: 0., 0.75: 25., 1.: 50.}[round(radius/config.wall_radius, 2)]
        normal = np.array([np.cos(np.radians(angle)), np.sin(np.radians(angle)), 0.])
        return normal, local, {'wall_radius': radius}

    monkeypatch.setattr(geometry, '_fit_wall', inconsistent)
    with pytest.raises(geometry.PoseNotFound, match='unstable_wall_normal'):
        geometry._search_wall(points, result, config)
    monkeypatch.setattr(geometry, '_fit_wall', lambda local, p, out, r, c:
                        (out, points[:25], {'wall_radius': r}))
    with pytest.raises(geometry.PoseNotFound):
        geometry._search_wall(points, result, config)


@pytest.mark.parametrize('field', ['rim_search_spacing', 'min_wall_span', 'max_wall_residual',
                                 'max_anchor_plane_distance', 'max_normal_change_deg'])
@pytest.mark.parametrize('value', [0, -1, float('nan'), True])
def test_search_parameter_validation(field, value):
    with pytest.raises(ValueError, match='invalid_config'):
        RimConfig(**{field: value})


@pytest.mark.parametrize('values', [dict(min_wall_points=11), dict(min_wall_points=True),
    dict(rim_search_max_candidates=0), dict(rim_search_max_candidates=1.5),
    dict(max_normal_change_deg=90), dict(wall_depth=.006)])
def test_search_limits_validation(values):
    with pytest.raises(ValueError, match='invalid'):
        RimConfig(**values)


def test_failed_localization_keeps_base_cloud_ids_and_diagnostics():
    from builtin_interfaces.msg import Time
    from rim_locator.main import RimNode
    from rim_locator.pointcloud import read_xyz_object_ids
    clouds, poses, reports = [], [], []
    points = bowl_points()
    stamp = Time(sec=123, nanosec=456)
    node = NS(config=RimConfig(orientation_mode='wall_normal'),
              cloud_pub=NS(publish=clouds.append), poses_publisher=NS(publish=poses.append),
              report=lambda state, **details: reports.append((state, details)))
    RimNode.finish(node, points, np.full(len(points), 7), stamp)
    assert not poses and reports[0][0] == 'failed'
    xyz, ids = read_xyz_object_ids(clouds[0])
    np.testing.assert_allclose(xyz, points, atol=1e-7)
    assert set(ids) == {7} and clouds[0].header.stamp == stamp
    assert reports[0][1]['rejected'][0]['geometry']['searched_contacts'] > 1

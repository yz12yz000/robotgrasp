import math

import numpy as np
import pytest

from rim_locator.config import RimConfig
from rim_locator.rim_detection import locate_grasp_pose


def bowl_points(center=(0.35, 0.20), tilt=0.0):
    angles = np.linspace(0, 2 * np.pi, 240, endpoint=False)
    radius = 0.085
    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)
    z = 0.42 + tilt * (x - center[0])
    return np.c_[x, y, z]


def test_one_pose_for_vertical_and_sloped_rims():
    config = RimConfig(min_points=30, min_rim_points=10, min_local_points=3,
                       near_side_ratio=.25)
    for tilt in (0.0, math.tan(math.radians(20))):
        result = locate_grasp_pose(bowl_points(tilt=tilt), config)
        assert len(result.point) == 3
        assert len(result.orientation) == 4
        assert math.isclose(math.hypot(*result.orientation), 1.0, abs_tol=1e-6)
        assert all(math.isfinite(v) for v in result.rpy)
        # The two-finger opening/closing direction is tool Y.  The generated
        # pose keeps tool Z pointing downward for the existing vertical
        # approach/retreat path.
        x, y, z, w = result.orientation
        rotation = np.array([
            [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
            [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
            [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
        ])
        expected_downward = -0.8 if tilt == 0.0 else -0.1
        assert rotation[2, 2] < expected_downward


def test_degenerate_points_are_rejected():
    config = RimConfig(min_points=30, min_rim_points=10, min_local_points=3)
    points = np.repeat([[.3, .2, .4]], 40, axis=0)
    try:
        locate_grasp_pose(points, config)
    except ValueError as exc:
        assert str(exc).startswith(('rim_not_found', 'pose_not_found'))
    else:
        raise AssertionError('degenerate points unexpectedly produced a pose')


def wall_cloud(center=(.4, .15), tilt_deg=0., top=.4):
    angles, depth = np.meshgrid(np.linspace(0, 2*np.pi, 240, endpoint=False),
                               np.linspace(0, .05, 40))
    radius = .085 - depth * math.tan(math.radians(tilt_deg))
    return np.c_[(center[0] + radius*np.cos(angles)).ravel(),
                 (center[1] + radius*np.sin(angles)).ravel(), (top-depth).ravel()]


def test_wall_normal_actually_controls_y_axis_for_two_bowls():
    from grasp_executor.cartesian import rotate
    config = RimConfig(orientation_mode='wall_normal')
    for angle in (0., 20.):
        points = wall_cloud(tilt_deg=angle)
        result = locate_grasp_pose(points, config)
        y_axis = np.array(rotate(result.orientation, (0., 1., 0.)))
        assert np.dot(y_axis, result.normal) > .99999
        measured = math.degrees(math.asin(abs(y_axis[2])))
        assert abs(measured-angle) < 2.0
        assert result.point[2] > .395
        assert rotate(result.orientation, (0., 0., 1.))[2] < -.90
        assert np.linalg.norm(result.orientation) == pytest.approx(1.)


def test_rim_only_is_not_a_measured_wall_normal():
    import pytest
    with pytest.raises(ValueError, match='insufficient_wall_points'):
        locate_grasp_pose(bowl_points(), RimConfig(orientation_mode='wall_normal'))


def test_multiple_instances_reject_individually_preserve_stamp_and_sort():
    from types import SimpleNamespace as NS
    from builtin_interfaces.msg import Time
    from rim_locator.main import RimNode
    from rim_locator.pointcloud import read_xyz_object_ids
    from yolo_vision.pointcloud import make_cloud
    from std_msgs.msg import Header
    stamp = Time(sec=123, nanosec=45)
    clouds = [wall_cloud((.6, .1), 20.), np.full((40, 3), .3), wall_cloud((.35,.1))]
    points = np.concatenate(clouds)
    ids = np.concatenate([np.full(len(c), i, dtype=np.uint32) for i,c in enumerate(clouds)])
    # Verify the actual PointCloud2 serialization/decoding, including bad XYZ.
    message = make_cloud(Header(stamp=stamp, frame_id='base_link'), points, ids)
    points, ids = read_xyz_object_ids(message)
    poses, status = [], []
    node = NS(config=RimConfig(orientation_mode='wall_normal', publish_debug=False),
              poses_publisher=NS(publish=poses.append), publisher=NS(publish=lambda m:None),
              report=lambda state, **kw:status.append(kw))
    RimNode.finish(node, points, ids, stamp)
    assert len(poses[0].poses) == 2
    assert poses[0].header.stamp == stamp
    assert status[0]['ordered_object_ids'] == [2,0]
    assert status[0]['rejected'][0]['object_id'] == 1


def test_all_rejected_instances_report_each_reason_without_a_pose():
    from types import SimpleNamespace as NS
    from builtin_interfaces.msg import Time
    from rim_locator.main import RimNode
    points = np.repeat([[.3, .2, .4]], 40, axis=0)
    statuses, poses = [], []
    node = NS(config=RimConfig(publish_debug=False),
              report=lambda state, **details: statuses.append((state, details)),
              poses_publisher=NS(publish=poses.append))
    RimNode.finish(node, points, np.full(40, 7), Time(sec=123))
    assert not poses and statuses[0][0] == 'failed'
    assert statuses[0][1]['reason'] == 'pose_not_found:no_valid_object'
    assert statuses[0][1]['rejected'] == [
        {'object_id': 7, 'reason': 'pose_not_found:normal_direction_ambiguous'}]

"""PointCloud2 decoding without a ROS import or loss of pixel correspondence."""

import numpy as np


def read_xyz(cloud):
    """Return H x W x 3; preserve invalid entries until AFTER mask indexing.

    Accepts sensor_msgs/PointCloud2 or a duck-typed equivalent for offline tests.
    Handles per-point padding, row padding and endian order.
    """
    height, width = int(cloud.height), int(cloud.width)
    if height < 1 or width < 0:
        raise ValueError("invalid_cloud_dimensions")
    if width == 0:
        return np.empty((height, 0, 3), dtype=np.float64)
    point_step, row_step = int(cloud.point_step), int(cloud.row_step)
    if point_step <= 0 or row_step < width * point_step:
        raise ValueError("invalid_cloud_stride")
    if len(cloud.data) < height * row_step:
        raise ValueError("truncated_cloud_data")
    fields = {field.name: field for field in cloud.fields}
    endian = ">" if cloud.is_bigendian else "<"
    # PointField.FLOAT32 = 7, FLOAT64 = 8. XYZ must be scalar floats.
    sizes = {7: ("f4", 4), 8: ("f8", 8)}
    channels = []
    for name in ("x", "y", "z"):
        field = fields.get(name)
        if field is None or field.count != 1 or field.datatype not in sizes:
            raise ValueError("invalid_xyz_fields")
        dtype, size = sizes[field.datatype]
        if field.offset < 0 or field.offset + size > point_step:
            raise ValueError("invalid_field_offset")
        channels.append(np.ndarray(
            (height, width), dtype=endian + dtype, buffer=cloud.data,
            offset=field.offset, strides=(row_step, point_step),
        ))
    return np.stack(channels, axis=-1).astype(np.float64, copy=False)


def extract_object_points(xyz, mask, min_points=30):
    xyz = np.asarray(xyz)
    mask = np.asarray(mask)
    if xyz.ndim != 3 or xyz.shape[2] != 3 or mask.shape != xyz.shape[:2]:
        raise ValueError("dimension_mismatch")
    if mask.dtype != np.bool_:
        raise ValueError("mask_must_be_boolean")
    points = xyz[mask]
    # Some camera drivers encode missing depth as the zero vector.
    points = points[np.isfinite(points).all(axis=1) & (points[:, 2] > 0)]
    if len(points) < min_points:
        raise ValueError("insufficient_pointcloud")
    return points


def make_cloud(header, points):
    """Create an unorganized little-endian XYZ cloud; import ROS only here."""
    from sensor_msgs.msg import PointCloud2, PointField

    points = np.asarray(points, dtype="<f4")
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("invalid_output_points")
    message = PointCloud2()
    message.header = header
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(name=name, offset=offset, datatype=7, count=1)
        for name, offset in (("x", 0), ("y", 4), ("z", 8))
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = 12 * len(points)
    message.is_dense = True
    message.data = points.tobytes(order="C")
    return message

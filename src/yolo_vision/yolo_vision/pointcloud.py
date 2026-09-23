"""PointCloud2 decoding without a ROS import or loss of pixel correspondence."""

import numpy as np


def read_aligned_depth_xyz(depth, info):
    """Project registered, rectified depth using its matching CameraInfo.

    Orbbec publishes scaled 16UC1 depth in millimetres; 32FC1 is metres.
    Distorted/unregistered images are rejected instead of guessing a transform.
    """
    if info is None:
        raise ValueError("depth_camera_info_missing")
    if not depth.header.frame_id or depth.header.frame_id != info.header.frame_id:
        raise ValueError("depth_camera_info_frame_mismatch")
    height, width = int(depth.height), int(depth.width)
    if height <= 1 or width <= 0 or (height, width) != (info.height, info.width):
        raise ValueError("depth_camera_info_dimension_mismatch")
    k = np.asarray(info.k, dtype=float)
    d = np.asarray(info.d, dtype=float)
    if (k.shape != (9,) or not np.isfinite(k).all() or k[0] <= 0 or k[4] <= 0
            or not np.allclose(k[[1, 3, 6, 7, 8]], [0, 0, 0, 0, 1])):
        raise ValueError("invalid_depth_intrinsics")
    if not np.isfinite(d).all() or np.any(d != 0):
        raise ValueError("depth_must_be_rectified")
    formats = {"16UC1": ("u2", 2, .001), "32FC1": ("f4", 4, 1.)}
    if depth.encoding not in formats:
        raise ValueError("unsupported_depth_encoding")
    dtype, size, scale = formats[depth.encoding]
    if depth.step < width * size or len(depth.data) < height * depth.step:
        raise ValueError("invalid_depth_buffer")
    z = np.ndarray((height, width), dtype=(">" if depth.is_bigendian else "<") + dtype,
                   buffer=depth.data, strides=(depth.step, size)).astype(np.float64) * scale
    xyz = np.empty((height, width, 3), dtype=np.float64)
    xyz[:, :, 2] = z
    xyz[:, :, 0] = z * (np.arange(width)[None, :] - k[2]) / k[0]
    xyz[:, :, 1] = z * (np.arange(height)[:, None] - k[5]) / k[4]
    return xyz


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


def make_cloud(header, points, object_ids=None):
    """Create an unorganized little-endian XYZ cloud; import ROS only here."""
    from sensor_msgs.msg import PointCloud2, PointField

    points = np.asarray(points, dtype="<f4")
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("invalid_output_points")
    if object_ids is not None:
        object_ids = np.asarray(object_ids, dtype="<u4")
        if object_ids.ndim != 1 or len(object_ids) != len(points):
            raise ValueError("object_id_length_mismatch")
        payload = np.empty(len(points), dtype=[("x", "<f4"), ("y", "<f4"),
                                                ("z", "<f4"), ("object_id", "<u4")])
        payload["x"], payload["y"], payload["z"] = points.T
        payload["object_id"] = object_ids
        fields = [
            PointField(name="x", offset=0, datatype=7, count=1),
            PointField(name="y", offset=4, datatype=7, count=1),
            PointField(name="z", offset=8, datatype=7, count=1),
            PointField(name="object_id", offset=12, datatype=6, count=1),
        ]
        message = PointCloud2()
        message.header = header
        message.height, message.width = 1, len(points)
        message.fields, message.is_bigendian = fields, False
        message.point_step, message.row_step = 16, 16 * len(points)
        message.is_dense, message.data = True, payload.tobytes(order="C")
        return message
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

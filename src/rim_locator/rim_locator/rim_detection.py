"""Pure NumPy geometry: no ROS, YOLO, OpenVINO, or hardware dependency."""

from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class RimResult:
    point: np.ndarray
    z_top: float
    candidates: np.ndarray
    local_points: np.ndarray


def valid_points(points):
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("invalid_points_shape")
    return points[np.isfinite(points).all(axis=1)]


def transform_points(points, translation, quaternion):
    """Apply source -> target transform; quaternion order is x, y, z, w."""
    points = valid_points(points)
    translation = np.asarray(translation, dtype=np.float64)
    q = np.asarray(quaternion, dtype=np.float64)
    if translation.shape != (3,) or q.shape != (4,) or not np.isfinite(translation).all() or not np.isfinite(q).all():
        raise ValueError("invalid_transform")
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid_transform_quaternion")
    x, y, z, w = q / norm
    rotation = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])
    transformed = points @ rotation.T + translation
    if not np.isfinite(transformed).all():
        raise ValueError("nonfinite_transformed_points")
    return transformed


def locate_rim(points, config):
    points = valid_points(points)
    if len(points) == 0:
        raise ValueError("empty_cloud")
    if len(points) < config.min_points:
        raise ValueError("insufficient_points")
    z_top = float(np.percentile(points[:, 2], config.top_percentile))
    # Intentionally retain the exact lower-bound-only algorithm in the design.
    candidates = points[points[:, 2] >= z_top - config.rim_height_band]
    if len(candidates) < config.min_rim_points:
        raise ValueError("rim_not_found:insufficient_candidates")
    count = max(config.min_local_points, math.ceil(len(candidates) * config.near_side_ratio))
    if count > len(candidates):
        raise ValueError("rim_not_found:insufficient_local_points")
    distances = np.hypot(candidates[:, 0], candidates[:, 1])
    order = np.argsort(distances, kind="stable")
    local_points = candidates[order[:count]]
    point = np.median(local_points, axis=0)
    return RimResult(point, z_top, candidates, local_points)

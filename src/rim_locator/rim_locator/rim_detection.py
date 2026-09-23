"""Pure NumPy geometry: no ROS, YOLO, OpenVINO, or hardware dependency."""

from dataclasses import dataclass, field
from collections import Counter
from itertools import product
import math
import numpy as np


@dataclass(frozen=True)
class RimResult:
    point: np.ndarray
    z_top: float
    candidates: np.ndarray
    local_points: np.ndarray


@dataclass(frozen=True)
class PoseResult:
    point: np.ndarray
    orientation: tuple
    rpy: tuple
    z_top: float
    candidates: np.ndarray
    local_points: np.ndarray
    normal: np.ndarray
    wall_points: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    diagnostics: dict = field(default_factory=dict)


class PoseNotFound(ValueError):
    """Keep the error string interface while exposing search evidence to ROS."""
    def __init__(self, reason, diagnostics):
        super().__init__(reason)
        self.diagnostics = diagnostics


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


def filter_component(points, config):
    """Keep original points in the dominant 26-connected voxel component.

    Memory is linear in points/occupied voxels; no pairwise distances. A
    component is weighted by point count, not by its number of occupied cells.
    """
    points = valid_points(points)
    if len(points) == 0:
        raise ValueError("empty_cloud")
    if len(points) < config.min_points:
        raise ValueError("insufficient_points")
    if not config.filter_disconnected_points:
        return points
    cells = np.floor(points / config.component_voxel_size)
    if not np.isfinite(cells).all():
        raise ValueError("pose_not_found:invalid_voxel_coordinates")
    # Python ints avoid int64 overflow for finite but very large coordinates.
    occupied = {}
    for index, cell in enumerate(cells):
        key = tuple(int(v) for v in cell)
        occupied.setdefault(key, []).append(index)
    neighbors = [delta for delta in product((-1, 0, 1), repeat=3) if delta != (0, 0, 0)]
    largest = []
    while occupied:
        seed, indices = occupied.popitem()
        component = list(indices)
        pending = [seed]
        while pending:
            x, y, z = pending.pop()
            for dx, dy, dz in neighbors:
                key = (x + dx, y + dy, z + dz)
                indices = occupied.pop(key, None)
                if indices is not None:
                    component.extend(indices)
                    pending.append(key)
        if len(component) > len(largest):
            largest = component
    if len(largest) < config.min_points or len(largest) / len(points) < config.min_component_ratio:
        raise ValueError("pose_not_found:ambiguous_components")
    # Preserve input ordering as well as exact original coordinates.
    return points[np.sort(largest)]


def locate_rim(points, config):
    return _locate_filtered_rim(filter_component(points, config), config)


def _locate_filtered_rim(points, config):
    z_top = float(np.percentile(points[:, 2], config.top_percentile))
    span = float(np.percentile(points[:, 2], 95) - np.percentile(points[:, 2], 5))
    band = min(config.max_height_band, max(config.min_height_band,
                                          config.adaptive_height_fraction * max(span, 1e-6)))
    # The percentile defines a rim-height band, not an unbounded upper tail.
    # Connected depth streaks above a reflective rim can otherwise win the
    # nearest-contact search despite being only a tiny fraction of the cloud.
    # Keep the full cloud for wall fitting; only bound candidate anchor height.
    candidates = points[(points[:, 2] >= z_top - band) & (points[:, 2] <= z_top + band)]
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


def _quaternion_from_matrix(matrix):
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all() or abs(np.linalg.det(matrix) - 1.0) > 1e-3:
        raise ValueError("invalid_pose_rotation")
    trace = float(np.trace(matrix))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = np.array([(matrix[2, 1]-matrix[1, 2])/s, (matrix[0, 2]-matrix[2, 0])/s,
                      (matrix[1, 0]-matrix[0, 1])/s, 0.25*s])
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = math.sqrt(max(1e-12, 1 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])) * 2
        q = np.array([0.25*s, (matrix[0, 1]+matrix[1, 0])/s,
                      (matrix[0, 2]+matrix[2, 0])/s, (matrix[2, 1]-matrix[1, 2])/s])
    elif matrix[1, 1] > matrix[2, 2]:
        s = math.sqrt(max(1e-12, 1 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])) * 2
        q = np.array([(matrix[0, 1]+matrix[1, 0])/s, 0.25*s,
                      (matrix[1, 2]+matrix[2, 1])/s, (matrix[0, 2]-matrix[2, 0])/s])
    else:
        s = math.sqrt(max(1e-12, 1 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])) * 2
        q = np.array([(matrix[0, 2]+matrix[2, 0])/s, (matrix[1, 2]+matrix[2, 1])/s,
                      (matrix[1, 0]-matrix[0, 1])/s, 0.25*s])
    q /= np.linalg.norm(q)
    return tuple(float(x) for x in q)


def quaternion_to_rpy(q):
    x, y, z, w = q
    return (float(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))),
            float(math.asin(max(-1., min(1., 2*(w*y - z*x))))),
            float(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))))


def _outward(point, centre):
    outward = point - centre
    outward[2] = 0.
    if np.linalg.norm(outward) < 1e-6:
        raise ValueError("pose_not_found:normal_direction_ambiguous")
    outward /= np.linalg.norm(outward)
    return outward


def _contact(local):
    # Keep the anchor on the upper observed surface, not halfway down the wall.
    upper = local[local[:, 2] >= np.median(local[:, 2])]
    return np.median(upper, axis=0)


def _rim_contacts(result, config):
    """Spatially distinct observed contacts; bounded linear memory, no NxN array."""
    yield _contact(result.local_points), result.local_points
    seeds, contacts = [], []
    candidates = result.candidates
    order = np.lexsort((candidates[:, 2], candidates[:, 1], candidates[:, 0],
                        np.hypot(candidates[:, 0], candidates[:, 1])))
    for candidate in candidates[order]:
        if seeds and np.min(np.linalg.norm(np.asarray(seeds) - candidate[:2], axis=1)) < config.rim_search_spacing:
            continue
        seeds.append(candidate[:2])
        local = candidates[np.linalg.norm(candidates[:, :2] - candidate[:2], axis=1) <= config.rim_search_spacing]
        if len(local) >= config.min_local_points:
            contacts.append((_contact(local), local))
        if len(seeds) >= config.rim_search_max_candidates:
            break
    # The closest valid supported contact wins; confidence does not set order.
    yield from sorted(contacts, key=lambda item: (np.hypot(*item[0][:2]), *item[0]))


def _fit_wall(local, point, outward, radius, config):
    if len(local) < max(config.min_local_points, config.min_wall_points):
        raise ValueError("pose_not_found:insufficient_wall_points")
    values, vectors = np.linalg.eigh(np.cov(local, rowvar=False, bias=True))
    if (values[1] <= 1e-8 or values[1] / max(values[2], 1e-12) < .02
            or values[0] / values[1] > config.max_planarity_ratio):
        raise ValueError("pose_not_found:degenerate_wall_neighborhood")
    normal = vectors[:, 0]
    if np.dot(normal, outward) < 0:
        normal = -normal
    if (abs(normal[2]) > math.sin(math.radians(config.max_wall_tilt_deg))
            or np.dot(normal[:2], outward[:2]) < .5):
        raise ValueError("pose_not_found:wall_normal_out_of_range")
    residual = math.sqrt(max(0., float(values[0])))
    height_span = float(np.diff(np.percentile(local[:, 2], [5, 95]))[0])
    gap = abs(float(np.dot(point - np.mean(local, axis=0), normal)))
    if height_span < config.min_wall_span:
        raise ValueError("pose_not_found:insufficient_wall_span")
    if residual > config.max_wall_residual:
        raise ValueError("pose_not_found:wall_fit_residual")
    if gap > config.max_anchor_plane_distance:
        raise ValueError("pose_not_found:anchor_not_on_wall")
    return normal, local, dict(wall_radius=radius, wall_count=len(local),
                              wall_span=height_span, wall_residual=residual,
                              anchor_plane_distance=gap,
                              wall_tilt_deg=math.degrees(math.asin(abs(normal[2]))))


def _search_wall(points, result, config):
    centre = np.median(points, axis=0)
    rejected = Counter()
    first_reason = None
    attempts = 0
    for point, local_rim in _rim_contacts(result, config):
        attempts += 1
        try:
            outward = _outward(point, centre)
        except ValueError as exc:
            rejected[str(exc)] += 1
            first_reason = first_reason or str(exc)
            continue
        xy_distance = np.linalg.norm(points[:, :2] - point[:2], axis=1)
        below = ((points[:, 2] < point[2] - config.min_height_band)
                 & (points[:, 2] >= point[2] - config.wall_depth))
        fits = []
        # A large patch can include the curved transition into the bowl bottom.
        # Require agreement at two scales; a single lucky small patch is insufficient.
        for scale in (.5, .75, 1.):
            radius = config.wall_radius * scale
            try:
                fits.append(_fit_wall(points[below & (xy_distance <= radius)],
                                      point, outward, radius, config))
            except ValueError as exc:
                rejected[str(exc)] += 1
                first_reason = first_reason or str(exc)
        for index, fit in enumerate(fits):
            for other in fits[index + 1:]:
                change = math.degrees(math.acos(float(np.clip(np.dot(fit[0], other[0]), -1., 1.))))
                extra_support = len(other[1]) - len(fit[1])
                if change <= config.max_normal_change_deg and extra_support >= max(3, math.ceil(.1 * len(fit[1]))):
                    # Smaller neighborhood estimates the wall AT the contact;
                    # the larger one corroborates rather than averaging in the bottom.
                    diagnostics = dict(fit[2], corroborating_radius=other[2]['wall_radius'],
                                       corroborating_count=len(other[1]),
                                       normal_change_deg=change, searched_contacts=attempts,
                                       rejected_neighborhoods=dict(rejected), method='supported_wall_search')
                    return point, fit[0], local_rim, fit[1], diagnostics
        if len(fits) >= 2:
            rejected['pose_not_found:unstable_wall_normal'] += 1
            first_reason = first_reason or 'pose_not_found:unstable_wall_normal'
        elif fits:
            rejected['pose_not_found:uncorroborated_wall_normal'] += 1
            first_reason = first_reason or 'pose_not_found:uncorroborated_wall_normal'
    primary_reason = rejected.most_common(1)[0][0] if rejected else 'pose_not_found:no_supported_contact'
    raise PoseNotFound(primary_reason,
                       dict(searched_contacts=attempts, rejected_neighborhoods=dict(rejected),
                            first_rejection=first_reason))


def locate_grasp_pose(points, config):
    """Return exactly one supported geometry-derived PoseResult for one bowl."""
    points = filter_component(points, config)
    result = _locate_filtered_rim(points, config)
    wall_points, diagnostics = np.empty((0, 3)), {}
    local_points = result.local_points
    if config.orientation_mode == 'wall_normal':
        point, normal, local_points, wall_points, diagnostics = _search_wall(points, result, config)
        diagnostics['measured_wall_normal'] = normal.tolist()
        if config.max_tool_tilt_deg is not None:
            # Wall evidence validates the contact, but pinching the rim from
            # above need not tilt the entire tool to follow a sloping bowl wall.
            # Preserve its measured horizontal closing direction and the
            # existing base-Z approach/retreat convention.
            measured_tilt = math.asin(float(np.clip(normal[2], -1., 1.)))
            limit = math.radians(config.max_tool_tilt_deg)
            tilt = float(np.clip(measured_tilt, -limit, limit))
            horizontal = normal[:2] / np.linalg.norm(normal[:2])
            normal = np.r_[horizontal * math.cos(tilt), math.sin(tilt)]
        diagnostics['tool_tilt_deg'] = math.degrees(math.asin(abs(float(normal[2]))))
    else:
        point = _contact(local_points)
        normal = _outward(point, np.median(points, axis=0))
    # Confirmed physical convention: tool Y closes across the wall, tool Z
    # points down along the wall. Translation remains along base Z.
    y_axis = normal
    down = np.array([0., 0., -1.])
    x_axis = np.cross(y_axis, down)
    if np.linalg.norm(x_axis) < 1e-6:
        raise ValueError("pose_not_found:closing_axis_parallel_to_approach")
    x_axis /= np.linalg.norm(x_axis)
    z_axis = np.cross(x_axis, y_axis)
    z_axis /= np.linalg.norm(z_axis)
    q = _quaternion_from_matrix(np.column_stack((x_axis, y_axis, z_axis)))
    return PoseResult(point, q, quaternion_to_rpy(q), result.z_top,
                      result.candidates, local_points, normal, wall_points, diagnostics)

"""Configuration for supported rim contact and wall-normal estimation."""

from dataclasses import dataclass
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class RimConfig:
    input_topic: str = "/yolo_vision/object_cloud"
    output_topic: str = "/rim_locator/rim_point"
    poses_topic: str = "/rim_locator/grasp_poses"
    target_frame: str = "base_link"
    top_percentile: float = 98.0
    rim_height_band: float = 0.015
    near_side_ratio: float = 0.1
    min_points: int = 30
    min_rim_points: int = 10
    min_local_points: int = 3
    tf_timeout: float = 5.0
    input_timeout: float = 60.0
    publish_debug: bool = True
    adaptive_height_fraction: float = 0.20
    min_height_band: float = 0.006
    max_height_band: float = 0.012
    max_planarity_ratio: float = 0.35
    orientation_mode: str = "vertical"
    wall_radius: float = 0.025
    wall_depth: float = 0.030
    max_wall_tilt_deg: float = 40.0
    min_output_z: float = -0.10
    max_output_z: float = 1.50
    filter_disconnected_points: bool = True
    component_voxel_size: float = 0.008
    min_component_ratio: float = 0.60
    rim_search_spacing: float = 0.008
    rim_search_max_candidates: int = 96
    min_wall_points: int = 20
    min_wall_span: float = 0.006
    max_wall_residual: float = 0.0025
    max_anchor_plane_distance: float = 0.006
    max_normal_change_deg: float = 12.0
    max_tool_tilt_deg: float | None = None  # None follows wall; 0 is vertical rim pinch.

    def __post_init__(self):
        value = self.max_tool_tilt_deg
        if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float))
                                  or not math.isfinite(value) or not 0 <= value < 90):
            raise ValueError('invalid_config:max_tool_tilt_deg')
        if type(self.filter_disconnected_points) is not bool:
            raise ValueError("invalid_config:filter_disconnected_points")
        for name in ("component_voxel_size", "min_component_ratio", "rim_search_spacing",
                     "min_wall_span", "max_wall_residual", "max_anchor_plane_distance", "max_normal_change_deg"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"invalid_config:{name}")
        if not .5 < self.min_component_ratio <= 1:
            raise ValueError("invalid_config:min_component_ratio")
        for name in ("input_topic", "output_topic", "poses_topic", "target_frame"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"invalid_config:{name}")
        if self.target_frame.startswith("/"):
            raise ValueError("frame_id_must_not_start_with_slash")
        for name in ("top_percentile", "rim_height_band", "near_side_ratio", "tf_timeout", "input_timeout"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"invalid_config:{name}")
        if self.top_percentile > 100 or self.near_side_ratio > 1:
            raise ValueError("invalid_percentile_or_ratio")
        for name in ("adaptive_height_fraction", "min_height_band", "max_height_band", "max_planarity_ratio", "wall_radius", "wall_depth", "max_wall_tilt_deg"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"invalid_config:{name}")
        if self.orientation_mode not in ("vertical", "wall_normal") or self.max_wall_tilt_deg >= 90:
            raise ValueError("invalid_orientation_config")
        if self.max_normal_change_deg >= 90 or self.wall_depth <= self.min_height_band:
            raise ValueError("invalid_wall_search_config")
        if self.min_height_band > self.max_height_band or self.adaptive_height_fraction > 1:
            raise ValueError("invalid_height_band_config")
        if not math.isfinite(self.min_output_z) or not math.isfinite(self.max_output_z) or self.min_output_z >= self.max_output_z:
            raise ValueError("invalid_output_workspace")
        for name in ("min_points", "min_rim_points", "min_local_points", "rim_search_max_candidates", "min_wall_points"):
            if type(getattr(self, name)) is not int or getattr(self, name) < 1:
                raise ValueError(f"invalid_config:{name}")
        if self.min_wall_points < 12:
            raise ValueError("invalid_config:min_wall_points")
        if type(self.publish_debug) is not bool:
            raise ValueError("invalid_config:publish_debug")


def load_config(path):
    with Path(path).expanduser().open(encoding="utf-8") as stream:
        return RimConfig(**json.load(stream))

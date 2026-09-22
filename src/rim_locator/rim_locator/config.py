"""Configuration for the first-version height-band method."""

from dataclasses import dataclass
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class RimConfig:
    input_topic: str = "/yolo_vision/object_cloud"
    output_topic: str = "/rim_locator/rim_point"
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

    def __post_init__(self):
        for name in ("input_topic", "output_topic", "target_frame"):
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
        for name in ("min_points", "min_rim_points", "min_local_points"):
            if type(getattr(self, name)) is not int or getattr(self, name) < 1:
                raise ValueError(f"invalid_config:{name}")
        if type(self.publish_debug) is not bool:
            raise ValueError("invalid_config:publish_debug")


def load_config(path):
    with Path(path).expanduser().open(encoding="utf-8") as stream:
        return RimConfig(**json.load(stream))

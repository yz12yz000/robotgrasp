"""Validated JSON configuration; paths are resolved against the package share."""

from dataclasses import dataclass
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class VisionConfig:
    rgb_topic: str = "/camera/color/image_raw"
    pointcloud_topic: str = "/camera/depth/points"
    output_topic: str = "/yolo_vision/object_cloud"
    target_class: str = "bowl"
    confidence: float = 0.25
    imgsz: int = 512
    device: str = "intel:cpu"
    model_path: str = "yolo26n-seg_openvino_model"
    sync_queue_size: int = 30
    sync_slop: float = 0.05
    input_timeout: float = 30.0
    min_points: int = 30
    expected_width: int = 1280
    expected_height: int = 800
    publish_debug: bool = True
    inference_roi: list | None = None  # Optional [x0, y0, x1, y1], original pixels.

    def __post_init__(self):
        for name in ("rgb_topic", "pointcloud_topic", "output_topic", "target_class", "model_path"):
            if not isinstance(getattr(self, name), str) or not getattr(self, name).strip():
                raise ValueError(f"invalid_config:{name}")
        if self.device != "intel:cpu":
            raise ValueError("first_version_requires_intel:cpu")
        for name in ("confidence", "sync_slop", "input_timeout"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"invalid_config:{name}")
        if self.confidence > 1:
            raise ValueError("invalid_config:confidence")
        for name in ("imgsz", "sync_queue_size", "min_points", "expected_width", "expected_height"):
            if type(getattr(self, name)) is not int or getattr(self, name) < 1:
                raise ValueError(f"invalid_config:{name}")
        if type(self.publish_debug) is not bool:
            raise ValueError("invalid_config:publish_debug")
        if self.inference_roi is not None:
            r = self.inference_roi
            if (not isinstance(r, list) or len(r) != 4 or any(type(v) is not int for v in r)
                    or not 0 <= r[0] < r[2] <= self.expected_width
                    or not 0 <= r[1] < r[3] <= self.expected_height):
                raise ValueError("invalid_config:inference_roi")


def load_config(path):
    with Path(path).expanduser().open(encoding="utf-8") as stream:
        return VisionConfig(**json.load(stream))


def resolve_model_path(config, share):
    path = Path(config.model_path).expanduser()
    return path if path.is_absolute() else Path(share) / path

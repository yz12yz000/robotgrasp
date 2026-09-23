"""Validated JSON configuration; paths are resolved against the package share."""

from dataclasses import dataclass
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class VisionConfig:
    rgb_topic: str = "/camera/color/image_raw"
    pointcloud_topic: str = "/camera/depth/points"
    depth_image_topic: str | None = None
    depth_info_topic: str = "/camera/depth/camera_info"
    output_topic: str = "/yolo_vision/object_cloud"
    target_class: str = "bowl"
    confidence: float = 0.25
    imgsz: int = 512
    device: str = "intel:cpu"
    model_path: str = "yolo26n-seg_openvino_model"
    sync_queue_size: int = 100
    sync_slop: float = 0.05
    input_timeout: float = 120.0
    min_points: int = 30
    expected_width: int = 1280
    expected_height: int = 800
    publish_debug: bool = True
    max_source_age: float = 10.0
    inference_roi: list | None = None  # Optional [x0, y0, x1, y1], original pixels.
    inference_tile_size: int = 0  # 0 keeps the legacy full-image model call.
    tile_overlap: float = 0.5
    tile_mask_iou: float = 0.5
    tile_single_confidence: float = 0.35

    def __post_init__(self):
        if type(self.inference_tile_size) is not int or self.inference_tile_size < 0 or 0 < self.inference_tile_size < 64:
            raise ValueError('invalid_config:inference_tile_size')
        for name in ('tile_overlap', 'tile_mask_iou', 'tile_single_confidence'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value < 1:
                raise ValueError('invalid_config:' + name)
        if self.tile_overlap > .75:
            raise ValueError('invalid_config:tile_overlap')
        for name in ("rgb_topic", "pointcloud_topic", "depth_info_topic", "output_topic", "target_class", "model_path"):
            if not isinstance(getattr(self, name), str) or not getattr(self, name).strip():
                raise ValueError(f"invalid_config:{name}")
        if self.depth_image_topic is not None and (not isinstance(self.depth_image_topic, str)
                                                   or not self.depth_image_topic.strip()):
            raise ValueError("invalid_config:depth_image_topic")
        if self.device != "intel:cpu":
            raise ValueError("first_version_requires_intel:cpu")
        for name in ("confidence", "sync_slop", "input_timeout", "max_source_age"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"invalid_config:{name}")
        if self.confidence > 1:
            raise ValueError("invalid_config:confidence")
        for name in ("imgsz", "sync_queue_size", "min_points", "expected_width", "expected_height"):
            if type(getattr(self, name)) is not int or getattr(self, name) < 1:
                raise ValueError(f"invalid_config:{name}")
        for name in ("publish_debug",):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f"invalid_config:{name}")
        if self.inference_roi is not None:
            r = self.inference_roi
            if (not isinstance(r, list) or len(r) != 4 or any(type(v) is not int for v in r)
                    or not 0 <= r[0] < r[2] <= self.expected_width
                    or not 0 <= r[1] < r[3] <= self.expected_height):
                raise ValueError("invalid_config:inference_roi")


def _read_layer(path):
    with Path(path).expanduser().open(encoding="utf-8") as stream:
        values = json.load(stream)
    if not isinstance(values, dict):
        raise ValueError(f"invalid_config:expected_object:{path}")
    # Old files remain readable, but this key no longer selects a detection mode.
    if "multi_instance" in values:
        if type(values.pop("multi_instance")) is not bool:
            raise ValueError("invalid_config:multi_instance")
    return values


def load_config(path):
    return VisionConfig(**_read_layer(path))


def load_effective_config(base_path, override_path=None):
    """Merge explicit fields; detection always returns all observed targets."""
    VisionConfig()  # Validate defaults independently of the supplied layers.
    values = _read_layer(base_path)
    VisionConfig(**values)
    if override_path is not None:
        override = _read_layer(override_path)
        # Validate explicit values with inherited dimensions (e.g. ROI bounds).
        values = {**values, **override}
        VisionConfig(**values)
    return VisionConfig(**values)


def resolve_model_path(config, share):
    path = Path(config.model_path).expanduser()
    return path if path.is_absolute() else Path(share) / path

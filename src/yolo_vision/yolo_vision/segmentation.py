"""YOLO/OpenVINO adapter. NumPy inputs are BGR, as required by Ultralytics."""

from dataclasses import dataclass
from pathlib import Path
import numpy as np


@dataclass(frozen=True)
class SegmentationResult:
    mask: np.ndarray
    class_name: str
    confidence: float


def select_target(result, target_class, image_shape):
    """Select one instance; require masks already restored to original pixels.

    Never blindly resize a letterboxed mask. The inference adapter requests
    retina_masks=True, so a mismatch here is a backend/configuration error.
    """
    if result.boxes is None or result.masks is None:
        raise ValueError("no_target:missing_boxes_or_masks")
    classes = result.boxes.cls.cpu().numpy().astype(int)
    scores = result.boxes.conf.cpu().numpy()
    matches = [i for i, cls in enumerate(classes)
               if result.names[int(cls)] == target_class and np.isfinite(scores[i])]
    if not matches:
        detected = [
            f"{result.names[int(cls)]}:{float(score):.3f}"
            for cls, score in zip(classes, scores)
        ]
        raise ValueError("no_target:detected=" + ",".join(detected[:20]))
    index = max(matches, key=lambda i: float(scores[i]))
    mask = result.masks.data[index].cpu().numpy()
    if mask.shape != tuple(image_shape[:2]):
        raise ValueError("dimension_mismatch:mask_not_in_original_image_coordinates")
    mask = mask > 0.5
    if not mask.any():
        raise ValueError("no_target:empty_mask")
    return SegmentationResult(mask, target_class, float(scores[index]))


class Segmenter:
    def __init__(self, model_path, config):
        path = Path(model_path)
        if not path.is_dir() or not list(path.glob("*.xml")) or not list(path.glob("*.bin")):
            raise ValueError(f"openvino_model_missing:{path}")
        from ultralytics import YOLO

        self.config = config
        self.model = YOLO(str(path), task="segment")

    def predict(self, bgr):
        if bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8:
            raise ValueError("invalid_bgr_image")
        roi = self.config.inference_roi
        image = bgr
        if roi is not None:
            x0, y0, x1, y1 = roi
            if x1 > bgr.shape[1] or y1 > bgr.shape[0]:
                raise ValueError("inference_roi_outside_image")
            image = np.ascontiguousarray(bgr[y0:y1, x0:x1])
        results = self.model.predict(
            source=image, conf=self.config.confidence, imgsz=self.config.imgsz,
            device=self.config.device, retina_masks=True, rect=False,
            verbose=False, save=False,
        )
        if len(results) != 1:
            raise ValueError("invalid_inference_result")
        selected = select_target(results[0], self.config.target_class, image.shape)
        if roi is None:
            return selected
        # Restore ORIGINAL pixel coordinates before selecting organized XYZ.
        mask = np.zeros(bgr.shape[:2], dtype=bool)
        mask[y0:y1, x0:x1] = selected.mask
        return SegmentationResult(mask, selected.class_name, selected.confidence)

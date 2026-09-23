"""YOLO/OpenVINO adapter. NumPy inputs are BGR, as required by Ultralytics."""

from dataclasses import dataclass
from pathlib import Path
import numpy as np


@dataclass(frozen=True)
class SegmentationResult:
    mask: np.ndarray
    class_name: str
    confidence: float


def _mask_for(result, index, image_shape):
    mask = result.masks.data[index].cpu().numpy()
    if mask.shape != tuple(image_shape[:2]):
        raise ValueError("dimension_mismatch:mask_not_in_original_image_coordinates")
    mask = mask > 0.5
    if not mask.any():
        raise ValueError("invalid_inference_result:empty_target_mask")
    return mask


def select_targets(result, target_class, image_shape):
    """Return every finite-confidence instance of ``target_class``.

    The detector order is made deterministic by confidence (descending) and
    the original model index.  Object IDs are local to one synchronized frame.
    """
    if result.boxes is None:
        return []
    classes = result.boxes.cls.cpu().numpy().astype(int)
    scores = result.boxes.conf.cpu().numpy()
    matches = [i for i, cls in enumerate(classes)
               if result.names[int(cls)] == target_class and np.isfinite(scores[i])]
    if not matches:
        return []
    if result.masks is None:
        raise ValueError("invalid_inference_result:missing_target_masks")
    targets = []
    for index in sorted(matches, key=lambda i: (-float(scores[i]), i)):
        targets.append(SegmentationResult(_mask_for(result, index, image_shape),
                                           target_class, float(scores[index])))
    return targets



class Segmenter:
    def __init__(self, model_path, config):
        path = Path(model_path)
        if not path.is_dir() or not list(path.glob("*.xml")) or not list(path.glob("*.bin")):
            raise ValueError(f"openvino_model_missing:{path}")
        from ultralytics import YOLO

        self.config = config
        self.model = YOLO(str(path), task="segment")

    def predict_all(self, bgr):
        if bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8:
            raise ValueError("invalid_bgr_image")
        roi = self.config.inference_roi
        image = bgr
        if roi is not None:
            x0, y0, x1, y1 = roi
            if x1 > bgr.shape[1] or y1 > bgr.shape[0]:
                raise ValueError("inference_roi_outside_image")
            image = np.ascontiguousarray(bgr[y0:y1, x0:x1])
        selected = self._predict_tiles(image) if self.config.inference_tile_size else self._predict_image(image)
        if roi is None:
            return selected
        # Restore ORIGINAL pixel coordinates before selecting organized XYZ.
        restored = []
        for target in selected:
            mask = np.zeros(bgr.shape[:2], dtype=bool)
            mask[y0:y1, x0:x1] = target.mask
            restored.append(SegmentationResult(mask, target.class_name, target.confidence))
        return restored

    def _predict_image(self, image):
        results = self.model.predict(
            source=image, conf=self.config.confidence, imgsz=self.config.imgsz,
            device=self.config.device, retina_masks=True, rect=False,
            verbose=False, save=False,
        )
        if len(results) != 1:
            raise ValueError("invalid_inference_result")
        return select_targets(results[0], self.config.target_class, image.shape)

    def _predict_tiles(self, image):
        height, width = image.shape[:2]
        # Keep a whole-view pass for large objects that cannot fit inside a tile.
        selected = list(self._predict_image(image))
        views = [0] * len(selected)
        windows = tile_windows(width, height, self.config.inference_tile_size, self.config.tile_overlap)
        # Crop the SAME synchronized image; never combine detections from
        # different acquisition times. Overlapping windows cover the full view.
        for view, (x0, y0, x1, y1) in enumerate(windows, 1):
            crop = np.ascontiguousarray(image[y0:y1, x0:x1])
            for target in self._predict_image(crop):
                yy, xx = np.nonzero(target.mask)
                # Internal crop boundaries can cut a bowl in half and yield a
                # misleadingly high score. A neighboring window supplies its
                # full mask. Actual image edges are not artificial tile seams.
                if ((x0 > 0 and xx.min() < 2) or (y0 > 0 and yy.min() < 2)
                        or (x1 < width and xx.max() >= x1-x0-2)
                        or (y1 < height and yy.max() >= y1-y0-2)):
                    continue
                mask = np.zeros((height, width), dtype=bool)
                mask[y0:y1, x0:x1] = target.mask
                selected.append(SegmentationResult(mask, target.class_name, target.confidence))
                views.append(view)
        return suppress_duplicate_masks(selected, self.config.tile_mask_iou, views=views,
                                        min_support=2, single_confidence=self.config.tile_single_confidence)


def tile_windows(width, height, size, overlap):
    def starts(length):
        if length <= size:
            return [0]
        stride = max(1, round(size*(1-overlap)))
        return sorted(set(list(range(0, length-size+1, stride)) + [length-size]))
    return [(x, y, min(x+size, width), min(y+size, height)) for y in starts(height) for x in starts(width)]


def suppress_duplicate_masks(targets, threshold, views=None, min_support=1, single_confidence=0.):
    groups = []
    views = list(range(len(targets))) if views is None else views
    for index in sorted(range(len(targets)), key=lambda i: -targets[i].confidence):
        target = targets[index]
        for other, supporting_views in groups:
            overlap = np.count_nonzero(target.mask & other.mask) / max(1, np.count_nonzero(target.mask | other.mask))
            if overlap >= threshold:
                supporting_views.add(views[index])
                break
        else:
            groups.append((target, {views[index]}))
    return [target for target, supporting_views in groups
            if target.confidence >= single_confidence or len(supporting_views) >= min_support]

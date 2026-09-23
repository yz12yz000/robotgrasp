"""Instance debug image: the labels use the same IDs and overlap policy as XYZ."""
import cv2
import numpy as np


def detection_debug(bgr, results, xyz):
    combined = np.zeros(bgr.shape[:2], dtype=np.uint8)
    overlay = bgr.copy()
    occupied = np.zeros(bgr.shape[:2], dtype=bool)
    valid = np.isfinite(xyz).all(axis=2) & (xyz[:, :, 2] > 0)
    palette = ((40, 200, 40), (220, 160, 20), (180, 60, 220), (30, 170, 240))
    for object_id, result in enumerate(results):
        mask = result.mask & ~occupied
        occupied |= result.mask
        combined[mask] = min(255, object_id + 1)
        colour = palette[object_id % len(palette)]
        overlay[mask] = (.5 * overlay[mask] + .5 * np.asarray(colour)).astype(np.uint8)
        overlay[mask & ~valid] = (0, 0, 255)
        yy, xx = np.nonzero(mask)
        if not len(xx):
            continue
        x, y = int(xx.min()), int(yy.min())
        cv2.rectangle(overlay, (x, y), (int(xx.max()), int(yy.max())), colour, 1)
        label = f'id={object_id} {result.confidence:.2f} depth={int((mask & valid).sum())}/{len(xx)}'
        cv2.putText(overlay, label, (max(0, min(x, bgr.shape[1]-310)), max(14, y-5)),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, colour, 1, cv2.LINE_AA)
    return combined, overlay

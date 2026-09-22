#!/usr/bin/env python3
"""Mark the saved rim result on the RGB overlay using mask/cloud ordering."""
import argparse
from pathlib import Path
import json
import cv2
import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument("output_dir", type=Path)
    args = p.parse_args()
    d = args.output_dir
    image = cv2.imread(str(d / "overlay.png"), cv2.IMREAD_COLOR)
    mask = cv2.imread(str(d / "mask.png"), cv2.IMREAD_GRAYSCALE)
    base = np.load(d / "base_cloud.npy")
    candidates = np.load(d / "rim_candidates.npy")
    with (d / "result.json").open(encoding="utf-8") as f:
        result = json.load(f)
    if image is None or mask is None:
        raise RuntimeError("missing overlay or mask")
    # extract_object_points uses xyz[mask], then removes invalid depth. Rebuild
    # the same pixel order from the saved binary mask. The recorded cloud has
    # the same length, so each base-frame point can be located in the image.
    pixels = np.column_stack(np.nonzero(mask > 0))  # row, col
    if len(pixels) != len(base):
        raise RuntimeError(f"mask/cloud length mismatch: {len(pixels)} != {len(base)}")
    canvas = image.copy()
    # Candidate points: nearest saved cloud pixel, shown in cyan.
    cand_idx = np.unique(np.argmin(((base[:, None, :] - candidates[None, :, :]) ** 2).sum(2), axis=0))
    for r, c in pixels[cand_idx]:
        cv2.circle(canvas, (int(c), int(r)), 2, (255, 255, 0), -1, cv2.LINE_AA)
    point = np.asarray(result["rim_status"]["point"], dtype=float)
    idx = int(np.argmin(((base - point) ** 2).sum(1)))
    r, c = map(int, pixels[idx])
    cv2.circle(canvas, (c, r), 12, (0, 0, 255), 3, cv2.LINE_AA)
    cv2.drawMarker(canvas, (c, r), (0, 0, 255), cv2.MARKER_CROSS, 32, 3, cv2.LINE_AA)
    label = f"RIM ({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})"
    cv2.putText(canvas, label, (max(8, c - 220), max(28, r - 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2, cv2.LINE_AA)
    out = d / "overlay_rim.png"
    if not cv2.imwrite(str(out), canvas):
        raise RuntimeError("could_not_save_overlay_rim")
    print(out)


if __name__ == "__main__":
    main()

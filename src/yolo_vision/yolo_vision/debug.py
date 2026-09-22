"""Offline image + organized XYZ extraction, optionally with a recorded mask."""

import argparse
import json
from pathlib import Path
import numpy as np

from .config import VisionConfig, load_config
from .pointcloud import extract_object_points
from .segmentation import Segmenter


def main():
    parser = argparse.ArgumentParser(description="Debug vision without ROS; XYZ must share image pixel coordinates")
    parser.add_argument("--image", required=True)
    parser.add_argument("--xyz", required=True, help="HxWx3 .npy in camera optical frame")
    parser.add_argument("--mask", help="HxW boolean .npy; skips model inference")
    parser.add_argument("--model", help="OpenVINO directory, required unless --mask is supplied")
    parser.add_argument("--config")
    parser.add_argument("--output-dir", default="debug_output/vision")
    args = parser.parse_args()
    if not args.mask and not args.model:
        parser.error("--model is required without --mask")
    import cv2

    config = load_config(args.config) if args.config else VisionConfig()
    bgr = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if bgr is None:
        parser.error("image could not be read")
    xyz = np.load(args.xyz, allow_pickle=False)
    if xyz.shape != (*bgr.shape[:2], 3):
        parser.error("dimension_mismatch")
    result = None
    if args.mask:
        mask = np.load(args.mask, allow_pickle=False)
    else:
        result = Segmenter(args.model, config).predict(bgr)
        mask = result.mask
    points = extract_object_points(xyz, mask, config.min_points)
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    np.save(output / "mask.npy", mask)
    np.save(output / "object_points.npy", points)
    overlay = bgr.copy()
    overlay[mask] = (0.5 * overlay[mask] + np.array([0, 127, 0])).astype(np.uint8)
    if not cv2.imwrite(str(output / "overlay.png"), overlay):
        raise RuntimeError("could_not_save_overlay")
    summary = {"point_count": len(points), "shape": list(mask.shape),
               "class_name": result.class_name if result else None,
               "confidence": result.confidence if result else None}
    (output / "result.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Replay a saved observation through localization; write poses and RGB contacts.

No ROS initialization, publication, or execution. The image is diagnostic,
not independent measurement of contact accuracy.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from rim_locator.config import load_config
from rim_locator.rim_detection import locate_grasp_pose, transform_points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('observation', type=Path)
    parser.add_argument('--config', default='src/rim_locator/config/place_config.json')
    args = parser.parse_args()
    config = load_config(args.config)
    data = np.load(args.observation, allow_pickle=False)
    image = data['bgr'].copy()
    rotation = transform_points(np.eye(3), [0, 0, 0], data['quaternion']).T
    k = data['k'].reshape(3, 3)

    def pixel(point):
        camera = (point-data['translation']) @ rotation
        uv = k @ camera
        return tuple(np.rint(uv[:2]/uv[2]).astype(int))

    objects = []
    for key in sorted(k for k in data.files if k.startswith('points_')):
        index = int(key.split('_')[-1])
        try:
            result = locate_grasp_pose(data[key], config)
            objects.append(dict(object_id=index, point=result.point.tolist(), orientation=result.orientation,
                                normal=result.normal.tolist(), geometry=result.diagnostics))
            uv = pixel(result.point)
            cv2.drawMarker(image, uv, (0, 255, 255), cv2.MARKER_CROSS, 15, 2)
            cv2.line(image, uv, pixel(result.point + result.normal*.025), (0, 255, 255), 2)
            cv2.putText(image, f'id={index}', (uv[0]+6, uv[1]-6), cv2.FONT_HERSHEY_SIMPLEX,
                        .5, (0, 255, 255), 1, cv2.LINE_AA)
        except ValueError as exc:
            objects.append(dict(object_id=index, reason=str(exc), geometry=getattr(exc, 'diagnostics', {})))
    objects.sort(key=lambda o: np.hypot(*o['point'][:2]) if 'point' in o else float('inf'))
    report = dict(objects=objects, valid_count=sum('point' in o for o in objects))
    (args.observation.parent / 'localization.json').write_text(json.dumps(report, indent=2)+'\n')
    cv2.imwrite(str(args.observation.parent / 'contacts.png'), image)
    mask_keys = [k for k in data.files if k.startswith('mask_')]
    if mask_keys:
        yy, xx = np.nonzero(np.logical_or.reduce([data[k] for k in mask_keys]))
        if len(xx):
            crop = image[max(0, yy.min()-25):yy.max()+26, max(0, xx.min()-25):xx.max()+26]
            cv2.imwrite(str(args.observation.parent / 'contacts_detail.png'), cv2.resize(crop, None, fx=3, fy=3))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Read-only RGB/depth/TF capture for repeatable pre-grasp diagnosis.

Run with the same ROS and YOLO Python environment as run_yolo_vision.sh.
No publishers, robot commands, or controller changes are created.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from rim_locator.rim_detection import transform_points
from yolo_vision.config import load_config, resolve_model_path
from yolo_vision.pointcloud import read_aligned_depth_xyz
from yolo_vision.segmentation import Segmenter
from yolo_vision.sensor_input import raw_subscriber


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='src/yolo_vision/config/place_config.json')
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--timeout', type=float, default=90.)
    args = parser.parse_args()
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=False)
    config = load_config(args.config)
    if not config.depth_image_topic:
        raise ValueError('capture_requires_registered_depth_image')
    segmenter = Segmenter(resolve_model_path(config, get_package_share_directory('yolo_vision')), config)
    rclpy.init()
    node = Node('grasp_observation_recorder')
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    pairs = []
    errors = []
    subs = [raw_subscriber(node, Image, topic, qos_profile_sensor_data,
                           lambda: not pairs, errors.append)
            for topic in (config.rgb_topic, config.depth_image_topic)]
    subs.append(Subscriber(node, CameraInfo, config.depth_info_topic, qos_profile=qos_profile_sensor_data))
    sync = ApproximateTimeSynchronizer(subs, config.sync_queue_size, config.sync_slop)
    sync.registerCallback(lambda *messages: pairs.append(messages) if not pairs else None)
    deadline = time.monotonic() + args.timeout
    try:
        transform = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if errors:
                raise errors[0]
            if pairs:
                rgb, depth, info = pairs[0]
                try:
                    transform = buffer.lookup_transform('base_link', depth.header.frame_id,
                                                        Time.from_msg(depth.header.stamp)).transform
                    break
                except TransformException:
                    pass
        if transform is None:
            raise TimeoutError('capture_rgb_depth_tf_timeout')
        rgb, depth = rgb.decode(), depth.decode()
        stamps = [m.header.stamp.sec * 10**9 + m.header.stamp.nanosec for m in (rgb, depth, info)]
        if any(not -.1 <= (node.get_clock().now().nanoseconds-s)/1e9 <= config.max_source_age for s in stamps):
            raise ValueError('capture_stale_source')
        if rgb.header.frame_id != depth.header.frame_id or max(stamps)-min(stamps) > config.sync_slop*1e9:
            raise ValueError('capture_not_aligned_or_synchronized')
        bridge = CvBridge()
        bgr = bridge.imgmsg_to_cv2(rgb, 'bgr8')
        xyz = read_aligned_depth_xyz(depth, info)
        t, q = transform.translation, transform.rotation
        translation, rotation = [t.x, t.y, t.z], [q.x, q.y, q.z, q.w]
        results = segmenter.predict_all(bgr)
        arrays = dict(bgr=bgr, xyz=xyz.astype(np.float32), k=np.asarray(info.k),
                      translation=translation, quaternion=rotation)
        occupied = np.zeros(bgr.shape[:2], bool)
        objects = []
        for index, result in enumerate(results):
            mask = result.mask & ~occupied
            occupied |= result.mask
            valid = mask & np.isfinite(xyz).all(axis=2) & (xyz[:, :, 2] > 0)
            arrays[f'mask_{index}'] = mask
            arrays[f'pixels_{index}'] = np.column_stack(np.nonzero(valid))
            arrays[f'points_{index}'] = transform_points(xyz[valid], translation, rotation)
            objects.append(dict(object_id=index, confidence=result.confidence,
                                mask_pixels=int(mask.sum()), valid_points=int(valid.sum())))
        np.savez_compressed(output / 'observation.npz', **arrays)
        cv2.imwrite(str(output / 'rgb.png'), bgr)
        metadata = dict(config=asdict(config), stamps_ns=stamps, source_frame=depth.header.frame_id,
                        target_frame='base_link', translation=translation, quaternion=rotation,
                        objects=objects)
        (output / 'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
        print(json.dumps(metadata, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""Offline .npy analysis or a ROS publisher of saved/synthetic target points."""

import argparse
import json
from pathlib import Path
import time
import numpy as np

from .config import RimConfig, load_config
from .rim_detection import locate_rim, valid_points


def synthetic_points():
    """Upright open cylinder with a visible rim, already in base coordinates."""
    theta = np.linspace(0, 2 * np.pi, 240, endpoint=False)
    ring = np.column_stack((0.5 + 0.08 * np.cos(theta), 0.08 * np.sin(theta), np.full_like(theta, 0.3)))
    walls = np.concatenate([ring - [0, 0, h] for h in np.linspace(0.02, 0.15, 12)])
    return np.concatenate((ring, walls))


def offline_main():
    parser = argparse.ArgumentParser(description="Analyze base-frame XYZ without ROS")
    parser.add_argument("--points", help="Nx3 .npy in target_frame; otherwise use a synthetic cylinder")
    parser.add_argument("--config")
    parser.add_argument("--output-dir", default="debug_output/rim")
    args = parser.parse_args()
    config = load_config(args.config) if args.config else RimConfig()
    points = np.load(args.points, allow_pickle=False) if args.points else synthetic_points()
    result = locate_rim(points, config)
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    np.save(output / "base_points.npy", points)
    np.save(output / "rim_candidates.npy", result.candidates)
    np.save(output / "local_points.npy", result.local_points)
    summary = {"frame_id": config.target_frame, "rim_point": result.point.tolist(),
               "z_top": result.z_top, "candidate_count": len(result.candidates), "local_count": len(result.local_points)}
    (output / "result.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


def publish_main():
    import rclpy
    from rclpy.node import Node
    from rclpy.utilities import remove_ros_args
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header
    from .main import retained_qos
    from .pointcloud import make_cloud

    parser = argparse.ArgumentParser(description="Publish one object cloud and retain it until stopped")
    parser.add_argument("--points", help="Nx3 .npy; synthetic points are in base")
    parser.add_argument("--frame", default="base_link")
    parser.add_argument("--topic", default="/yolo_vision/object_cloud")
    args = parser.parse_args(remove_ros_args()[1:])
    if not args.points and args.frame != "base_link":
        parser.error("Synthetic points are in base_link. Supply --points for another frame")
    points = valid_points(np.load(args.points, allow_pickle=False) if args.points else synthetic_points())
    rclpy.init()
    node = Node("debug_object_cloud")
    publisher = node.create_publisher(PointCloud2, args.topic, retained_qos())
    try:
        deadline = time.monotonic() + 10.0
        while node.get_clock().now().nanoseconds <= 0:
            if time.monotonic() > deadline:
                raise RuntimeError("clock_unavailable")
            rclpy.spin_once(node, timeout_sec=0.1)
        header = Header(frame_id=args.frame, stamp=node.get_clock().now().to_msg())
        publisher.publish(make_cloud(header, points))
        node.get_logger().info("Published one cloud. File coordinates must match --frame; no transform is applied.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    offline_main()

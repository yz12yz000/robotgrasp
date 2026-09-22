"""Offline pose preview and a retained ROS target publisher."""

import argparse
from dataclasses import asdict
import json
import math
import time

from .config import GraspConfig, load_config
from .grasp import build_approach_pose, build_grasp_pose


def offline_main():
    parser = argparse.ArgumentParser(description="Preview target poses only; never publish or move hardware")
    parser.add_argument("--point", nargs=3, type=float, default=[0.518, -0.162, 0.824])
    parser.add_argument("--config")
    args = parser.parse_args()
    config = load_config(args.config) if args.config else GraspConfig()
    grasp = build_grasp_pose(args.point, config)
    approach = build_approach_pose(grasp, config)
    print(json.dumps({"mode": "pose_preview_only", "approach_pose": asdict(approach),
                      "grasp_pose": asdict(grasp)}, indent=2))
    return 0



def publish_main():
    import rclpy
    from geometry_msgs.msg import PointStamped
    from rclpy.node import Node
    from rclpy.utilities import remove_ros_args
    from .main import retained_qos

    parser = argparse.ArgumentParser(description="Publish one fresh PointStamped and retain it until stopped")
    parser.add_argument("--point", nargs=3, type=float, default=[0.518, -0.162, 0.824])
    parser.add_argument("--frame", default="base_link")
    parser.add_argument("--topic", default="/rim_locator/rim_point")
    args = parser.parse_args(remove_ros_args()[1:])
    if not all(math.isfinite(x) for x in args.point):
        parser.error("point must be finite")
    rclpy.init()
    node = Node("debug_rim_point")
    publisher = node.create_publisher(PointStamped, args.topic, retained_qos())
    try:
        # With use_sim_time wait for /clock rather than publish a zero timestamp.
        deadline = time.monotonic() + 10.0
        while node.get_clock().now().nanoseconds <= 0:
            if time.monotonic() > deadline:
                raise RuntimeError("clock_unavailable")
            rclpy.spin_once(node, timeout_sec=0.1)
        message = PointStamped()
        message.header.frame_id = args.frame
        message.header.stamp = node.get_clock().now().to_msg()
        message.point.x, message.point.y, message.point.z = args.point
        publisher.publish(message)
        node.get_logger().info("Published one target; keep this publisher alive for late subscribers.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(offline_main())

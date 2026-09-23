#!/usr/bin/env python3
"""Remove retired grasp-table/envelope objects; sends no arm/gripper commands."""
import sys
import time
from pathlib import Path

import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/grasp_executor'))
from grasp_executor.scene_cleanup import clear_legacy_objects


def main():
    rclpy.init()
    node = rclpy.create_node('clear_legacy_grasp_scene')

    def call(service, name, request, deadline):
        client = node.create_client(service, name)
        try:
            if not client.wait_for_service(timeout_sec=max(0., deadline - time.monotonic())):
                raise TimeoutError(name)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=max(0., deadline - time.monotonic()))
            if not future.done():
                raise TimeoutError(name)
            return future.result()
        finally:
            node.destroy_client(client)

    try:
        clear_legacy_objects(call, time.monotonic() + 30.)
        print('场景已确认：无旧桌面和额外夹爪保护盒；未发送运动指令。')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

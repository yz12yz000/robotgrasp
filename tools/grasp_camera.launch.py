"""Camera settings validated with the grasp pipeline; no robot control."""
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


CAMERA_PARAMETERS = {
    'camera_name': 'camera',
    'enable_color': True, 'enable_depth': True, 'enable_ir': False,
    'color_width': 1280, 'color_height': 800, 'color_fps': 15,
    'depth_width': 1280, 'depth_height': 800, 'depth_fps': 15,
    'color_format': 'MJPG', 'depth_format': 'Y14',
    'depth_registration': True, 'align_mode': 'HW', 'ordered_pc': True,
    'enable_frame_sync': True, 'enable_point_cloud': True,
    'enable_colored_point_cloud': False,
    'color_qos': 'SENSOR_DATA', 'depth_qos': 'SENSOR_DATA', 'point_cloud_qos': 'SENSOR_DATA',
    # SDK acquisition system timestamp, not a timestamp rewritten by YOLO.
    'time_domain': 'system', 'enable_sync_host_time': False,
}


def generate_launch_description():
    return LaunchDescription([ComposableNodeContainer(
        name='camera_container', namespace='/camera',
        package='rclcpp_components', executable='component_container', output='screen',
        composable_node_descriptions=[ComposableNode(
            package='orbbec_camera', plugin='orbbec_camera::OBCameraNodeDriver',
            name='camera', namespace='/camera', parameters=[CAMERA_PARAMETERS],
        )],
    )])

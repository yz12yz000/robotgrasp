from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=PathJoinSubstitution([
            FindPackageShare('yolo_vision'), 'config', 'config.json',
        ])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('task_id', default_value=''),
        DeclareLaunchArgument('observation_id', default_value=''),
        DeclareLaunchArgument('source_not_before_ns', default_value='0'),
        Node(package='yolo_vision', executable='yolo_vision', output='screen',
             namespace=LaunchConfiguration('namespace'), parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'task_id': ParameterValue(LaunchConfiguration('task_id'), value_type=str),
            'observation_id': ParameterValue(LaunchConfiguration('observation_id'), value_type=str),
            'source_not_before_ns': ParameterValue(LaunchConfiguration('source_not_before_ns'), value_type=int),
        }]),
    ])

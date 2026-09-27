from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=PathJoinSubstitution([
            FindPackageShare('rim_locator'), 'config', 'config.json',
        ])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('task_id', default_value=''),
        DeclareLaunchArgument('observation_id', default_value=''),
        Node(package='rim_locator', executable='rim_locator', output='screen',
             namespace=LaunchConfiguration('namespace'), parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'task_id': ParameterValue(LaunchConfiguration('task_id'), value_type=str),
            'observation_id': ParameterValue(LaunchConfiguration('observation_id'), value_type=str),
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }]),
    ])

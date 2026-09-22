from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=PathJoinSubstitution([
            FindPackageShare('grasp_executor'), 'config', 'config.json',
        ])),
        DeclareLaunchArgument('plan_only', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='grasp_executor', executable='grasp_executor', output='screen', parameters=[{
            'plan_only': ParameterValue(LaunchConfiguration('plan_only'), value_type=bool),
            'config_path': LaunchConfiguration('config_path'),
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }]),
    ])

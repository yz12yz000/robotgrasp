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
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('task_id', default_value=''),
        DeclareLaunchArgument('observation_id', default_value=''),
        DeclareLaunchArgument('source_not_before_ns', default_value='0'),
        DeclareLaunchArgument('task_deadline_monotonic_ns', default_value='0'),
        DeclareLaunchArgument('supervisor_pid', default_value='0'),
        DeclareLaunchArgument('supervisor_start_ticks', default_value='0'),
        DeclareLaunchArgument('task_preview_topic', default_value='/display_planned_path'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='grasp_executor', executable='grasp_executor', output='screen',
             namespace=LaunchConfiguration('namespace'), parameters=[{
            'task_id': ParameterValue(LaunchConfiguration('task_id'), value_type=str),
            'observation_id': ParameterValue(LaunchConfiguration('observation_id'), value_type=str),
            'source_not_before_ns': ParameterValue(LaunchConfiguration('source_not_before_ns'), value_type=int),
            'task_deadline_monotonic_ns': ParameterValue(LaunchConfiguration('task_deadline_monotonic_ns'), value_type=int),
            'supervisor_pid': ParameterValue(LaunchConfiguration('supervisor_pid'), value_type=int),
            'supervisor_start_ticks': ParameterValue(LaunchConfiguration('supervisor_start_ticks'), value_type=int),
            'task_preview_topic': ParameterValue(LaunchConfiguration('task_preview_topic'), value_type=str),
            'plan_only': ParameterValue(LaunchConfiguration('plan_only'), value_type=bool),
            'config_path': LaunchConfiguration('config_path'),
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }]),
    ])

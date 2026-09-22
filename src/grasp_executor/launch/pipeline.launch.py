"""Compose the three independent nodes. The MoveIt executor defaults to planning only."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    actions = [
        DeclareLaunchArgument("plan_only", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("enable_grasp", default_value="false"),
    ]
    for package in ("rim_locator", "grasp_executor", "yolo_vision"):
        key = package + "_config"
        actions.append(DeclareLaunchArgument(key, default_value=PathJoinSubstitution([
            FindPackageShare(package), "config", "config.json",
        ])))
        options = {}
        if package == "grasp_executor":
            options["condition"] = IfCondition(LaunchConfiguration("enable_grasp"))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare(package), "launch", package + ".launch.py",
            ])),
            launch_arguments={
                **({"plan_only": LaunchConfiguration("plan_only")} if package == "grasp_executor" else {}),
                "config_path": LaunchConfiguration(key),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
            **options,
        ))
    return LaunchDescription(actions)

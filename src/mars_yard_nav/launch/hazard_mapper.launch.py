"""Standalone launch file for the Mars Yard step-height hazard_mapper node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory("mars_yard_nav")
    default_params = os.path.join(nav_share, "config", "hazard_mapper.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock simulation time.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to hazard_mapper parameter YAML.",
            ),
            Node(
                package="mars_yard_nav",
                executable="hazard_mapper.py",
                name="hazard_mapper",
                output="both",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )

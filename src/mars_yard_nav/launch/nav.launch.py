"""Launch Nav2 for the Mars Yard rover with pluggable planner and controller overlays.

No AMCL is launched: map -> odom remains the static identity TF published by
sim.launch.py. Base parameters in nav2_base.yaml are deep-merged with the
selected planner_*.yaml and controller_*.yaml overlays and the world's static
map YAML (slope.yaml for mars, flat.yaml for flat).
"""

import copy
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


VALID_WORLDS = ("flat", "mars")
VALID_PLANNERS = ("navfn", "smac_hybrid")
VALID_CONTROLLERS = ("dwb", "mppi", "rpp")


def _deep_merge(base: dict, overlay: dict) -> dict:
    """Recursively merge overlay dict into a deep copy of base."""
    out = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ("true", "1", "yes")


def _launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    planner = LaunchConfiguration("planner").perform(context)
    controller = LaunchConfiguration("controller").perform(context)
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    autostart = _as_bool(LaunchConfiguration("autostart").perform(context))
    log_level = LaunchConfiguration("log_level").perform(context)

    if world not in VALID_WORLDS:
        raise ValueError(f"Unsupported world '{world}'. Expected one of {VALID_WORLDS}.")
    if planner not in VALID_PLANNERS:
        raise ValueError(f"Unsupported planner '{planner}'. Expected one of {VALID_PLANNERS}.")
    if controller not in VALID_CONTROLLERS:
        raise ValueError(f"Unsupported controller '{controller}'. Expected one of {VALID_CONTROLLERS}.")

    nav_share = get_package_share_directory("mars_yard_nav")
    worlds_share = get_package_share_directory("mars_yard_worlds")

    base_yaml_path = os.path.join(nav_share, "config", "nav2_base.yaml")
    planner_yaml_path = os.path.join(nav_share, "config", f"planner_{planner}.yaml")
    controller_yaml_path = os.path.join(nav_share, "config", f"controller_{controller}.yaml")
    map_yaml_path = os.path.join(
        worlds_share, "terrain", "slope.yaml" if world == "mars" else "flat.yaml"
    )

    with open(base_yaml_path, "r", encoding="utf-8") as f:
        merged = yaml.safe_load(f)
    with open(planner_yaml_path, "r", encoding="utf-8") as f:
        merged = _deep_merge(merged, yaml.safe_load(f) or {})
    with open(controller_yaml_path, "r", encoding="utf-8") as f:
        merged = _deep_merge(merged, yaml.safe_load(f) or {})

    merged.setdefault("map_server", {}).setdefault("ros__parameters", {})[
        "yaml_filename"
    ] = map_yaml_path

    # Ensure use_sim_time propagates uniformly to every node section.
    for section in (
        "map_server",
        "bt_navigator",
        "planner_server",
        "controller_server",
        "behavior_server",
        "velocity_smoother",
    ):
        merged.setdefault(section, {}).setdefault("ros__parameters", {})[
            "use_sim_time"
        ] = use_sim_time
    for cm_section in ("global_costmap", "local_costmap"):
        merged.setdefault(cm_section, {}).setdefault(cm_section, {}).setdefault(
            "ros__parameters", {}
        )["use_sim_time"] = use_sim_time

    tmp = tempfile.NamedTemporaryFile(
        mode="w",
        prefix=f"mars_yard_nav2_{world}_{planner}_{controller}_",
        suffix=".yaml",
        delete=False,
        encoding="utf-8",
    )
    yaml.safe_dump(merged, tmp, sort_keys=False)
    tmp.close()
    params_file = tmp.name

    lifecycle_nodes = [
        "map_server",
        "planner_server",
        "controller_server",
        "behavior_server",
        "bt_navigator",
        "velocity_smoother",
    ]

    log_args = ["--ros-args", "--log-level", log_level]

    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params_file],
            arguments=log_args,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file],
            arguments=log_args,
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file],
            arguments=log_args,
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params_file],
            arguments=log_args,
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params_file],
            arguments=log_args,
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[params_file],
            arguments=log_args,
            remappings=[
                ("cmd_vel", "cmd_vel_nav"),
                ("cmd_vel_smoothed", "/cmd_vel"),
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            arguments=log_args,
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "bond_timeout": 10.0,
                    "node_names": lifecycle_nodes,
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="mars",
                description="World map to load: 'flat' or 'mars'.",
            ),
            DeclareLaunchArgument(
                "planner",
                default_value="smac_hybrid",
                description="Global planner overlay: 'navfn' or 'smac_hybrid'.",
            ),
            DeclareLaunchArgument(
                "controller",
                default_value="dwb",
                description="Local controller overlay: 'dwb', 'mppi', or 'rpp'.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use /clock simulation time.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically transition Nav2 lifecycle nodes to active.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Nav2 node ROS log level.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

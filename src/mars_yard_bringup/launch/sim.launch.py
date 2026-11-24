"""Launch the Mars Yard rover in MuJoCo with ros2_control, EKF, static map->odom TF, and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ("true", "1", "yes")


def _launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    if world not in ("flat", "mars"):
        raise ValueError(f"Unsupported world '{world}'. Expected 'flat' or 'mars'.")

    headless = LaunchConfiguration("headless").perform(context)
    use_rviz = _as_bool(LaunchConfiguration("rviz").perform(context))
    use_ekf = _as_bool(LaunchConfiguration("use_ekf").perform(context))

    bringup_share = get_package_share_directory("mars_yard_bringup")
    description_share = get_package_share_directory("mars_yard_description")
    worlds_share = get_package_share_directory("mars_yard_worlds")

    urdf_path = os.path.join(description_share, "urdf", "rover.urdf.xacro")
    mujoco_model_path = os.path.join(worlds_share, "worlds", f"{world}.xml")
    controllers_yaml = os.path.join(bringup_share, "config", "controllers.yaml")
    mujoco_plugins_yaml = os.path.join(bringup_share, "config", "mujoco_plugins.yaml")
    ekf_yaml = os.path.join(bringup_share, "config", "ekf.yaml")
    rviz_config = os.path.join(bringup_share, "rviz", "sim.rviz")

    robot_description_str = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            urdf_path,
            " mujoco_model:=",
            mujoco_model_path,
            " headless:=",
            headless,
        ]
    ).perform(context)
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_str, value_type=str)
    }

    nodes = [
        # Publishes URDF kinematics and /robot_description
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        ),
        # MuJoCo physics + ros2_control hardware interface + plugins (RayScanPlugin)
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            emulate_tty=True,
            output="both",
            parameters=[
                {"use_sim_time": True},
                ParameterFile(controllers_yaml),
                ParameterFile(mujoco_plugins_yaml),
            ],
            on_exit=Shutdown(),
        ),
        # Bridge public Twist /cmd_vel -> TwistStamped /diff_drive_controller/cmd_vel
        Node(
            package="mars_yard_bringup",
            executable="twist_stamper.py",
            name="twist_stamper",
            output="both",
            parameters=[
                {
                    "use_sim_time": True,
                    "in_topic": "/cmd_vel",
                    "out_topic": "/diff_drive_controller/cmd_vel",
                    "frame_id": "base_link",
                }
            ],
        ),
        # Identity map -> odom TF (replaced by SLAM/AMCL in later phases)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_map_to_odom",
            output="both",
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "map",
                "--child-frame-id", "odom",
            ],
            parameters=[{"use_sim_time": True}],
        ),
    ]

    for controller in (
        "joint_state_broadcaster",
        "imu_sensor_broadcaster",
        "diff_drive_controller",
    ):
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--controller-manager-timeout", "30"],
                output="both",
                parameters=[{"use_sim_time": True}],
            )
        )

    if use_ekf:
        nodes.append(
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="both",
                parameters=[ekf_yaml, {"use_sim_time": True}],
            )
        )

    if use_rviz:
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="both",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="mars",
                description="MuJoCo world to load: 'flat' or 'mars'.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run MuJoCo without the interactive viewer window.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Launch RViz2 with the rover visualization config.",
            ),
            DeclareLaunchArgument(
                "use_ekf",
                default_value="true",
                description="Launch robot_localization ekf_node for odom -> base_link fusion.",
            ),
            # Placeholders reserved for later navigation and domain-randomization phases
            DeclareLaunchArgument(
                "planner",
                default_value="",
                description="Placeholder (unused in Phase 3): Nav2 global planner selection.",
            ),
            DeclareLaunchArgument(
                "controller",
                default_value="",
                description="Placeholder (unused in Phase 3): Nav2 local controller selection.",
            ),
            DeclareLaunchArgument(
                "randomization",
                default_value="",
                description="Placeholder (unused in Phase 3): terrain/physics randomization profile.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

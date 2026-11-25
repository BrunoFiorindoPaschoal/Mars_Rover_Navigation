"""Launch the Mars Yard rover in MuJoCo with ros2_control, EKF, static map->odom TF, and RViz."""

import importlib.util
import os
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ("true", "1", "yes")


def _load_randomizer_module():
    candidates = [
        pathlib.Path(os.path.realpath(__file__)).parents[3] / "tools" / "randomize_world.py",
        pathlib.Path.cwd() / "tools" / "randomize_world.py",
    ]
    for path in candidates:
        if path.is_file():
            spec = importlib.util.spec_from_file_location("mars_yard_randomize_world", str(path))
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            return mod
    raise FileNotFoundError(
        f"Could not locate tools/randomize_world.py in {[str(c) for c in candidates]}"
    )


def _launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    if world not in ("flat", "mars"):
        raise ValueError(f"Unsupported world '{world}'. Expected 'flat' or 'mars'.")

    headless = LaunchConfiguration("headless").perform(context)
    use_rviz = _as_bool(LaunchConfiguration("rviz").perform(context))
    use_ekf = _as_bool(LaunchConfiguration("use_ekf").perform(context))
    use_metrics = _as_bool(LaunchConfiguration("metrics").perform(context))
    use_hazards = _as_bool(LaunchConfiguration("hazards").perform(context))
    use_nav = _as_bool(LaunchConfiguration("nav").perform(context))
    nav_planner = LaunchConfiguration("planner").perform(context)
    nav_controller = LaunchConfiguration("controller").perform(context)
    rand_level = int(LaunchConfiguration("rand_level").perform(context))
    rand_seed = int(LaunchConfiguration("rand_seed").perform(context))
    rand_out_dir = LaunchConfiguration("rand_out_dir").perform(context).strip()

    if rand_level not in (0, 1, 2, 3):
        raise ValueError(f"Unsupported rand_level '{rand_level}'. Expected 0, 1, 2, or 3.")

    bringup_share = get_package_share_directory("mars_yard_bringup")
    description_share = get_package_share_directory("mars_yard_description")
    worlds_share = get_package_share_directory("mars_yard_worlds")
    nav_share = get_package_share_directory("mars_yard_nav")

    urdf_path = os.path.join(description_share, "urdf", "rover.urdf.xacro")
    mujoco_model_path = os.path.join(worlds_share, "worlds", f"{world}.xml")
    controllers_yaml = os.path.join(bringup_share, "config", "controllers.yaml")
    mujoco_plugins_yaml = os.path.join(bringup_share, "config", "mujoco_plugins.yaml")
    ekf_yaml = os.path.join(bringup_share, "config", "ekf.yaml")
    hazard_mapper_yaml = os.path.join(nav_share, "config", "hazard_mapper.yaml")
    rviz_config = os.path.join(bringup_share, "rviz", "sim.rviz")

    rand_params = None
    if rand_level > 0:
        out_dir = rand_out_dir or f"/tmp/mars_yard_rand_{world}_L{rand_level}_S{rand_seed}"
        rand_mod = _load_randomizer_module()
        world_xml_path, rand_params = rand_mod.randomize_world(
            world=world,
            level=rand_level,
            seed=rand_seed,
            out_dir=pathlib.Path(out_dir),
            worlds_dir=pathlib.Path(worlds_share),
            description_dir=pathlib.Path(description_share),
        )
        mujoco_model_path = str(world_xml_path)

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

    twist_stamper_params = {
        "use_sim_time": True,
        "in_topic": "/cmd_vel",
        "out_topic": "/diff_drive_controller/cmd_vel",
        "frame_id": "base_link",
    }
    if rand_level > 0:
        twist_stamper_params["latency_s"] = float(rand_params["cmd_latency_s"])

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
            parameters=[twist_stamper_params],
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

    if rand_level > 0:
        nodes.append(
            Node(
                package="mars_yard_bringup",
                executable="noise_relay.py",
                name="noise_relay",
                output="both",
                parameters=[
                    {
                        "use_sim_time": True,
                        "points_in": "/points",
                        "points_out": "/points_noisy",
                        "imu_in": "/imu_sensor_broadcaster/imu",
                        "imu_out": "/imu_sensor_broadcaster/imu_noisy",
                        "lidar_noise_sigma": float(rand_params["lidar_noise_sigma_m"]),
                        "lidar_dropout": float(rand_params["lidar_dropout"]),
                        "imu_gyro_sigma": float(rand_params["imu_gyro_sigma_rad_s"]),
                        "imu_gyro_z_bias": float(rand_params["imu_gyro_z_bias_rad_s"]),
                        "lidar_seed": int(rand_params["lidar_relay_seed"]),
                        "imu_seed": int(rand_params["imu_relay_seed"]),
                    }
                ],
            )
        )

    if use_ekf:
        ekf_remappings = (
            [("/imu_sensor_broadcaster/imu", "/imu_sensor_broadcaster/imu_noisy")]
            if rand_level > 0
            else []
        )
        nodes.append(
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="both",
                parameters=[ekf_yaml, {"use_sim_time": True}],
                remappings=ekf_remappings,
            )
        )

    if use_metrics:
        nodes.append(
            Node(
                package="mars_yard_metrics",
                executable="metrics_node",
                name="metrics_node",
                output="both",
                parameters=[{"use_sim_time": True}],
            )
        )

    if use_hazards:
        hazard_remappings = (
            [
                ("/points", "/points_noisy"),
                ("/imu_sensor_broadcaster/imu", "/imu_sensor_broadcaster/imu_noisy"),
            ]
            if rand_level > 0
            else []
        )
        nodes.append(
            Node(
                package="mars_yard_nav",
                executable="hazard_mapper.py",
                name="hazard_mapper",
                output="both",
                parameters=[hazard_mapper_yaml, {"use_sim_time": True}],
                remappings=hazard_remappings,
            )
        )

    if use_nav:
        nav_launch_path = os.path.join(nav_share, "launch", "nav.launch.py")
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch_path),
                launch_arguments={
                    "world": world,
                    "planner": nav_planner,
                    "controller": nav_controller,
                    "use_sim_time": "true",
                }.items(),
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
            DeclareLaunchArgument(
                "metrics",
                default_value="true",
                description="Launch mars_yard_metrics metrics_node for ground-truth and navigation metrics.",
            ),
            DeclareLaunchArgument(
                "hazards",
                default_value="true",
                description="Launch mars_yard_nav hazard_mapper for 3D step-height hazard detection.",
            ),
            DeclareLaunchArgument(
                "nav",
                default_value="false",
                description="Launch the Nav2 stack (map_server, planner, controller, BT navigator).",
            ),
            DeclareLaunchArgument(
                "planner",
                default_value="smac_hybrid",
                description="Nav2 global planner overlay: 'navfn' or 'smac_hybrid'.",
            ),
            DeclareLaunchArgument(
                "controller",
                default_value="dwb",
                description="Nav2 local controller overlay: 'dwb', 'mppi', or 'rpp'.",
            ),
            DeclareLaunchArgument(
                "rand_level",
                default_value="0",
                description="Domain randomization level: 0 (nominal), 1, 2, or 3.",
            ),
            DeclareLaunchArgument(
                "rand_seed",
                default_value="0",
                description="RNG seed for domain randomization when rand_level > 0.",
            ),
            DeclareLaunchArgument(
                "rand_out_dir",
                default_value="",
                description="Optional directory for generated randomized MJCF and params.json.",
            ),
            DeclareLaunchArgument(
                "randomization",
                default_value="",
                description="Deprecated placeholder kept for backwards compatibility; use rand_level and rand_seed.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

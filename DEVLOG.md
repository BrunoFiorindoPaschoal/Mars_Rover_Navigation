# Dev log

Chronological record of coding and architecture decisions. Newest entry at the bottom.
Format per entry: **Decision**, **Alternatives**, **Why**, **Consequences / watch**.
Failures and tuning steps are logged too, with what changed and what happened.

---

## 2026-11-24 · Phase 0 · Repo lives in WSL ext4, not on the Windows drive

- **Decision:** the repo is `~/mars-yard-mujoco` inside WSL Ubuntu 24.04. Open it from Windows through `\\wsl.localhost\Ubuntu\home\bhfpa\mars-yard-mujoco` or VS Code's "WSL: Open Folder".
- **Alternatives:** `C:\...\ROS_Mujoco`, mounted in WSL as `/mnt/c/...`.
- **Why:** `/mnt/c` goes through the 9P file bridge, which is 5–20× slower for the thousands of small files colcon touches. It also loses Linux permissions, so execute bits and symlinks break `install/`.
- **Watch:** the Windows folder keeps only the original spec. Blender (on Windows) reads exported USD files through the `\\wsl.localhost` path.

## 2026-11-24 · Phase 0 · One idempotent setup script instead of a list of manual steps

- **Decision:** `tools/setup_wsl.sh` installs everything: the ROS apt source, ROS 2 Jazzy desktop, Nav2, robot_localization, ros2_control and its controllers, mujoco_ros2_control, xacro and MCAP. It also creates a Python venv.
- **Alternatives:** follow the install docs by hand, or start straight away with Docker.
- **Why:** a script is reproducible, and you can rerun it after a WSL reset. Docker comes later (Phase 8), but its Dockerfile will reuse this same package list. Running the dev loop inside Docker on WSLg makes GUI forwarding harder.
- **Watch:** the venv uses `--system-site-packages`. That lets tools in `tools/` import both `mujoco` (pip) and `rclpy` (apt). The ROS nodes themselves run on the system Python, so they don't depend on the venv.

## 2026-11-24 · Phase 0 · Bridge = official `mujoco_ros2_control` (apt), not a custom node

- **Decision:** MuJoCo plugs into ROS 2 as a ros2_control *hardware interface*, using the ros-controls package.
- **Alternatives:** (a) a hand-written rclpy node that steps MuJoCo and publishes topics; (b) Gazebo.
- **Why:** with ros2_control, everything above the hardware line is the same code as on a real rover: `diff_drive_controller`, `joint_state_broadcaster` and the controller manager. Only the hardware plugin changes. This is the exact skill the Siemens role asks for. A custom node would mean rewriting the controllers.
- **Watch:** the package is young. If its lidar plugin fails, the fallback is MuJoCo rangefinders published as a `LaserScan` by a small node (see spec, Risks).

## 2026-11-24 · Phase 0 · Pin Python `mujoco` to the bridge's MuJoCo version (3.12.0)

- **Decision:** the venv installs `mujoco==3.12.0`, the same version that `ros-jazzy-mujoco-vendor` links against (`/opt/ros/jazzy/opt/mujoco_vendor/lib/libmujoco.so.3.12.0`).
- **Alternatives:** the latest pip release (4.0.1 at install time).
- **Why:** the same MJCF file is loaded by two programs: Python tools (drive tests, rendering, USD export) and the C++ bridge. If their versions differ, a model can parse in one and fail in the other, or behave differently, because MJCF attributes and defaults change between major versions.
- **Watch:** when the apt `mujoco_vendor` is upgraded, bump the pin in `tools/setup_wsl.sh` too.

## 2026-11-24 · Phase 0 · Install verified, and what the bridge offers

- **Verified:** turtlesim runs through WSLg. `mujoco_ros2_control_demos 01_basic_robot` loads `MujocoSystemInterface` and activates its controllers.
- **Finding (changes the plan):** the bridge ships plugins: `free_joint_state_publisher` (the true pose of free joints), `pose_broadcaster`, `rangefinder_lidar_plugin` and `mujoco_camera_plugin`.
  - Consequence for Phase 4: ground truth comes from `free_joint_state_publisher` / `pose_broadcaster`, so the C++ node does not have to touch MuJoCo internals.
  - Consequence for Phase 1: the lidar is a fan of MuJoCo `rangefinder` sensors, which the plugin turns into a `LaserScan`.
- **Known warning, harmless:** `Could not enable FIFO RT scheduling`. WSL has no realtime kernel, and simulation doesn't need one.
- **Known pip conflict, harmless for now:** the system `python3-scipy` (apt) requires numpy<1.28, while the venv has numpy 2.x. Only venv code that imports scipy would break, and none does. If scipy is ever needed, add `pip install scipy` to the venv.

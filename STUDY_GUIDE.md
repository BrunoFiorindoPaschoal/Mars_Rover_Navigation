# Study guide

What to study to understand this project fully, **most important first**. Each part lists where it sits in the data flow, the files to read, the concepts, self-check questions and one experiment.
This guide is updated at the end of every phase. Parts marked *(planned)* don't exist in code yet.

## 0. Architecture overview (read this first)

```
            +------------------- ROS 2 Jazzy -------------------+
  RViz2 <---|  Nav2 (planner, controller, costmaps, BT)          |
            |     | /cmd_vel                 ^ odom->base_link  |
            |     v                          |                  |
            |  diff_drive_controller --> EKF (robot_localization)|
            |     | wheel vel cmds     ^ /odom   ^ /imu         |
            |  controller_manager (ros2_control)                |
            |     |                                             |
            |  mujoco_ros2_control  (hardware interface)  ------+--> /scan, /imu
            +-----|---------------------------------------------+
                  v
              MuJoCo physics (rover MJCF + Jezero heightfield)
                  |
                  +--> true pose --> mars_yard_metrics (C++) --> /ground_truth/odom, metrics
```

tf tree: `map -> odom -> base_link -> {lidar_link, imu_link, wheel_*}`
- `map -> odom`: static at first (no AMCL)
- `odom -> base_link`: the EKF
- `base_link -> *`: robot_state_publisher, from the URDF

## Core parts (ranked)

| # | Part | Why it matters | Status |
|---|------|----------------|--------|
| 1 | ros2_control + mujoco_ros2_control | The integration the project exists for | planned (Phase 3) |
| 2 | tf2 tree + EKF | Every node depends on correct frames | planned (Phase 3) |
| 3 | Nav2 config: costmaps, planners, controllers | The DLR role | planned (Phase 5) |
| 4 | Rover MJCF/URDF + contact physics | Explains slip and jitter, and every number in the model | planned (Phase 1) |
| 5 | Terrain pipeline (DEM -> hfield + slope map) | Planning on orbital data | planned (Phase 2) |
| 6 | C++ metrics node | rclcpp, CMake, tf2 lookups | planned (Phase 4) |
| 7 | Benchmark runner + analysis | Evaluation protocol | planned (Phase 6) |
| 8 | Docker + CI | Reproducibility | planned (Phase 8) |

Sections for each part are filled in as the code lands.

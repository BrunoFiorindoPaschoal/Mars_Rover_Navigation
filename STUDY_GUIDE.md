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
| 1 | ros2_control + mujoco_ros2_control | The integration the project exists for | **done**, see Part 1 |
| 2 | tf2 tree + EKF | Every node depends on correct frames | **done**, see Part 2 |
| 3a | 3D lidar + step-height hazard detection | Local rock/boulder perception on slopes | **done**, see Part 3a |
| 3b | Nav2 config: costmaps, planners, controllers | The DLR role | planned (Phase 5b) |
| 4 | Rover MJCF/URDF + contact physics | Explains slip and jitter, and every number in the model | **done**, see Part 4 |
| 5 | Terrain pipeline (DEM -> hfield + slope map) | Planning on orbital data | **done**, see Part 5 |
| 6 | C++ metrics node | rclcpp, CMake, gtest, metric definitions | **done**, see Part 6 |
| 7 | Benchmark runner + analysis | Evaluation protocol | planned (Phase 6) |
| 8 | Docker + CI | Reproducibility | planned (Phase 8) |

Sections for each part are filled in as the code lands.

---

## Part 1 · ros2_control + mujoco_ros2_control: from `/cmd_vel` to wheel torque  *(Phase 3, done)*

**Where it sits:** between every ROS node and the physics. Nav2 (or teleop) sends a body velocity, and MuJoCo needs one number per actuator. ros2_control is the standard layer in between, and on a real rover only the hardware plugin would change.

**The chain, one control tick (100 Hz):**
1. `/cmd_vel` (Twist: v, ω) reaches `diff_drive_controller`.
2. Inverse kinematics: `ω_L = (v − ω·s·m/2)/r`, `ω_R = (v + ω·s·m/2)/r`, where s is the wheel separation (0.64), m is `wheel_separation_multiplier` (2.94) and r is the radius (0.12). Both wheels on one side get the same value.
3. The controller writes these into the joints' **velocity command interfaces**.
4. `MujocoSystemInterface::write()` copies each command into `data->ctrl` of the MJCF actuator with the same name. That actuator is a `<velocity kv=50>` servo, a stand-in for a motor and its speed loop.
5. MuJoCo steps (2 ms). Contact with the ground turns wheel torque into motion, with slip.
6. `read()` copies `qpos`/`qvel` back into the **state interfaces**, and the IMU sensor data into the sensor interfaces.
7. The broadcasters publish them: `/joint_states` and `/imu_sensor_broadcaster/imu`. diff_drive integrates the wheel speeds into `/diff_drive_controller/odom`.

**Read, in this order:**
1. The `<ros2_control>` block at the end of `src/mars_yard_description/urdf/rover.urdf.xacro`: which joints have command interfaces and which only have state interfaces, and why `front_axle_joint` is there at all.
2. `src/mars_yard_bringup/config/controllers.yaml`: the controller types and every diff_drive parameter.
3. `src/mars_yard_bringup/config/mujoco_plugins.yaml` and `src/mars_yard_sim_plugins/src/ray_scan_plugin.cpp`: our own lidar. It is a mujoco_ros2_control *plugin* (it runs inside the sim loop with direct access to `mjData`), not a ros2_control controller. It casts 360 rays with one `mj_multiRay` call at 10 Hz of sim time and publishes `LaserScan` with REP 117 `±inf`.
4. `src/mars_yard_bringup/launch/sim.launch.py`: who starts what. robot_description goes to *both* robot_state_publisher and ros2_control_node.

**Concepts to know:**
- **Hardware interface vs controller vs broadcaster.** The hardware interface only exposes numbers. Controllers *claim* command interfaces (only one controller per interface). Broadcasters only read.
- **Controller lifecycle:** unconfigured → inactive → active. The `spawner` loads a controller, configures it and activates it.
- **Why `wheel_separation_multiplier`?** A skid-steer rover turns by scraping its wheels sideways, so it turns slower than an ideal diff drive with the same wheel speeds. We measured an efficiency of 0.34 (Phase 1). Pretending the track is 1/0.34 times wider makes the controller command proportionally more wheel-speed difference, and the odometry integrates turns at the right rate. The value is right on flat ground only. On sand or slopes it changes, and the EKF with IMU yaw has to absorb the difference.
- **Performance is part of the design (DEVLOG, Phase 3 lidar bug):** 360 MJCF `<rangefinder>` sensors are evaluated on *every* 2 ms physics step, and one ray against a 500×500 heightfield costs about 0.65 ms. That gave RTF 0.008. The fix had two parts. Rays are cast only at the scan rate, by the plugin. The collision terrain is 5 m tiles (hidden group 3), because the ray-heightfield cost grows with the field's area, while the full textured field stays visual only (group 2). Lesson: profile one component in isolation (`mjDSBL_SENSOR`, then one `mj_ray`) before guessing.
- **Sim time:** MuJoCo publishes `/clock`, and every node uses `use_sim_time: true`. If one node misses it, tf lookups fail with "extrapolation into the future".
- Docs: [ros2_control concepts](https://control.ros.org/jazzy/doc/getting_started/getting_started.html), [diff_drive_controller](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html), [mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control)

**Self-check questions:**
1. What would break if both diff_drive_controller and a forward_command_controller listed `wheel_fl_joint`?
2. Why is the MJCF actuator a *velocity* servo, and what would you lose with a *motor* (torque) actuator plus a ros2_control PID?
3. With m = 2.94, the rover commanded at ω = 0.5 rad/s really turns at about 0.5 rad/s on flat ground. What do you expect on sand (friction 0.35), and which odometry source shows the error?

**Experiments:**
- `ros2 control list_hardware_interfaces` and `ros2 control list_controllers`: find the claimed interfaces.
- Measured on flat ground (`tools/check_bringup.sh flat`): with ω = 0.5 rad/s for 10 s, the wheel odometry says 283° and the truth is 335°. So at this speed the multiplier over-corrects by about 18%. Why might turn efficiency depend on the turn rate?
- Set `wheel_separation_multiplier: 1.0`, rerun `tools/check_bringup.sh`, and compare the wheel-odom yaw with the ground truth after the 10 s turn.

## Part 2 · tf2 tree + EKF: who publishes which frame  *(Phase 3, done)*

**Where it sits:** every consumer (Nav2, RViz, the metrics node) asks tf2 "where is X in frame Y at time t". If the tree is wrong or two nodes publish the same edge, everything above it fails in confusing ways.

**The tree and its publishers:**
- `map → odom`: a static identity transform for now (`static_transform_publisher`). Later a global localizer could correct the drift here. It never jumps the robot inside `odom`.
- `odom → base_link`: the **EKF only**. diff_drive's `enable_odom_tf` is off, and the ground-truth odometry must not publish tf either. Two publishers of one edge make the robot flicker between two poses.
- `base_link → front_axle_link → wheel_f*`, `base_link → wheel_r*`: robot_state_publisher, from the URDF plus `/joint_states`. A joint with no state comes out as a missing frame.
- `base_link → imu_link, lidar_link`: robot_state_publisher, from the fixed joints in the URDF.

**Read:** `src/mars_yard_bringup/config/ekf.yaml`. For each sensor, the 15-element boolean vector says which state variables it feeds: `[x y z, roll pitch yaw, vx vy vz, vroll vpitch vyaw, ax ay az]`.

**Concepts to know:**
- **REP 105:** why `odom` is continuous but drifts, and why `map` doesn't drift but may jump.
- **Fuse velocities, not poses, from wheel odometry.** Wheel odometry's pose is an integral of the same velocities, so fusing both counts the information twice and makes the EKF overconfident.
- **The IMU gives absolute roll and pitch** (from gravity) and yaw *rate*. On Earth a magnetometer adds absolute yaw. Mars has no global magnetic field, so the EKF fuses only the gyro's yaw rate. MuJoCo's `framequat` would give the *true* world yaw for free, and fusing it made the EKF heading error exactly 0. That is a simulation cheat, and it would hide the heading drift the benchmark is meant to measure (DEVLOG, Phase 3).
- **two_d_mode:** it forces z, roll and pitch to 0. That's fine for Nav2's 2D planning, but it hides the terrain tilt.
- Docs: [REP 105](https://www.ros.org/reps/rep-0105.html), [robot_localization state estimation nodes](https://docs.ros.org/en/jazzy/p/robot_localization/), [tf2 tutorials](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html)

**Self-check questions:**
1. Who publishes `odom → base_link`, and what happens if diff_drive publishes it too?
2. Why are `map` and `odom` separate frames, if `map → odom` is identity right now?
3. You remove `front_axle_joint` from `<ros2_control>`. Which frames disappear, and why?

**Experiments:**
- `ros2 run tf2_tools view_frames` and compare with `results/tf_frames.pdf`.
- `ros2 run tf2_ros tf2_echo odom base_link` while driving, compared with the ground-truth topic: that difference is the drift Phase 4 measures.

---

## Part 3a · 3D lidar + step-height hazard detection: finding rocks on slopes  *(Phase 5a, done)*

**Where it sits:** between the simulator's 3D lidar and Nav2's local obstacle costmap. A 2D horizontal lidar at `0.57 m` passes straight over rocks (`0.04–0.50 m` exposed) and hits uphill slopes, so it cannot separate a drivable slope from a rock. Phase 5a casts a downward-concentrated 3D ray fan, levels each scan with IMU roll/pitch, measures local step height against a robust local ground plane, and publishes `/hazards` (`PointCloud2` in `odom`: hazards at `z = 0.3`, observed free cells at `z = 0.0`) plus `/hazard_grid` (`OccupancyGrid`) for RViz.

**The pipeline, one 5 Hz 3D scan:**
1. `RayScanPlugin` (`ray_cloud_plugin`) casts `28 × 91 = 2,548` rays (`[-90°, +90°]` azimuth, `[-30°, +2°]` elevation) in ONE `mj_multiRay` call (~`15.3 ms` on `mars`) and publishes hits-only float32 `(x, y, z)` on `/points` in `lidar_link`.
2. `hazard_mapper.py` matches the scan timestamp to the nearest IMU sample and transforms `lidar_link → base_link` (static tf) → gravity-leveled frame (`Ry(pitch) @ Rx(roll)`, discarding IMU yaw) → `odom` (`(x, y, yaw)` from the 2D EKF tf).
3. `detect_hazards_scan()` bins points into `0.10 m` cells, keeping exact `(x, y, z)` of each cell's lowest and highest point, and fits a 2-pass vectorized integral-image local plane over a `±0.70 m` (`1.5 m × 1.5 m`) window. Pass 1 rejects elevated rock points (`res > 0.06 m`); Pass 2 refits the ground plane and computes `step_height` at the cell's exact `(x_max, y_max)`.
4. `RollingHazardMap` updates discrete labels (`-1 / 0 / 100`) and hysteresis scores in a `24 m × 24 m` rolling `odom` grid (`memory_ttl = 30 s`)—never accumulating raw `z` across scans—and publishes `/hazards` and `/hazard_grid`.

**Read, in this order:**
1. `src/mars_yard_sim_plugins/include/mars_yard_sim_plugins/ray_scan_plugin.hpp` and `src/mars_yard_sim_plugins/src/ray_scan_plugin.cpp`: how one plugin class serves both the 1-ring `/scan` and the 28-ring `/points` instances using one `mj_multiRay` call and `realtime_tools::RealtimePublisher`.
2. `src/mars_yard_bringup/config/mujoco_plugins.yaml`: the two plugin instances and why the 3D lidar uses a forward-only `180°` azimuth sector with `28` downward-concentrated rings.
3. `src/mars_yard_nav/mars_yard_nav/hazard_core.py`: the pure-NumPy math (`level_rotation`, `transform_points_to_odom`, `_box_sum_2d`, `_fit_local_plane`, `detect_hazards_scan`, `RollingHazardMap`).
4. `src/mars_yard_nav/scripts/hazard_mapper.py` and `src/mars_yard_nav/config/hazard_mapper.yaml`: the thin ROS 2 wrapper (fast NumPy `PointCloud2` parsing/packing, IMU ring buffer, TF lookups).
5. `tools/tests/test_hazard_core.py` and `tools/check_hazards.sh`: offline slope/bump/levelling tests and headless end-to-end verification against `obstacles.json`.
6. `tools/make_mars_world.py` (`place_boulders()`): how 8 larger boulders (`0.30–0.50 m` exposed) are added from a separate RNG stream without moving any of the 40 existing rocks or 3 sand patches.

**Concepts to know:**
- **Why step height (not raw height) for planetary rovers:** on a 15° smooth slope, elevation changes by `0.27 m` per metre, so a fixed `z` threshold (`min_obstacle_height`) flags the entire uphill slope as an obstacle. Perseverance ENav and Nav2 terrain-costmap discussions (navigation2 #4325) instead measure *local roughness / step height* relative to the local ground plane and compare it against the rover's wheel radius (`0.12 m`).
- **2D EKF vs 3D perception:** `two_d_mode: true` zeroes `z`, `roll`, and `pitch` in `odom → base_link`. You must level 3D sensor data with IMU roll/pitch yourself, discard IMU yaw (no Mars compass), and never fuse raw `z` across scans at different elevations—only persist hazard/free *labels* in `odom`.
- **Two-pass robust plane fitting & integral images:** a 2D least-squares plane depends only on 9 windowed sums (`N, Sx, Sy, Sz, Sxx, Sxy, Syy, Sxz, Syz`), computable over every grid cell in `< 1 ms` via 2D prefix sums (`cumsum`). Refitting after trimming Pass-1 positive outliers prevents a large boulder from tilting the fitted ground plane upward. Checking the smaller 2D covariance eigenvalue (`λ_min >= 0.015 m²`) rejects ill-conditioned single-ring arcs at far range.
- **Exact-point evaluation vs cell centres:** evaluating the fitted plane at the cell's actual `(x_max, y_max)` instead of the cell centre removes `cell_size * tan(slope)` discretization error, making the residual identically `0` on any smooth tilted plane.
- **Nav2 obstacle-layer contract (`/hazards`):** publishing hazard cells at `z = 0.3` and observed free cells at `z = 0.0` lets a standard Nav2 `ObstacleLayer` (`min_obstacle_height: 0.1`, `clearing: true`) mark rocks and raytrace-clear free ground without a custom C++ costmap plugin.
- **Docs:** [sensor_msgs/PointCloud2](https://docs.ros2.org/jazzy/api/sensor_msgs/msg/PointCloud2.html), [nav_msgs/OccupancyGrid](https://docs.ros2.org/jazzy/api/nav_msgs/msg/OccupancyGrid.html), [Nav2 ObstacleLayer](https://docs.nav2.org/configuration/packages/costmap-plugins/obstacle.html), [Navigation2 #4325 (rough terrain)](https://github.com/ros-navigation/navigation2/issues/4325), [MuJoCo `mj_multiRay`](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-multiray)

**Self-check questions:**
1. Why did a 360° downward-tilted lidar hit the rover's own chassis even though `RayScanPlugin` passes `body_exclude_ = lidar_link` to `mj_multiRay`?
2. What would happen to `/hazard_grid` on Mars if `hazard_mapper.py` used `odom → lidar_link` directly from tf2 instead of applying IMU roll/pitch?
3. Why does `detect_hazards_scan()` evaluate `z_plane` at `(x_max, y_max)` rather than the cell centre, and why does it run a second pass after excluding `res1_min > 0.06 m`?
4. Why does `RollingHazardMap` store discrete `-1 / 0 / 100` labels in `odom` instead of averaging raw `z` elevations over 30 seconds?

**Experiments:**
- Run `./tools/check_hazards.sh flat` and `./tools/check_hazards.sh mars`, then launch `ros2 launch mars_yard_bringup sim.launch.py world:=mars` and drive around `boulder_0` while watching `/points` and `/hazard_grid` in RViz2.
- In `src/mars_yard_nav/config/hazard_mapper.yaml`, lower `step_threshold` to `0.05` and rerun `./tools/check_hazards.sh mars`: watch `rock_13` (`0.059 m` exposed) and small terrain undulations start triggering hazards.
- In `tools/tests/test_hazard_core.py`, replace `(x_max, y_max)` plane evaluation in `hazard_core.py` with cell centres on a `25°` slope and observe the discretization bias appear.

## Part 4 · Rover model: MJCF physics + URDF frames  *(Phase 1, done)*

**Where it sits:** at the very bottom. MuJoCo simulates `mjcf/rover.xml`, and ROS sees the same robot through `urdf/rover.urdf.xacro`.

**Read, in this order:**
1. `src/mars_yard_description/mjcf/rover.xml`
   - `<default>` classes: the wheel friction and joint damping live here
   - the `base_link` body with its `<freejoint>`: a free body in 3D, which is the rover itself
   - the `front_axle_link` pivot
   - the lidar `<replicate>` block, plus its comment on how the frame trick works
   - `<actuator>`: velocity servos named like the joints
   - `<sensor>`: IMU naming convention + `rangefinder`
2. `src/mars_yard_description/urdf/rover.urdf.xacro`: the same numbers as xacro properties; the `wheel` macro
3. `tools/drive_test.py`
   - `wheel_speeds()`: differential-drive inverse kinematics, the same formula `diff_drive_controller` uses
   - `run()`: the bare MuJoCo loop: set `data.ctrl`, call `mj_step`, read `data.qpos`
4. `tools/tests/test_model_consistency.py`: how the two files are kept in sync

**Concepts to know:**
- `MjModel` (constant: geometry, masses, gains) vs `MjData` (changing: `qpos`, `qvel`, `ctrl`, `sensordata`, `contact`).
- **qpos layout:** a free joint takes 7 numbers (x, y, z + quaternion w, x, y, z), and each hinge takes 1. `qvel` for a free joint takes 6.
- **Velocity actuator:** torque = kv·(ctrl − joint velocity), clipped to `forcerange`. It works like the speed loop of a real motor driver.
- **Contacts:** soft constraints (`solref`/`solimp`); the friction cone (pyramidal vs elliptic).
- **Why `implicitfast`:** stiff damping-like forces would explode with explicit Euler at a 2 ms step.
- **Skid-steer:** to turn, the wheels must slide sideways. The turn efficiency is the true yaw rate divided by the commanded one. It is ≈ 0.35 here, and the Coulomb upper bound is b²/(a²+b²).
- Docs: [MuJoCo Modeling](https://mujoco.readthedocs.io/en/stable/modeling.html), [XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html), [URDF/xacro tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html)

**Self-check questions:**
1. Why does the rover turn about 3× slower than the wheel speeds say? How is that corrected in ROS, and why not in the physics?
2. A rangefinder ignores which geoms? What went wrong with the first lidar version?
3. What breaks if the URDF wheel radius is 0.13 but the MJCF one is 0.12? Think about which node uses each file.

**Experiments:**
- Set `kv="5"` in `rover.xml` and run `tools/drive_test.py --headless`. Watch the turn efficiency drop.
- Change the wheelbase (`wheel_x`) to 0.18 in both files. Does the turn efficiency get closer to b²/(a²+b²)?
- Set the wheel friction to 0.3 and run the straight segment. Is the distance still 1.57 m? Why, or why not?

---

## Part 5 · Terrain pipeline: orbital DEM → MuJoCo heightfield + Nav2 map  *(Phase 2, done)*

**Where it sits:** offline, before any simulation. A single DEM feeds two consumers: MuJoCo, which drives on the terrain, and Nav2, which plans on its slope map. This is how real missions plan with orbital data.

**Read, in this order:**
1. `tools/dem_to_world.py`, `run_pipeline()` (the overview), then:
   - `crop_and_resample_dem()`: rasterio windows and `reproject` with bicubic resampling
   - `compute_slope_deg()`: `np.gradient` with the real pixel spacing, then the slope angle `atan(|∇z|)`
   - `slope_to_nav2_pgm()` and `write_nav2_map()`: the PGM pixel ↔ cost convention (254 free … 0 lethal, `mode: scale`)
2. `tools/make_mars_world.py`
   - `Terrain.index()`: **the** coordinate convention (row 0 = north, column 0 = west)
   - `place_sand()`: the sand-as-second-heightfield trick
   - `world_xml()`: the `<hfield size="rx ry z_range base">` element and the z-shift so the spawn ground sits at 0
3. `tools/tests/test_dem_to_world.py` and `tools/tests/test_mars_world.py`: how orientation is *proven* with ray casts

**Concepts to know:**
- **GeoTIFF / PDS IMG basics:** CRS, affine transform, nodata.
- **Upsampling** (1 m → 0.1 m) adds no information, only smoothness.
- **MuJoCo `hfield`:** a 16-bit PNG normalised to [0, 1] × the elevation in `size`. It has one friction value per geom.
- **Nav2 map YAML:** `resolution`, `origin` (the world pose of the image's *bottom-left* pixel), `mode: trinary/scale/raw`, and the thresholds.
- Docs: [MuJoCo hfield](https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-hfield), [Nav2 map_server](https://docs.nav2.org/configuration/packages/configuring-map-server.html), [rasterio windows](https://rasterio.readthedocs.io/en/stable/topics/windowed-rw.html)

**Self-check questions:**
1. Nav2 says the map origin is the bottom-left pixel, but row 0 of the PGM is north. Why does that work out?
2. Why is the sand a second heightfield and not a flat box?
3. Which is more trustworthy for the planner, a slope computed at 1 m or at 0.1 m spacing? Why?

**Experiments:**
- Run `dem_to_world.py` with `--lethal-slope 15`, then regenerate the world, and compare `results/terrain_preview.png`.
- In `test_mars_world.py`, flip y in the lookup and watch the test fail. That is the bug it guards against.

---

## Part 6 · C++ metrics node: ground truth, drift, slip, and energy  *(Phase 4, done)*

**Where it sits:** beside the navigation stack, watching only ROS topics. It never touches MuJoCo directly and never publishes TF. It turns `/simulator/floating_base_state`, `/odometry/filtered`, `/diff_drive_controller/odom`, `/joint_states`, and `/plan` into a standard `/ground_truth/odom` plus ten `/metrics/*` scalar streams for live plotting and Phase 6 benchmarking.

**Read, in this order:**
1. `src/mars_yard_metrics/include/mars_yard_metrics/metrics_core.hpp` and `src/mars_yard_metrics/src/metrics_core.cpp`: all the math (`Pose2D`, `wrap_angle`, `rotate_world_to_body`, `relative_pose`, `drift`, `cross_track_error`, `SlipAccumulator`, `EnergyAccumulator`), with zero ROS headers.
2. `src/mars_yard_metrics/test/test_metrics_core.cpp`: GoogleTest (`ament_cmake_gtest`) unit tests covering `±π` wrap, quaternion rotations, polyline endpoints, empty paths, reverse driving, in-place turns, and negative torques.
3. `src/mars_yard_metrics/src/metrics_node.cpp`: the thin `rclcpp::Node` wrapper—subscriptions, origin `T0` capture, `/ground_truth/odom` twist conversion, and the 10 Hz sim-time publisher timer.
4. `src/mars_yard_metrics/CMakeLists.txt` and `package.xml`: how `ament_cmake` builds a static library (`metrics_core`), links it into both `metrics_node` and `test_metrics_core`, and registers the test target.
5. `tools/check_metrics.sh`: headless end-to-end check that publishes a straight `/plan`, drives forward then turns, and prints every metric and energy breakdown.

**Concepts to know:**
- **Pure core + thin ROS shell:** putting every formula in a ROS-free library (`metrics_core`) means unit tests compile and run in milliseconds without launching DDS or a simulator. `metrics_node.cpp` only translates ROS messages into plain structs and manages timers.
- **Sim-time timers in `rclcpp`:** `create_wall_timer` ticks on wall-clock time; `rclcpp::create_timer(this, this->get_clock(), period, cb)` ticks on `/clock` when `use_sim_time: true`, so metrics stay aligned with physics even if RTF < 1.
- **REP 105 twist frames vs MuJoCo free joints:** `nav_msgs/Odometry` puts `pose` in `header.frame_id` (`map`) and `twist` (both linear and angular) in `child_frame_id` (`base_link`). `MujocoSystemInterface` 0.1.4 copies MuJoCo's free-joint `qvel` verbatim: `qvel[0..2]` is world-frame linear velocity (must be rotated by `q⁻¹`), while `qvel[3..5]` is already body-local angular velocity (must *not* be rotated again).
- **Metric definitions and their limits:**
  - *Drift (`ekf_*_drift`, `wheel_*_drift`):* captures the first paired GT pose as `T0`, expresses GT in `T0`'s planar frame (`GT_rel = relative_pose(T0, GT)`), and compares against EKF/wheel odom. **Limits:** assumes `map == MuJoCo world` and EKF starts at spawn (will need revision when SLAM/AMCL moves `map → odom` in Phase 5); compares latest messages directly, so EKF's ~12 ms pipeline lag slightly shifts live position drift while moving (~5 mm at 0.4 m/s).
  - *Slip (`slip_ratio`, `slip_instant`, `lateral_slide`):* `v_wheel = r · mean(ω_i)` vs body-frame `v_true_x`. Longitudinal distances only accumulate when `|v_wheel| > 0.02 m/s` so in-place turns don't divide by zero, while `lateral_slide = ∑|v_true_y|dt` always accumulates so downhill sliding during turns is visible (`0.51 m` on Mars vs `0.18 m` on flat). **Limit:** if asymmetric wheel speeds briefly push `|v_wheel| > 0.02 m/s` while sliding downhill faster than the wheels roll, `slip_ratio` can go negative.
  - *Cross-track error (`cross_track_error`):* minimum Euclidean distance from `GT_rel (x, y)` to all segments of `/plan` (`NaN` until a plan arrives). **Limit:** planar 2D distance to the closest segment, not progress-indexed along the path (on self-crossing paths it picks the nearest branch).
  - *Energy (`energy`):* integrates mechanical work magnitude `∑ |τ_i · ω_i| dt` from `/joint_states` wheel `effort` (`qfrc_actuator`) and `velocity`. **Limit:** captures mechanical shaft work without regenerative credit (so straight driving costs ~`7–8 J/m` while a 10 s skid-steer turn burns ~`440 J`), and omits motor copper losses (`I²R`) and avionics idle power.
- Docs: [rclcpp API](https://docs.ros.org/en/jazzy/p/rclcpp/), [ament_cmake user guide](https://docs.ros.org/en/jazzy/How-To-Guides/Ament-CMake-Documentation.html), [GoogleTest primer](https://google.github.io/googletest/primer.html), [REP 105](https://www.ros.org/reps/rep-0105.html)

**Self-check questions:**
1. Why does `metrics_node` rotate `floating_base_state.twist.twist.linear` by `q⁻¹` but leave `twist.twist.angular` untouched? What would go wrong on a 15° slope if you rotated both?
2. Why does `SlipAccumulator` gate longitudinal distance on `|v_wheel| > 0.02 m/s` yet still integrate `lateral_slide` below that threshold?
3. Why did `mars` straight driving use slightly *less* energy per metre (`6.6 J/m`) than `flat` (`7.6 J/m`), while a 10 s in-place turn burned more than 14× as much energy as driving 4 m straight?

**Experiments:**
- Run `colcon test --merge-install --packages-select mars_yard_metrics` and break `wrap_angle` (e.g., return `angle` unchanged); inspect `colcon test-result --verbose` to see which gtest cases fail.
- Run `./tools/check_metrics.sh flat` and `./tools/check_metrics.sh mars`, then compare `lateral_slide`, `ekf_pos_drift`, and the forward vs turn energy breakdown between the two worlds.

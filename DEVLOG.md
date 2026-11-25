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

## 2026-11-24 · Phase 1 · Rover dimensions (every number justified)

| Quantity | Value | Why |
|---|---|---|
| Total mass | 40 kg (chassis 35.5 + 4 wheels × 1 + front axle 0.5) | Small planetary rover class (DLR LRU, ESA ExoTeR range of 20–40 kg); one person can carry it |
| Chassis | 0.8 × 0.5 × 0.2 m box | Room for batteries, electronics and the solar panel on top |
| Wheel radius / width | 0.12 m / 0.10 m | Climbs rocks up to about 1 wheel radius; the width spreads load on loose soil |
| Wheelbase / track | 0.50 m / 0.64 m (half: 0.25 / 0.32) | Track > wheelbase: skid-steer turning is easier the shorter the wheelbase is relative to the track |
| Ground clearance | 0.17 m | Wheel centre 0.15 m below base_link, chassis half-height 0.10, wheel radius 0.12 |
| Lidar height | 0.57 m above ground (base_link + 0.40) | Sees over 0.35 m rocks; higher means it hits sloped ground later |
| Wheel servo | velocity actuator, kv = 50 N·m·s/rad, ±15 N·m, ±6 rad/s | See the turning entry below. ±6 rad/s = 0.72 m/s maximum; Nav2 will use ≤ 0.5 m/s |
| Gravity | 3.721 m/s² (Mars) | Lower normal force means proportionally less traction; part of the realism the benchmark measures |
| Wheel friction | 0.8 (regolith); 0.35 (sand patches) | Typical dry regolith simulant vs loose sand |
| Timestep / integrator | 2 ms, `implicitfast` | `implicitfast` integrates the actuator damping implicitly, so stiff velocity servos stay stable at 2 ms |

Torque check: climbing a 25° slope needs m·g·sin(25°) = 40 · 3.721 · 0.42 ≈ 63 N, or 16 N per wheel, which is 1.9 N·m at r = 0.12. The ±15 N·m limit therefore leaves a large margin for climbing rocks and for scrub torque while turning.

## 2026-11-24 · Phase 1 · Hand-written URDF + MJCF, checked by a test (no URDF→MJCF generator)

- **Decision:** two hand-written files. `urdf/rover.urdf.xacro` serves ROS (tf, RViz, ros2_control joint list); `mjcf/rover.xml` serves MuJoCo (physics, sensors, actuators). `tools/tests/test_model_consistency.py` checks that frame positions, wheel size, total mass and joint↔actuator names match.
- **Alternatives:** generate the MJCF from the URDF, using mujoco_ros2_control's `mujoco_inputs` / `raw_inputs` tags (demo 02).
- **Why:** MuJoCo-only features dominate this model: rangefinder `replicate`, actuator gains, contact friction and `implicitfast`. Through a generator they would be strings inside URDF tags, hard to read and hard to test. Two small files plus a test are easier to study and catch the same mistakes.
- **Watch:** when you change a dimension, change it in both files. The test fails otherwise.

## 2026-11-24 · Phase 1 · The lidar is 360 MuJoCo rangefinders made with `<replicate>`

- **Decision:** a single `<rangefinder name="lidar" site="rf"/>` on a site replicated 360 times (1° steps). MuJoCo compiles it into `lidar-000 … lidar-359`, and the bridge's `RangefinderLidarPlugin` packs these into one `LaserScan`.
- **The frame trick:** a rangefinder measures along its site's +z axis. The parent body `lidar_rays` has `xyaxes="0 -1 0 0 0 1"`: its y axis points up and its z axis points backwards (the −π direction). Each copy rotates a further 1° about y, so ray k points at angle −π + k·1°, counter-clockwise, which is exactly the `LaserScan` convention (angle_min = −π).
- **Bug found and fixed:** at first all 360 rays "hit" something on an empty plane. They were hitting the rover's own lidar head and mast. A rangefinder ignores only geoms of its own body, so the head geom moved into `lidar_rays` and the mast now ends 3 cm below the rays.
- **Bug 2:** visual-only geoms (no collision) still get mass from the default density, which made the rover 44 kg instead of 40. Fixed with `mass="0"` on the `visual_only` default class.

## 2026-11-24 · Phase 1 · Skid-steer turning: measured turn efficiency, and the false lead first

This was the longest debugging session of Phase 1. The method matters as much as the result.

1. **Symptom:** spin in place commanded at 0.5 rad/s for 4 s (2 rad). The rover turned 0.34 rad.
2. **Hypothesis 1, servo too soft (kv = 5):** partly true. At kv = 5 the wheels reached only about 70 % of the commanded speed. Raising kv to 50 made them track (1.30 vs 1.33 rad/s), but yaw improved only to about 0.7 rad.
3. **Hypothesis 2, the rigid chassis rocks onto 3 wheels:** a contact count showed one wheel "without contact" 70–95 % of the time. **False lead.** Logging the chassis showed zero roll, zero pitch and a height std of 0.02 mm, so nothing was lifting. The wheels sit only about 0.04 mm into the ground, so contact detection flickers on and off at zero distance. The flicker is harmless: average normal force still equals weight, and a detection margin didn't change the turning.
4. **Hypothesis 3, soft friction (MuJoCo's regularised contacts):** an elliptic cone with `impratio` 10–50 made it slightly *worse*, so that wasn't it.
5. **Conclusion:** this is skid-steer physics. Whenever the rover yaws, the wheels must slide sideways. Rigid-wheel Coulomb theory gives an upper bound of yaw/commanded = b²/(a²+b²) ≈ 0.62, with a = half wheelbase and b = half track. MuJoCo gives about 0.34–0.38. Published skid-steer robots measure 0.3–0.7 depending on the surface.
- **Decision:** accept the physics. Measure the turn efficiency (`tools/drive_test.py` prints it), and compensate the standard way in ROS: `diff_drive_controller`'s `wheel_separation_multiplier` ≈ 1/efficiency, an "effective track width" (Phase 3).
- **Why:** faking friction to make turning "nice" would destroy what the benchmark measures. Odometry drift in turns is a headline metric.
- **Watch:** the efficiency depends on friction, so it changes on sand and under randomisation (Phase 7). Wheel odometry yaw will therefore be wrong, which is why the EKF fuses the IMU gyro.

## 2026-11-24 · Phase 1 · `colcon build --merge-install`, so worlds can `<include>` the rover

- **Decision:** `build.sh` runs `colcon build --merge-install --symlink-install`. The world files include the rover as `../../mars_yard_description/mjcf/rover.xml`.
- **Why:** MuJoCo resolves `<include>` relative to the including file. With merge-install, every package's share folder sits side by side under `install/share/`, the same layout as `src/`. One relative path then works in both trees, so Python tools load from `src/` and the bridge loads from `install/`.
- **Alternatives:** absolute paths written by the launch file into a temporary scene; or putting the worlds and the rover in one package (that breaks the package split in the spec).
- **Watch:** a plain `colcon build`, without merge-install, breaks the include. Docker and CI must use `build.sh`.

## 2026-11-24 · Phase 2 · Terrain data pipeline (HiRISE DTM → MuJoCo heightfield + Nav2 slope map)

- **What:** HiRISE DTM `DTEEC_045994_1985_046060_1985_U01` (1.01 m/px, Jezero crater) and orthoimage `ESP_045994_1985_RED_A_01_ORTHO` (0.25 m/px) in `data/raw/` (git-ignored, 767 MiB). `tools/dem_to_world.py` crops a 50 × 50 m patch, upsamples it bicubically to 0.1 m/px, and writes `heightmap.png` (16-bit), `texture.png`, `slope.pgm/yaml` (Nav2, `mode: scale`: ≤ 10° free, ≥ 20° lethal) and `terrain.json`. `tools/find_patch.py` searched the DTM on a 25 m grid for a patch with mostly gentle ground plus a steep zone.
- **Chosen patch:** centre (-2025.38, 1094617.41) m in the Equirectangular Mars CRS. Median slope 5.6°, p90 16.5°, 6.2 % lethal, height range 7.56 m, no nodata.
- **How it was checked:** tests in `tools/tests/test_dem_to_world.py` (tilted-plane slope = 10 ± 0.5°, PGM row 0 = north, nodata rejected).
- **Gotchas:**
  1. The `.JP2` georeference header swaps two latitude parameters, so the `.LBL` sidecar must be passed to `--ortho`.
  2. The system matplotlib (apt, built for numpy 1.x) breaks in the numpy-2 venv, so the venv now installs its own matplotlib (pinned in `setup_wsl.sh`).
  3. Upsampling overshoots the slope at sharp ridges: 35° max at 0.1 m vs 29° at the native 1 m spacing.
- **Known limitation:** the DTM's true resolution is about 1 m. Detail below 1 m comes from interpolation (it's smooth), and small-scale roughness comes only from the rock geoms.

## 2026-11-24 · Phase 2 · Mars world is generated (`tools/make_mars_world.py`), not hand-written

- **Decision:** a seeded script writes `worlds/mars.xml` from the terrain assets, places 40 rocks and 3 sand patches, and records their positions in `terrain/obstacles.json`.
- **Why:** rocks must sit on the terrain, which requires a height lookup per rock. A generator does this reproducibly (seed 7). The benchmark also needs to know where the rocks and sand are.
- **Sand trick:** a MuJoCo heightfield has one friction value. Each sand patch is therefore a *second* small heightfield, cut from the same DEM and lifted 5 mm, with friction 0.35. The wheels touch the sand surface inside the patch and the regolith outside it, so the patch follows the terrain exactly. (A flat box would float or sink on slopes.)
- **Spawn convention:** the terrain is shifted so that the ground under (0, 0) is at z = 0. The rover's spawn height in `rover.xml` (0.275 m) is then correct in every world, and the Nav2 map frame sits on the ground. *(Updated in Phase 3c: shifted so the highest ground point under the 4 spawn wheel centres sits at z = 0, eliminating the 21 mm initial uphill-wheel penetration on the slope.)*
- **Orientation verified, not assumed:** `tools/tests/test_mars_world.py` casts rays down onto the MuJoCo heightfield and compares them with the DEM read in the Nav2 convention (image row 0 = north). A flipped map gives about 60 cm of error, so it can't slip through. (The first version had a 1.1 cm error, which hid a bug; see the next entry.)

## 2026-11-24 · Phase 2 · Bug: MuJoCo reads a PNG heightfield as 8-bit, so the terrain had 3 cm terraces

- **Symptom:** the first Tier-A render showed the terrain as regular shadowed steps. The orientation test passed, but with a 1.1 cm median error and a 3.4 cm maximum, which is suspiciously large for a 0.1 mm-precision 16-bit PNG.
- **Cause:** MuJoCo loads PNG heightfields as 8-bit grey. 7.56 m / 255 ≈ 3 cm per step. Those steps are visible, and they shake the wheels like driving on stairs.
- **Fix:** `make_mars_world.py` now writes MuJoCo's binary heightfield format: `int32 nrow`, `int32 ncol`, then `float32` data, as `heightmap.bin` and `sand_<k>.bin`. The 16-bit PNG stays the human-readable source and the input of the Nav2 tools.
- **Second gotcha, caught by the test:** the binary format stores **row 0 = south**, the opposite of PNG, where MuJoCo flips the image. The first `.bin` gave a 60 cm median error. `write_hfield_bin()` now flips the rows, so every caller keeps the image convention (row 0 = north).
- **Result:** median error 0.4 cm (that's nearest-pixel lookup vs interpolation), down from 1.1 cm. The test threshold tightened from 3 cm to 1 cm, so 8-bit terraces would now fail it.
- **Lesson:** "the test passed" is not the same as "the numbers are right". The error size was the hint, and the render made it obvious. Look at your data.

## 2026-11-24 · Phase 2 · Texture and render checks

- A top-down render matches the orthoimage preview, and the sand patches appear where `obstacles.json` says (x right, y up). So the texture orientation is correct.
- Sand patches are now untextured. A 2D texture on a heightfield always spans the whole field, so each 4 m patch had the full 50 m orthoimage squeezed onto it.
- `render_tier_a.py` turns off `mjVIS_RANGEFINDER`. Otherwise the 360 lidar rays are drawn as yellow lines.
- Offscreen rendering works in WSL with `MUJOCO_GL=egl`. At exit, EGL prints a harmless "Exception ignored in `Renderer.__del__`" during interpreter shutdown.

## 2026-11-24 · Phase 1–2 · Keep the front axle pivot (decided on terrain, not on flat ground)

- **Decision:** the front wheels sit on a `front_axle_link` that rolls ±0.35 rad about x (damping 2), a one-axis cousin of a rocker. The rear wheels are fixed to the chassis.
- **Alternatives:** a rigid chassis (simplest; it was the Phase 1 model), or a full rocker-bogie (6 wheels, a differential; that's a Phase 10 extension).
- **Why:** a rigid 4-wheel body on uneven ground stands on 3 wheels, so one wheel hangs in the air and loses traction. On flat ground the pivot changes nothing (the spin efficiency stayed 0.34), so the decision needed a terrain test. Each variant drove 30 s straight at 0.4 m/s on the Jezero world in 4 headings. The lidar was removed in that scratch copy for speed, and the repo was untouched:

  | variant | distance (of 12 m) | all 4 wheels touching | max tilt | vertical accel std |
  |---|---|---|---|---|
  | rigid | 11.78–11.97 m | **1–8 %** of the time | 5.7–10.4° | 0.24–0.38 m/s² |
  | pivot | 11.81–11.94 m | **99 %** | 5.7–11.2° | 0.19–0.24 m/s² |

  On these gentle slopes the distance is the same, but the rigid rover is nearly always on 3 wheels and shakes more. On rocks, sand and steeper ground, a hanging wheel means lost traction and wheel odometry error, which the Phase 6 benchmark measures.
- **Watch:** the URDF has `front_axle_joint` as a revolute joint, so `joint_state_broadcaster` must publish it, otherwise robot_state_publisher can't place the front wheels in tf. The consistency test already checks the axle frame.
- **Process lesson:** the first comparison run took more than 45 min with no output. The 360 lidar rays were cast against the heightfield every step, and Python buffered all prints until exit. With the lidar removed and `PYTHONUNBUFFERED=1` it took about 2 minutes. Earlier, a cancelled job had left the variant model in the repo, so comparison scripts now work only on a `/tmp` copy.

## 2026-11-24 · Phase 3 · `ros2_control` integration in `rover.urdf.xacro` and ground-truth odometry

- **Decision:** added `<ros2_control name="MujocoSystem" type="system">` using `mujoco_ros2_control/MujocoSystemInterface`, gated on `mujoco_model != ''`, with xacro args `mujoco_model` (default `""`) and `headless` (default `false`). The 4 wheels expose `velocity` command + `position`/`velocity` state; `front_axle_joint` exposes `position`/`velocity` state only (registered as passive); `base_imu` exposes orientation, angular velocity, and linear acceleration (`mujoco_type=imu`); `odom_free_joint_name` is set to `base_free_joint`.
- **Alternatives:** emit `<ros2_control>` unconditionally with a default `$(find mars_yard_worlds)/worlds/mars.xml` path; or add a separate ground-truth publisher plugin (`FreeJointStatePublisherPlugin`).
- **Why:**
  1. `tools/tests/test_model_consistency.py` runs `xacro rover.urdf.xacro` with no arguments (and without `install/setup.bash` sourced) and collects joints via `root.iter("joint")`, which traverses `<ros2_control><joint>` elements that have no `<child>` tag. Gating `<ros2_control>` on `mujoco_model != ''` keeps `xacro rover.urdf.xacro` and the 13-test suite passing without touching the test.
  2. `MujocoSystemInterface` natively supports `odom_free_joint_name=base_free_joint` and publishes ground-truth `nav_msgs/Odometry` on `/simulator/floating_base_state` (`frame_id: odom`, `child_frame_id: base_link`). Source inspection (`mujoco_system_interface.cpp` 0.1.4) and `ros2 topic info -v /tf` confirm it does **not** broadcast any TF, leaving `odom -> base_link` exclusively to the EKF.
  3. In `mujoco_ros2_control` 0.1.4, `<sensor name="base_imu">` requires `<param name="mujoco_type">imu</param>` (omitting it silently skips the sensor), and `imu_sensor_broadcaster` uses the all-caps class name `imu_sensor_broadcaster/IMUSensorBroadcaster`.
- **Watch:** `MujocoSystemInterface` copies `qvel[0..2]` (world-frame linear velocity) directly into `floating_base_state.twist.twist.linear`, while `pose` is the exact world-frame free-joint pose. Use `pose` for ground-truth trajectory comparison.

## 2026-11-24 · Phase 3 · `/cmd_vel` Twist stamper, lidar plugin, and EKF configuration

- **Decision:** created `src/mars_yard_bringup` with `sim.launch.py`, `controllers.yaml`, `mujoco_plugins.yaml`, `ekf.yaml`, `rviz/sim.rviz`, and a small `scripts/twist_stamper.py` relay (`/cmd_vel` `Twist` → `/diff_drive_controller/cmd_vel` `TwistStamped`).
- **Alternatives:** (a) remap `/diff_drive_controller/cmd_vel` directly to `/cmd_vel` and require stamped teleop/Nav2 everywhere; (b) install `twist_stamper` from apt (not installed, no apt changes allowed).
- **Why:** Jazzy's `diff_drive_controller` (4.43.1) dropped `use_stamped_vel` and only accepts `TwistStamped` on `~/cmd_vel`, whereas `teleop_twist_keyboard` and standard `/cmd_vel` publishers default to unstamped `Twist`. A 35-line `twist_stamper.py` node running on `use_sim_time:=true` stamps incoming `/cmd_vel` with `/clock` sim time so `cmd_vel_timeout` behaves accurately even when RTF < 1.
- **Lidar (`RangefinderLidarPlugin`):** `parse_lidar_name()` splits on the last `-` and parses the integer index, so `rover.xml`'s replicated `lidar-000 … lidar-359` works out of the box with no MJCF changes. Two gotchas found in upstream 0.1.4 (`rangefinder_lidar_plugin.cpp`):
  1. `num_rangefinders = int((max_angle - min_angle) / angle_increment) + 1` truncates toward zero, so `max_angle` is set to `3.1241425` (just above `-π + 359 * 0.0174533`) to guarantee 360 rays.
  2. Out-of-range and miss readings (`range < range_min` or `range > range_max`, including MuJoCo's `-1.0` miss with `cutoff=0`) are published as **`-1.0`**, not `+inf`. Verified counter-clockwise sweep on a `/tmp` copy of `flat.xml` with a box at `+y` (`x=0.3, y=3.0`): hits centered cleanly on `ray[270]` at `+90.00°`, range `2.7000 m`.
- **EKF (`robot_localization` `ekf_node`):** `frequency: 50`, `world_frame: odom`, `publish_tf: true`, `two_d_mode: true`. Fuses `/diff_drive_controller/odom` (`vx, vy, vyaw`) and `/imu_sensor_broadcaster/imu` (roll, pitch, and angular velocity — **not** absolute yaw). Absolute IMU yaw is intentionally excluded because MuJoCo's `framequat` outputs the exact world heading whereas a Mars rover has no magnetometer/compass (no global magnetic field), so fusing absolute yaw would mask all heading drift. Note that MuJoCo's gyro is still noise- and bias-free, so EKF heading drift remains optimistic until Phase 7 adds gyro bias and noise via the sensor relay node. Linear acceleration is excluded because `robot_localization` hardcodes Earth gravity (`9.80665 m/s²`) when removing gravity, whereas Mars gravity is `3.721 m/s²` (so `imu0_remove_gravitational_acceleration: true` would inject a ~`6.09 m/s²` phantom bias, while leaving it `false` turns slope tilt into fake acceleration). `two_d_mode: true` keeps the state planar for 2D Nav2 costmaps and avoids unobservable z-drift; horizontal distance cosine error is < 1.5 % on 10° slopes.
- **Watch (resolved in Phase 3b):** `RangefinderLidarPlugin` and its `-1.0` no-return encoding were replaced by `mars_yard_sim_plugins/RayScanPlugin`, which publishes REP-117 `+inf`/`-inf` and eliminates per-step rangefinder cost (see Phase 3b entry below).

## 2026-11-24 · Phase 3 · First odometry drift numbers and Mars RTF bottleneck

- **Measured drive test (0.4 m/s forward then 0.5 rad/s turn on `/cmd_vel`, headless, `wheel_separation_multiplier=2.94`):**

  | World & segment | Ground truth (`/simulator/floating_base_state`) | EKF (`/odometry/filtered`) | Wheel odom (`/diff_drive_controller/odom`) | RTF |
  |---|---|---|---|---|
  | `flat` · 10 s fwd | `3.957 m`, yaw `+0.04°` | `3.967 m` (err `0.01 m`), yaw `+0.04°` | `3.983 m` (err `0.03 m`), yaw `0.00°` | `1.000×` |
  | `flat` · +10 s turn | `(4.059, -0.027) m`, yaw `-24.26°` | `(3.995, +0.003) m`, yaw `-24.26°` (**0.00°** err) | `(3.995, +0.000) m`, yaw `-77.05°` (**52.8°** err) | `1.000×` |
  | `mars` (unmodified) · 1.93 s fwd | `0.590 m` (`dy=-0.133`), yaw `-10.29°` | `0.742 m` (`dy=-0.132`), yaw `-10.29°` (**0.00°** err) | `0.760 m` (`dy=-0.001`), yaw `-0.06°` | `0.0088×` |
  | `mars` (unmodified) · +1.82 s turn | `(0.957, +0.037) m`, yaw `+58.86°` | `(0.768, -0.137) m`, yaw `+57.71°` (**1.15°** err) | `(0.778, +0.000) m`, yaw `+51.30°` (**18.8°** Δyaw err) | `0.0083×` |
  | `mars` (`/tmp`, no lidar) · 10 s fwd | `3.948 m` (`dx=+3.885, dy=-0.704`), yaw `-10.33°` | `3.952 m` (`dx=+3.887, dy=-0.711`, **0.008 m** err), yaw `-10.33°` | `3.988 m` (`dy=-0.007`, **0.706 m** lateral err), yaw `-0.10°` | `1.000×` |
  | `mars` (`/tmp`, no lidar) · +10 s turn | `(4.796, -0.222) m`, yaw `-21.40°` (slid +0.89 m downhill) | `(3.932, -0.710) m`, yaw `-21.40°` (**0.00°** err) | `(4.024, -0.004) m`, yaw `-77.39°` (**56.0°** err) | `1.000×` |

- **Key finding (blocking for interactive Nav2 on `mars`):** on unmodified `mars.xml`, RTF is **`0.008–0.024×`** (~40–120× slower than real-time). Pure MuJoCo step benchmarks isolate the exact cause:
  - `flat.xml`: `0.10 ms/step` (`21.0×` uncapped)
  - `mars.xml` (with 360 rangefinders): `178.56 ms/step` (`0.0112×`)
  - `mars.xml` (`mjDSBL_SENSOR` in memory): `0.04 ms/step` (`51.5×`) — a **4,500×** slowdown from sensors alone.
  - Setting `cutoff=10.0` in memory did not help (`233.03 ms/step`, `0.0086×`), because MuJoCo evaluates all 360 `<rangefinder>` rays against the 500×500 heightfield on every 2 ms physics step (180,000 hfield raycasts/sim-second), even though `/scan` publishes at 10 Hz.
- **Watch (resolved in Phase 3b & 3c):** replaced `RangefinderLidarPlugin` + 360 `<rangefinder>` sensors with `mars_yard_sim_plugins/RayScanPlugin` (Phase 3b) and split collision terrain into 5 m tiles with a spawn-height shift (Phase 3c), restoring `1.000×` physics RTF, `100 Hz` control rate, `10 Hz` `/scan`, and zero initial wheel penetration. Saved TF tree to `results/tf_frames.pdf` and reusable check to `tools/check_bringup.sh`.

## 2026-11-24 · Phase 3b · Custom `RayScanPlugin` replaces 360 per-step rangefinders

- **Symptom & measured cause:** `mars.xml` ran at `0.008×` RTF because MuJoCo evaluated 360 `<rangefinder>` sensors on every 2 ms physics step (~0.65 ms/ray against the 500×500 heightfield, ~240 ms/step). Diagonal rays were far slower than axis-aligned rays (0.02 ms along x vs up to 2.4 ms diagonal) because MuJoCo's ray-heightfield test scans cells in the ray's 2D bounding box. Neither `cutoff=10` (still ~238 ms for 360 rays) nor `interval="0.1"` (still ~62 ms median/step) removed the per-step overhead; with sensors disabled, a `mars` step costs `0.04 ms`.
- **Decision:** created `src/mars_yard_sim_plugins` exporting `mars_yard_sim_plugins/RayScanPlugin` (derived from `mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase`). Replaced the 360 replicated `rf` sites and `<rangefinder>` in `rover.xml` with one `lidar` site in `lidar_link` (x forward, y left, z up, matching URDF `lidar_link`). In `update_write()`, the plugin checks `data->time` (sim time) and, only when `1/publish_rate` (0.1 s) has elapsed, casts 360 horizontal CCW rays in one `mj_multiRay` call on the borrowed post-step snapshot (no full `mjData` copy under lock), excluding `lidar_link` (`bodyexclude`) and visual-only geoms (`geomgroup={1,1,0,0,0,0}`). Publishes REP-117 `sensor_msgs/LaserScan` stamped with snapshot sim time (`+inf` for misses / `> range_max`, `-inf` for `< range_min`).
- **Alternatives:** (a) keep `<rangefinder>` with `interval="0.1"` (still ~62 ms/step); (b) upstream `Mujoco3dLidarPlugin` (requires `mujoco_3d_lidar` extension plugin not packaged in apt, and we are not changing `/opt/ros`).
- **Why (numbers before → after, headless `tools/check_bringup.sh`, full 10 s fwd + 10 s turn):**
  - Angle direction & REP-117 verified in a `/tmp` flat world with boxes at `+x` (hit at index `180`, `2.900 m`), `+y` (hit at index `270`, `2.000 m`), close `-y` (`< 0.05 m`, `-inf` at index `90`), and open `-x` (`+inf` at index `0`), with zero rover self-hits.
  - `flat`: RTF `1.0000×`, `/scan` `9.99 Hz` (`360` rays: `0` valid, `360` `+inf`), EKF fwd `3.959 m` vs GT `3.956 m` (`0.002 m` err), final yaw `-24.42°` vs GT `-24.45°` (`0.03°` err).
  - `mars` (unmodified 500×500 heightfield): physics RTF improved from **`0.008×` → `1.0000×`** (full `20.20 s` sim completed in `20.20 s` wall time; `0` valid / `360` `+inf` at final settled pose on the open slope, `112–174` valid terrain hits at spawn/tilted poses).
- **Watch (resolved in Phase 3c):** on the monolithic 500×500 heightfield, a single 360-ray `mj_multiRay` still took `242.6 ms` inside `update_write()`, stalling the `controller_manager` loop and `/scan` at `3.70 Hz` (~270 ms period). Splitting the collision terrain into 10×10 5 m tiles in Phase 3c (below) cut scan latency 37× and restored the full `100 Hz` control rate and `10 Hz` `/scan`.

## 2026-11-24 · Phase 3 · Collision tiles + spawn height

- **Benchmark (Phase 3b Part B, 360-ray `mj_multiRay` + physics step + 20 s straight drive on `/tmp` copies of `mars.xml`):**

  | Option | Scan med (p95) ms | Physics step ms | Uncapped RTF (10 Hz scan) | Terrain h err med/max | 20 s drive Δ vs current |
  |---|---:|---:|---:|---:|---|
  | 0.10 m, 1×1 (500×500, previous) | 242.61 (248.08) | 0.0440 | 0.41× | 0.0 / 0.0 mm | baseline (`7.854 m`, 4w `98.0%`) |
  | 0.20 m, 1×1 (250×250) | 62.15 (69.51) | 0.0325 | 1.57× | 0.1 / 1.6 mm | `+12.0 cm`, Δyaw `+9.82°` |
  | 0.25 m, 1×1 (200×200) | 38.99 (44.17) | 0.0277 | 2.48× | 0.1 / 3.8 mm | `+12.3 cm`, Δyaw `+9.54°` |
  | 0.10 m, 10 m tiles (5×5, 1-cell ovlp) | 27.25 (32.80) | 0.0467 | 3.38× | 0.0 / 0.0 mm | `+0.9 cm`, Δyaw `-0.96°` |
  | **0.10 m, 5 m tiles (10×10, 1-cell ovlp)** | **6.54 (7.54)** | **0.0467** | **11.27×** | **0.0 / 0.0 mm** | **`-1.1 cm`, Δyaw `+0.18°`** |
  | 0.20 m, 5 m tiles (10×10, 1-cell ovlp) | 2.03 (2.46) | 0.0364 | 25.98× | 0.1 / 1.6 mm | `+12.0 cm`, Δyaw `+9.68°` |

- **Decision:**
  1. **Collision tiles:** `tools/make_mars_world.py` keeps the existing full 500×500 textured heightfield (`terrain`) as **visual-only** (`contype="0" conaffinity="0" group="2"`) and generates 100 full-resolution 5 m collision tiles (`terrain_tile_0_0` … `terrain_tile_9_9`) in `src/mars_yard_worlds/terrain/tiles/` with 1-cell overlap between neighbours, `group="3"` (hidden in default renders), `rgba="0 0 0 0.001"`, and the same regolith friction (`0.8 0.02 0.001`). Each tile `.bin` reuses `write_hfield_bin()` with its local `[lo, lo + span]` vertical scale so world heights match the full field identically.
  2. **`RayScanPlugin` group mask:** added a 6-element `geomgroups` parameter (default `[1, 1, 0, 1, 0, 0]`, set in `src/mars_yard_bringup/config/mujoco_plugins.yaml`) so lidar rays include collision tiles (group 3) and still skip visual-only geoms (group 2).
  3. **Spawn height convention:** `make_mars_world.py` now shifts the terrain, tiles, sand, and rocks so the **highest** of the 4 ground heights under the spawn wheel centres (`x = ±0.25, y = ±0.32`) sits at `z = 0` (`ground_offset_m = -2.3223 m`, 24.0 mm lower than before). With `base_link` at `z = 0.275 m` (wheel bottoms at `+5 mm`), all 4 wheels start above ground (`+3.69` to `+61.31 mm` clearance, `data.ncon = 0`), eliminating the 21 mm uphill penetration and `-10.3°` spawn yaw kick.
- **Alternatives:**
  - *0.20 m resample (1×1):* still takes `62 ms` per scan (overrunning the 10 ms control period) and smooths micro-slopes enough to shift 20 s straight-drive yaw by `+9.8°`.
  - *Per-tile textures (dropping the full visual hfield):* MuJoCo stretches a 2D texture across each heightfield individually, which would require cropping 100 ortho textures and risks visible seam artefacts.
  - *Worker thread inside `RayScanPlugin`:* would unblock the `controller_manager` loop, but a single monolithic scan still takes `243 ms` (~4 Hz max `/scan`) and adds `mjData` copy/synchronization complexity without fixing the root cause.
- **Why (verified end-to-end):**
  - *MuJoCo raycast gotcha:* `mj_ray` / `mj_multiRay` ignores geoms with `rgba[3] == 0.0` (exact zero). Setting `group="3"` (excluded by MuJoCo's default `MjvOption.geomgroup = [1, 1, 1, 0, 0, 0]`) with `rgba="0 0 0 0.001"` keeps tiles invisible in renders while remaining hit-testable by rays.
  - *Tests (`pytest tools/tests -q`, 16 passed):* collision-tile DEM orientation median error `0.351 cm` (max `1.539 cm`); 200 random + 18 seam points match the visual full field within `0.001 mm` max (threshold `1 mm`); zero initial wheel penetration at spawn. `tools/drive_test.py --headless` on `flat.xml` is unchanged (`1.57 m` straight, `0.34` turn efficiency).
  - *Full 10 s + 10 s headless `check_bringup.sh` (with EKF fusing IMU yaw rate only, not absolute yaw):*

    | World | Physics RTF | `/imu` (100 Hz) | `/joint_states` / wheel odom (50 Hz) | `/scan` (10 Hz) | After 10 s fwd (GT vs EKF vs wheel) | After +10 s turn (GT vs EKF vs wheel) |
    |---|---:|---:|---:|---:|---|---|
    | `flat` | `1.0000×` | `99.99 Hz` | `50.00 / 50.00 Hz` | `10.00 Hz` | GT `(3.956, +0.003) m, +0.04°` · EKF err `0.009 m, +0.00°` · wheel err `0.030 m, -0.04°` | GT `(4.061, -0.028) m, -24.49°` · EKF err `0.071 m, +0.08°` · wheel err `0.070 m, -52.48°` |
    | `mars` | `1.0000×` | `99.99 Hz` | `50.00 / 50.00 Hz` | `10.00 Hz` | GT `(3.995, +0.021) m, +0.21°` · EKF err `0.017 m, +0.04°` · wheel err `0.025 m, -0.23°` | GT `(4.885, +0.344) m, -17.53°` · EKF err `0.921 m, -0.01°` · wheel err `0.923 m, -59.95°` |

  - *Spawn fix impact:* Mars settled start yaw dropped from `-10.3°` to `-0.05°`; forward lateral drift dropped from `-0.704 m` to `+0.020 m`.
  - *Tier-A render & TF tree:* offscreen EGL render confirms seamless textured terrain with no visible tile seams or untextured patches; `results/tf_frames.pdf` refreshed via `tf2_tools view_frames` (`map → odom → base_link → {front_axle_link → wheel_f{l,r}_link, wheel_r{l,r}_link, imu_link, lidar_link}`).
- **Watch:**
  - *2D lidar mast geometry on slopes:* the lidar sits `0.57 m` above ground. At level spawn (`t = 0 s`), `/scan` returns `110` valid uphill collision-tile hits (`6.71–9.95 m`, `250` `+inf`, `0` `-inf`), and driving uphill returns `23–78` terrain hits (`7.77–10.00 m`). Once settled parallel to the uniform ~5.1° spawn slope, the horizontal scan plane clears the rising terrain out to `10.68 m` (just beyond `range_max = 10.0 m`) and passes above nearby short rocks (`0.04–0.24 m` tall, 30% buried), yielding `0` valid / `360` `+inf` at the settled start pose. For Phase 5/6 Nav2 obstacle avoidance, static hazards come from the slope costmap; evaluate whether `range_max` or rock sizes should be adjusted if 2D lidar rock detection is needed.
  - *Downhill slip during turns & optimistic gyro:* scrubbing in place on the Mars slope slides the rover `+0.89 m` downhill (`0.92 m` EKF/wheel position error, unobservable without scan-matching/SLAM in Phase 5), and MuJoCo's noise-/bias-free gyro keeps EKF heading error near zero (`≤ 0.08°`) until Phase 7 adds realistic gyro bias and noise.

## 2026-11-25 · Phase 4 · Bridge twist frame and wheel effort verified in `mujoco_ros2_control` 0.1.4

- **Decision:** rotate only `twist.twist.linear` from world into `base_link` when republishing `/simulator/floating_base_state` as `/ground_truth/odom`, leave `twist.twist.angular` untouched, and add `<state_interface name="effort"/>` to the 4 wheel joints in `src/mars_yard_description/urdf/rover.urdf.xacro`.
- **Alternatives:** (a) rotate both linear and angular velocity by `q⁻¹`; (b) estimate wheel torque from commanded vs actual wheel velocity (`kv · (cmd − ω)`) instead of reading `/joint_states.effort`.
- **Why:**
  1. **Twist-frame finding (`mujoco_system_interface.cpp`, tag `0.1.4`, lines 1147–1156):** `MujocoSystemInterface::read()` copies `qvel[0..2]` directly into `twist.twist.linear` and `qvel[3..5]` directly into `twist.twist.angular` without any rotation. In MuJoCo's free-joint (`mjJNT_FREE`) convention, `qvel[0..2]` is world-frame linear velocity while `qvel[3..5]` is already **body-local** (`base_link`) angular velocity. Rotating `twist.angular` by `q⁻¹` would corrupt roll/pitch/yaw rates whenever the rover is tilted. Confirmed empirically on the tilted `mars` slope by comparing timestamp-matched `/ground_truth/odom` `twist.angular` against `/imu_sensor_broadcaster/imu` `angular_velocity` (body-local match mean diff `0.035 rad/s` from the 2 ms step-vs-sensor offset; rotating IMU to world increased mean diff to `0.096 rad/s` and flipped pitch-rate sign).
  2. **Effort finding (`mujoco_system_interface.cpp:1052`, `mujoco_simulation.cpp:1396`):** `MujocoSimulation::publish_control_state()` snapshots `mj_data_->qfrc_actuator`, and `MujocoSystemInterface::read()` copies `control_state_.qfrc_actuator[mj_vel_adr]` into each joint's `effort` state interface, including for `<velocity>` actuators under `implicitfast`. Adding `<state_interface name="effort"/>` to the 4 wheel joints causes `joint_state_broadcaster` to publish real MuJoCo actuator torques on `/joint_states` (confirmed non-zero on `4036/4040` samples on `flat` and `4040/4040` on `mars`, peaking at the `±15.0 N·m` actuator `forcerange` during turns).
- **Watch:** `front_axle_joint` is passive and has no effort interface, so `joint_state_broadcaster` fills its `effort` slot with `NaN`; `metrics_node` filters by wheel joint name and finite values.

## 2026-11-25 · Phase 4 · C++ `mars_yard_metrics` package (pure core + `metrics_node`) and baseline numbers

- **Decision:** created `src/mars_yard_metrics` (`ament_cmake`, C++17) split into a ROS-free static library (`metrics_core.hpp/.cpp`, tested by `test_metrics_core` gtest) and `metrics_node.cpp` (started by `sim.launch.py` when `metrics:=true`, default `true`), plus `tools/check_metrics.sh [flat|mars]`.
- **Alternatives:** (a) put metric math directly inside ROS callbacks (hard to unit-test without spinning a ROS graph); (b) publish a custom `RoverMetrics.msg` interface package (adds an extra build dependency for downstream tools).
- **Why:** keeping geometry, slip, and energy accumulators as pure C++ structs/functions allows fast deterministic unit tests (`ament_cmake_gtest`, 7 test cases covering `±π` wrap, quaternion world→body rotation, planar `relative_pose`, polyline segment projection/endpoints/empty path, zero/reverse/in-place slip, and mixed-sign energy). Publishing 10 `std_msgs/Float64` topics at 10 Hz (`rclcpp::create_timer` on sim clock) works directly with `ros2 topic echo`, PlotJuggler, and MCAP bags without custom message types.
- **Measured headless `tools/check_metrics.sh` (10 s fwd at 0.4 m/s + 10 s turn at 0.5 rad/s, straight `/plan` from `(0,0)` to `(10,0)` every 0.1 m, RTF `1.0000×`):**

  | World & segment | GT `(x, y)` m | `true_distance` m | `ekf_pos_drift` m | `ekf_yaw_drift` deg | `wheel_pos_drift` m | `wheel_yaw_drift` deg | `slip_ratio` | `lateral_slide` m | `cross_track_error` m | `energy` J |
  |---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
  | `flat` · 10 s fwd | `(3.956, +0.003)` | `3.952` | `0.0014` | `+0.00°` | `0.0256` | `-0.04°` | `+0.0076` | `0.0008` | `0.0034` | `30.09` |
  | `flat` · +10 s turn | `(4.061, -0.028)` | `4.301` | `0.0707` | `+0.05°` | `0.0695` | `-52.48°` | `+0.0014` | `0.1824` | `0.0279` | `464.82` |
  | `mars` · 10 s fwd | `(3.984, +0.031)` | `3.957` | `0.0199` | `-0.07°` | `0.0557` | `-0.50°` | `+0.0094` | `0.0083` | `0.0383` | `25.97` |
  | `mars` · +10 s turn | `(4.907, +0.317)` | `5.006` | `0.9087` | `-0.13°` | `0.9190` | `-67.75°` | `-0.0239` | `0.5069` | `0.3253` | `469.52` |

- **Energy per metre / segment breakdown (strictly monotonic on both worlds):**
  - `flat` straight: `30.09 J / 3.952 m = 7.61 J/m` (consistent with wheel joint damping `0.05` + rolling friction + kinetic energy).
  - `mars` straight: `25.89 J / 3.949 m = 6.56 J/m` (lower than `flat` because driving `+x` from spawn descends `0.24 m` in `z`, so gravity assists forward motion).
  - `flat` 10 s turn: `434.73 J` (`43.46 J/s`, `1243 J/m` of incidental motion) — ~14× the 4 m straight cost due to lateral tire scrub.
  - `mars` 10 s turn: `443.55 J` (`44.35 J/s`, `422.5 J/m` of downhill slide) — ~17× the 4 m straight cost.
- **Watch:**
  - *Map/world shortcut:* `/ground_truth/odom` assumes `map == MuJoCo world` and EKF starts at spawn (`T0` captured on the first `(GT, est)` message pair). Revisit when SLAM/AMCL moves `map → odom` in Phase 5.
  - *Moving-snapshot EKF latency:* `/simulator/floating_base_state` updates at 100 Hz while `/odometry/filtered` updates at 50 Hz (~`11.8 ms` mean lag, `30 ms` max). At `0.4 m/s` this shifts `latest_ekf_` behind `latest_gt_` by `~5–8 mm`, which partially cancels EKF's `+9 mm` forward overshoot on `flat` (`0.0014 m` live vs `0.009 m` at matched timestamps).
  - *In-place turn slip ratio:* `lateral_slide` clearly exposes the Mars downhill slide (`0.507 m` vs `0.182 m` on `flat`). During the Mars in-place turn, brief left/right wheel-speed asymmetries push `|v_wheel|` slightly above `0.02 m/s` while gravity slides the rover faster longitudinally than the wheels roll, pulling cumulative `slip_ratio` negative (`-0.024`).
  - *Verified regression checks:* `colcon test --packages-select mars_yard_metrics` (7 gtests passed), `pytest tools/tests -q` (16 passed), and `tools/check_bringup.sh flat` (RTF `1.0000×`, all topic rates nominal).
  - *Run-to-run spread on Mars:* an independent rerun of `check_metrics.sh mars` gave `ekf_pos_drift 0.990 m` (vs `0.909 m`), `lateral_slide 0.487 m`, final GT yaw `-18.2?` (vs `-17.5?`). The downhill slide during the in-place turn is contact-chaotic, so Phase 6 must report means over seeds, not single runs.
  - *`lateral_slide` is not zero on flat ground (`0.18 m` per 10 s turn):* it integrates `|v_y|` of `base_link`, and a skid-steer turn scrubs about a centre that is not exactly `base_link`. Read it as "sideways motion of the base", and compare worlds, not absolute values.

## 2026-11-25 · Phase 5a · 3D lidar mode in `RayScanPlugin` and forward downward-tilted sensor layout

- **Decision:** extended `mars_yard_sim_plugins/RayScanPlugin` with `num_rings` (default `1`, unchanged 2D `/scan`), `min_elevation`, `max_elevation` (rad), and `cloud_topic` (default `"/points"`). When `num_rings > 1`, all `num_rings × num_rays` rays are cast in a single `mj_multiRay` call and published as hits-only float32 `(x, y, z)` `sensor_msgs/PointCloud2` in `lidar_link` via `realtime_tools::RealtimePublisher` with sim-time stamps (and optional 2D `/scan` from the ring closest to 0 rad when `topic` is non-empty). Configured **two** instances in `src/mars_yard_bringup/config/mujoco_plugins.yaml`:
  1. `ray_scan_plugin`: unchanged 1-ring 360° `/scan` at `10 Hz` (`range_max = 10.0 m`).
  2. `ray_cloud_plugin`: **28 rings × 91 rays = 2,548 rays** over forward azimuth `[-90°, +90°]` (`2.0°` step) and downward-concentrated elevation `[-30°, +2°]` (`1.19°` step), `range = [0.25, 8.0] m`, `5 Hz`, `topic: ""`, `cloud_topic: "/points"`.
- **Alternatives:**
  - *Single 360° 3D instance (`16 × 180 = 2,880` rays over `[-30°, +2°]`):* benchmarked at `26.1–28.9 ms/scan` on `mars.xml` and produced **`100` self-hits/scan** on the rover's own rear chassis (because the mast sits at `x = +0.30 m` and the chassis rear edge at `x = -0.40 m, z = +0.10 m` intercepts backward rays steeper than `-23.2°`, while `bodyexclude` only excludes `lidar_link`).
  - *20 rings × 121 rays (`[-32°, +2°]`, `1.79°` vertical step):* `14.7 ms/scan` with zero self-hits, but at `4.0 m` ahead the `0.42°`-tall top cap (`z ∈ [0.12, 0.15] m`) of a `0.15 m` rock fell between adjacent rings (`-6.95°` hit at `z = 0.085 m`, `-5.16°` cleared the top), leaving `0` hazard cells at `4.0 m`.
  - *32 rings × 81 rays (`[-30°, +2°]`, `1.03°` step):* `15.3 ms/scan`, still straddled the ellipsoid apex at `3.5–4.0 m` on a single static frame (`maxZ = 0.110–0.119 m`).
- **Why (measured numbers):**
  - *Zero chassis self-hits:* restricting azimuth to `[-90°, +90°]` keeps all rays ≥ `18°` clear of the front chassis (`-48.1°`) and front wheels (`-53.0°`). Keeping `ray_scan_plugin` as a separate 1-ring instance preserves full 360° `/scan` at `10 Hz` for existing tools and future 2D consumers.
  - *0.15 m rock detection across 1–4 m ahead (`/tmp` flat-world test, `28 × 91`):*
    - Ellipsoid (`0.15 m` exposed): detected at `1.0 m` (`20` hits, `4` cells), `2.0 m` (`19` hits, `2` cells), `3.0 m` (`8` hits, `1` cell), `3.5 m` (`7` hits, `1` cell), `4.0 m` (`5` hits, `1` cell, `maxZ = 0.154 m`).
    - Box (`0.15 m` tall): detected at `1.0 m` (`14` cells), `2.0 m` (`8` cells), `3.0 m` (`3` cells), `3.5 m` (`3` cells), `4.0 m` (`3` cells, `maxZ = 0.156 m`).
  - *Performance on `mars.xml`:* single 3D `mj_multiRay` takes **`15.25 ms` median (`16.64 ms` p95)**. In full headless `tools/check_bringup.sh mars` with both plugins active, physics **RTF = `1.0000×`**, `/scan = 9.81 Hz`, `/diff_drive_controller/odom = 50.01 Hz`, `/joint_states = 48.87 Hz`, `/imu = 93.38 Hz` (small ~6.6% IMU rate dip from the 15 ms 3D scan overlapping one 10 ms control tick 5 times/s).
- **Watch:** if a future task needs rear-hemisphere 3D hazard detection while reversing, either mount a second rear sensor site or filter chassis-box hits in `base_link` before mapping.

## 2026-11-25 · Phase 5a · IMU gravity levelling for 3D clouds under a 2D EKF

- **Decision:** `mars_yard_nav/hazard_core.py` (`transform_points_to_odom()`) levels every `/points` cloud explicitly by chaining: (1) static `base_link ← lidar_link` from tf2, (2) IMU roll and pitch `R_level = Ry(pitch) @ Rx(roll)` from `/imu_sensor_broadcaster/imu` (nearest in sim time from a 400-sample ring buffer; IMU yaw is discarded), and (3) planar `(x, y, yaw)` from tf2 `odom ← base_link`.
- **Alternatives:** (a) switch the EKF to `two_d_mode: false` and transform clouds directly via `tf2` `odom ← lidar_link`; (b) fit planes directly in the tilted `lidar_link` frame and rotate cell coordinates afterward.
- **Why:**
  1. Switching the EKF to 3D mode would alter EKF state behaviour (explicitly out of scope) and introduce unobservable `z` drift in `odom` without an altimeter.
  2. Transforming through the 2D `odom → base_link` TF leaves the rover's real roll/pitch tilt baked into the cloud. Because the rover settles on a ~5.1° slope at Mars spawn (and drives across slopes up to 20°), projecting unlevelled points into `odom` tilts the whole point cloud and breaks vertical step-height geometry whenever the rover pitches over terrain transitions.
  3. Discarding IMU yaw preserves the Phase 3 realism rule (no magnetometer on Mars; heading comes strictly from EKF gyro integration).
  4. Verified analytically and in `tools/tests/test_hazard_core.py::test_levelling_with_known_roll_and_pitch`: under `roll = +12°`, `pitch = -18°`, `yaw = +42°`, a flat ground plane is recovered at constant `z = -0.275 m` (`atol = 1e-9 m`) with exact `(x, y)` coordinates.
- **Watch:** `z` in the leveled `odom` frame is always relative to the rover's instantaneous `base_link` height (unknown in 2D EKF mode). Never accumulate raw `z` values across scans—detect step height per scan and store only discrete hazard/free labels in `odom`.

## 2026-11-25 · Phase 5a · Slope-invariant 2-pass local plane step-height detector and rolling label map

- **Decision:** created package `src/mars_yard_nav` (`ament_cmake` + `ament_cmake_python`) with pure-NumPy core (`mars_yard_nav/hazard_core.py`) and ROS 2 node (`scripts/hazard_mapper.py`, config `config/hazard_mapper.yaml`, started by `sim.launch.py` when `hazards:=true`, default `true`). Per scan, leveled points are binned into `0.10 m` cells retaining the exact `(x, y, z)` of both the lowest and highest point in each cell. A vectorized integral-image 2D least-squares plane is fitted over a `±0.70 m` (`1.5 m × 1.5 m`) window in two passes:
  - **Pass 1:** fit local plane to all occupied cell minima `(x_min, y_min, z_min)`; reject elevated outlier cells with `z_min - z_plane > 0.06 m`.
  - **Pass 2:** refit the plane using only ground-inlier cells (requiring `>= 8` ground cells, 2D covariance `λ_min >= 0.015 m²` to reject single-ring arcs, and fitted slope `<= 35°`), then compute `step_height = max(z_max - z_plane(x_max, y_max), z_plane(x_min, y_min) - z_min)`.
  - Cells with `step_height > 0.12 m` (1 wheel radius) are marked hazard; cells with valid Pass-2 plane and `step_height <= 0.084 m` (`0.7 × threshold` hysteresis) are marked free. Only discrete labels (`-1 / 0 / 100`) and hysteresis scores are kept in a `24 m × 24 m` rolling `odom` grid (`memory_ttl = 30 s`), published on `/hazards` (`PointCloud2`, hazards at `z = 0.3`, free at `z = 0.0`) and `/hazard_grid` (`OccupancyGrid`).
- **Alternatives:**
  - *Single-pass OLS plane fit:* on a 0.5–1.2 m boulder, rock points fill 25–40% of the local window and pull the fitted plane upward, underestimating step height.
  - *Evaluating the plane at cell centres `(xc, yc)`:* on a 20° slope, a point at a `0.10 m` cell corner differs from cell-centre `z` by up to `0.026 m`, eating into the `0.12 m` margin. Evaluating `z_plane` at exact `(x_max, y_max)` makes the residual identically `0` on any noise-free tilted plane.
  - *Accumulating a persistent 2.5D elevation map in `odom`:* corrupts step heights as soon as the rover drives uphill/downhill because 2D EKF has no `z`.
- **Why (measured numbers):**
  - *Unit tests (`pytest tools/tests/test_hazard_core.py`, 6 cases, all pass):* 20° tilted plane across 4 azimuths with `σ = 8 mm` noise → `0` hazards; `0.15 m` box and ellipsoid on a 15° slope → cleanly detected (`>= 4` cells, max step `>= 0.135 m`, `0` false hazards); `0.08 m` bump on 10° slope → `0` hazards; rolling memory preserves world coordinates across rover motion and expires after `memory_ttl`.
  - *End-to-end `tools/check_hazards.sh`:*
    - `flat`: RTF `1.0000×`, `/points` `5.00 Hz` (`2002` pts/scan), `/hazards` `5.00 Hz`, `0` hazards, `0` false hazards (`6,275` free cells).
    - `mars`: RTF `1.0002×`, `/points` `4.92 Hz` (`1,967` pts/scan), `/hazards` `4.92 Hz`; settled on the `~5.1°` spawn slope (`roll -1.06°, pitch +5.10°`): **`0` false hazards** (`8` true hazard cells already seeing `boulder_0` at `5.63 m`). After driving to `2.43 m` in front of `boulder_0` (`EKF err = 0.024 m`): `64` hazard cells, `6,727` free cells, **`boulder_0` (`0.465 m`) DETECTED (`43` cells, `0.000 m` surface dist)**, **`boulder_6` (`0.352 m` at `4.18 m`) DETECTED (`6` cells, `0.000 m` surface dist)**, `rock_13` (`0.059 m`, below threshold) correctly ignored, and **`0` false hazards** (`> 0.5 m` from any rock surface).
- **Watch:** if a boulder is wider than the `1.5 m` window diameter (e.g. `> 1.5 m` across), its centre has no ground neighbours in Pass 2; the Pass-1 fallback still flags its perimeter, which is sufficient for Nav2 obstacle inflation. Tune `window_radius_m` if larger geological outcrops are added later.

## 2026-11-25 · Phase 5a · Boulders added to `mars.xml` from a separate RNG stream

- **Decision:** added `--boulders` (default `8`) and `place_boulders()` to `tools/make_mars_world.py` using an independent RNG (`seed + 10_000`), placing 8 ellipsoid boulders (`boulder_0 … boulder_7`) with `0.30–0.50 m` exposed height (`sz = exposed_h / 0.7`, 30% buried, `group = 0`, `material = "rock"` with alpha `1.0`). At least 2 boulders are placed within `3–10 m` of spawn on drivable ground (`slope >= 200`), outside the `3.0 m` spawn clearance and clear of the `+x` straight-drive verification corridor (`[-0.8, 5.5] × [-1.5-sx, +1.5+sx]`) so `check_bringup.sh` and `check_metrics.sh` remain collision-free. Boulders are appended to `obstacles.json["rocks"]` with `"kind": "boulder"`.
- **Alternatives:** (a) draw boulders from the same RNG stream after `place_sand()` (safe only until `place_rocks`/`place_sand` change); (b) place boulders in a separate JSON key (risks downstream obstacle scripts forgetting to iterate both lists).
- **Why:** all 40 existing rocks were `0.04–0.20 m` exposed and none lay within `6.5 m` of spawn (`rock_13` at `6.53 m` is only `0.059 m` tall). With `seed = 7`, the separate stream preserves all 40 original rocks and 3 sand patches at their exact coordinates (verified by `test_boulders_present_rayhit_and_preserve_existing_obstacles` in `tools/tests/test_mars_world.py`, 23 pytest tests passing) and places 3 boulders within `10 m` of spawn: `boulder_0` (`5.63 m, az -7.1°, exp_h 0.465 m`), `boulder_6` (`7.38 m, az -2.7°, exp_h 0.352 m`), and `boulder_1` (`8.75 m, az -44.1°, exp_h 0.344 m`). Regression `tools/check_metrics.sh mars` still runs at `RTF = 1.0000×` (`ekf_pos_drift = 0.0197 m` after 10 s straight drive, `energy_monotonic = True`).
- **Watch:** `obstacles.json["rocks"]` now contains `48` entries (`40` rocks + `8` boulders). Filter by `r.get("kind") == "boulder"` when selecting large hazards specifically.

## 2026-11-25 · Phase 5b · Nav2 stack layout (no AMCL, static `map → odom`, slope + lidar costmap layers)

- **Decision:** added Nav2 configs in `src/mars_yard_nav/config/` (`nav2_base.yaml` plus planner/controller overlays) and `src/mars_yard_nav/launch/nav.launch.py`, included optionally from `sim.launch.py` when `nav:=true` (default `false` so existing checks stay unchanged). Launched `map_server`, `planner_server`, `controller_server`, `behavior_server`, `bt_navigator`, `velocity_smoother`, and `lifecycle_manager_navigation`, all with `use_sim_time: true` and `odom_topic: /odometry/filtered`. **No AMCL:** `map → odom` remains the static identity TF from `sim.launch.py`. Generated an all-free 50 × 50 m `flat.pgm`/`flat.yaml` (`tools/make_flat_map.py`, identical 0.1 m / `[-25, -25]` / `mode: scale` layout to `slope.pgm`) for the `flat` world.
  - **Global costmap (`map`, 50 × 50 m, 0.1 m):** `static_layer` (`trinary_costmap: false` so continuous 10°–20° slope costs from `mode: scale` are preserved instead of flattened to binary free/lethal) + `obstacle_layer` (`/hazards`, `sensor_frame: base_link`, `min_obstacle_height: 0.1`, `max_obstacle_height: 0.5`, `marking: true`, `clearing: true`) + `inflation_layer` (`inflation_radius: 1.0`, `cost_scaling_factor: 3.0`). Footprint `[[0.42, 0.38], [0.42, -0.38], [-0.42, -0.38], [-0.42, 0.38]]` (+ `0.02 m` padding).
  - **Local costmap (`odom`, rolling 8 × 8 m, 0.1 m):** `static_layer` (`trinary_costmap: false`) + `obstacle_layer` (`/hazards`) + `inflation_layer` (`1.0 m`). Including `static_layer` in the local costmap ensures local controllers respect orbital steep-slope lethal zones (`>= 20°`), which `hazard_mapper` intentionally excludes as smooth terrain.
  - **Command chain:** `controller_server` and `behavior_server` publish to `cmd_vel_nav` → `velocity_smoother` (`max_velocity: [0.42, 0.0, 0.75]`) → `/cmd_vel` → `twist_stamper.py` → `/diff_drive_controller/cmd_vel`.
- **Alternatives:**
  - *Run AMCL on 2D `/scan`:* rejected — on open Mars slopes the horizontal 2D scan returns mostly `+inf` or uphill terrain hits with no walls/landmarks, so scan-matching particle filters cannot localize. Keeping static `map → odom` lets EKF drift accumulate transparently so Phase 6 measures it directly.
  - *Omit `static_layer` from `local_costmap`:* rejected — a local controller dodging a boulder near a steep crater rim would be blind to orbital slope limits.
  - *Leave `sensor_frame: ""` on the `/hazards` `ObstacleLayer`:* **critical gotcha avoided** — `/hazards` is stamped in `odom`, so an empty `sensor_frame` makes `ObservationBuffer` raytrace free-space clearing from `odom` origin `(0, 0)` instead of from the rover, which would erase boulders behind the rover's line of sight from spawn. Setting `sensor_frame: base_link` anchors raytracing at the rover.
- **Why:**
  - *Stock TB3 sanity demo (`nav2_bringup tb3_simulation_launch.py headless:=True use_rviz:=False` + `nav2_simple_commander`):* Gazebo Sim + Nav2 ran headless in WSL (`tb3_sandbox`, initial pose `(-2.0, -0.5)` → goal `(0.5, 0.5)`), reaching the goal with **`TaskResult.SUCCEEDED` in `14.16 s`** (wall = sim, `RTF = 1.000×`, `0` recoveries).
  - *`controller_frequency: 10.0 Hz`:* keeps CPU load low enough that MuJoCo + 28-ring 3D lidar + `hazard_mapper` + full Nav2 (including 1000-sample MPPI) all run at **`RTF = 0.9998–1.0000×`** (well above the `>= 0.9` requirement).
  - *RViz (`src/mars_yard_bringup/rviz/sim.rviz`):* switched `Fixed Frame` to `map`, added `Map` (`/map`), `Global Costmap`, `Local Costmap`, `Global Plan` (`/plan`), `Local Plan` (`/local_plan`), `Footprint` (`/local_costmap/published_footprint`), and `rviz_default_plugins/SetGoal` publishing to `/goal_pose`.
- **Watch:**
  - *FastDDS startup discovery race:* `BasicNavigator.waitUntilNav2Active()` calls `/bt_navigator/get_state` without a per-call timeout and can hang if a single startup response is dropped. `tools/check_nav_runner.py` warms up `/bt_navigator/get_state` with bounded 2 s retries before calling `waitUntilNav2Active(navigator="bt_navigator", localizer="robot_localization")`.

## 2026-11-25 · Phase 5b · Overlay mechanism, tuning log, and 6-combo Mars matrix

- **Decision:** `nav.launch.py` uses an `OpaqueFunction` to deep-merge `nav2_base.yaml` + `planner_<planner>.yaml` (`navfn` | `smac_hybrid`) + `controller_<controller>.yaml` (`dwb` | `mppi` | `rpp`) plus the world's map path (`slope.yaml` or `flat.yaml`) into a single resolved temporary YAML file passed to all Nav2 nodes. Selected **`planner:=smac_hybrid`, `controller:=dwb`** as the default combo in `sim.launch.py`, `nav.launch.py`, and `tools/check_nav.sh`.
- **Alternatives:** passing multiple YAML files directly in `Node(parameters=[...])` (harder to inspect resolved nested keys like `planner_server.ros__parameters.GridBased`); or selecting `navfn + dwb` / `navfn + rpp` as default (wider boulder clearance `0.87–0.92 m`, slightly higher GT error `0.36–0.47 m` and energy `454–512 J`).
- **Why (single-variable tuning log on `mars`, goal `(9.0, -0.6, yaw=0)` behind `boulder_0` at `(5.59, -0.69)` and `boulder_6` at `(7.37, -0.34)`, `slope.pgm` verified `254` free in a 1.3 m patch around goal and `1.12 m` from nearest boulder surface):**
  - *Baseline (all 6 combos):* `navfn+dwb` (`28.96 s`, GT err `0.419 m`, min boulder `0.951 m`, `479 J`), `navfn+rpp` (`28.16 s`, `0.437 m`, `0.958 m`, `539 J`), and `smac_hybrid+dwb` (`27.46 s`, `0.187 m`, `0.554 m`, `435 J`) all succeeded cleanly with `0` recoveries. However, `navfn+mppi` (`60.55 s`, GT err `4.711 m`, `1916 J`) and `smac_hybrid+mppi` (`110.06 s`, `4` rec, GT err `10.620 m`, `4315 J`) slid far downhill while turning near the goal, and `smac_hybrid+rpp` (`62.66 s`, GT err `2.601 m`, `1499 J`) missed the `0.25 m` goal circle on pass 1 and chased two `3.8 m` Dubin orbits.
  - **Step 1 (`controller_mppi.yaml`, `vx_min: -0.15 → 0.0`, kept):** forbade reverse dithering on the slope. `smac_hybrid+mppi` improved sharply: `110.06 → 49.15 s`, recoveries `4 → 0`, GT err `10.620 → 3.271 m`, lateral slide `9.031 → 2.472 m`, energy `4315 → 1392 J`. `navfn+mppi` unchanged (`62.56 s`, `4.815 m`).
  - **Step 2 (`controller_mppi.yaml`, `time_steps: 40 → 24`, reverted):** tested shorter `2.4 s` horizon; `navfn+mppi` improved marginally (`57.86 s`, `4.140 m`), but `smac_hybrid+mppi` regressed to `6` recoveries stuck near `boulder_6`. Reverted to `40`.
  - **Step 3 (`controller_mppi.yaml`, `CostCritic.consider_footprint: true → false`, reverted):** regressed both combos (`navfn+mppi` `74.26 s` / `6.473 m`; `smac_hybrid+mppi` `FAILED` at `26.52 s` with `"Start occupied"` near `boulder_6`). Reverted to `true`.
  - **Diagnostic (read-only `/cmd_vel` + yaw trace on `navfn+mppi`):** revealed the true cause of MPPI's downhill slide. At `wz = -0.52 rad/s` commanded, `wheel_separation_multiplier: 2.94` over-turned the rover at `-1.05 rad/s` actual, overshooting the target heading to `-120°`; MPPI's warm-start then remained pinned at `wz = -0.65 rad/s` (`> 2σ` from `wz_std = 0.30`) for **six full 360° pirouettes** while sliding `+4.7 m` downhill.
  - **Step 4 (`controller_mppi.yaml`, enable `TwirlingCritic` weight `10.0`, reverted):** over-penalized turning near the goal so `navfn+mppi` stalled `0.34 m` sideways of `(9.0, -0.6)` until `180 s` timeout (`8` recoveries). Reverted.
  - **Step 5 (`controller_mppi.yaml`, `wz_max: 0.65 → 0.35`, kept):** capped rotation to gentle skid-steer rates within `1.2σ` of `wz_std: 0.30`, eliminating MPPI pirouettes. `navfn+mppi` dropped from `62.56 → 50.46 s`, GT err `4.815 → 2.139 m`, lateral slide `4.152 → 1.801 m`, energy `2095 → 765 J`, `wheel_yaw_drift` `-164° → +12°`. (`smac_hybrid+mppi` still hit `"no valid path found"` near `boulder_6` due to `SmacPlannerHybrid` hugging the inflated edge.)
  - **Step 6 (`planner_smac_hybrid.yaml`, `cost_penalty: 2.5 → 5.0`, kept):** increased obstacle-cost avoidance so Hybrid-A* routes wider around `boulder_6`. `min_boulder_surf` improved across all three `smac_hybrid` combos (`0.554 → 0.624 m` DWB, `0.651 → 0.695 m` RPP, `0.548 → 0.715 m` MPPI); `smac_hybrid+mppi` succeeded (`90.52 s`, `3.520 m` GT err) but still looped near the goal.
  - **Step 7 (`planner_smac_hybrid.yaml`, `minimum_turning_radius: 0.60 → 0.35`, kept):** tightened Dubin turning radius to match the skid-steer track (`0.64 m`), cutting the re-entry orbit diameter nearly in half. Huge gains across all `smac_hybrid` combos: `smac_hybrid+mppi` `90.52 → 35.76 s`, recoveries `13 → 0`, GT err `3.520 → 0.654 m`, energy `1225 → 518 J`; `smac_hybrid+rpp` `91.68 → 38.06 s`, GT err `4.260 → 1.154 m`, energy `2332 → 847 J`; `smac_hybrid+dwb` `28.25 s`, `0.273 m` GT err, `0.685 m` clearance, `405 J`.
  - **Step 8 (`controller_rpp.yaml`, `min_lookahead_dist: 0.6 → 0.40`, kept):** improved tight-arc tracking for RPP. `smac_hybrid+rpp` dropped to `29.55 s`, GT err `0.413 m`, lateral slide `0.222 m`, energy `506 J` (`0` rec) on the tuning run, while `navfn+rpp` stayed at `28.25 s` (`0.909 m` clearance).
- **Final 6-combo matrix on `mars` (goal `(9.0, -0.6, 0°)`, timeout `180 s` sim) + default combo on `flat` (`(6.0, 0.0, 0°)`):**

  | World | Planner | Controller | Result (rec) | Sim s | GT err (m) | GT / init `/plan` (m) | Min boulder surf (m) | EKF drift (m) | Lat slide (m) | Energy (J) | RTF |
  |---|---|---|---|---:|---:|---|---:|---:|---:|---:|---:|
  | `mars` | `navfn` | `dwb` | SUCCEEDED (0) | 29.06 | 0.360 | 10.06 / 9.28 | 0.871 | 0.429 | 0.305 | 453.6 | 1.0000× |
  | `mars` | `navfn` | `mppi` | SUCCEEDED (0) | 42.76 | 1.441 | 11.16 / 9.28 | 0.927 | 1.413 | 1.139 | 629.8 | 0.9999× |
  | `mars` | `navfn` | `rpp` | SUCCEEDED (0) | 28.15 | 0.474 | 10.21 / 9.28 | 0.915 | 0.508 | 0.336 | 512.0 | 0.9998× |
  | `mars` | **`smac_hybrid`** | **`dwb`** | **SUCCEEDED (0)** | **28.15** | **0.222** | **9.93 / 9.27** | **0.614** | **0.327** | **0.245** | **448.8** | **0.9998×** |
  | `mars` | `smac_hybrid` | `mppi` | SUCCEEDED (0) | 39.15 | 1.034 | 10.58 / 9.33 | 0.632 | 1.222 | 1.048 | 576.8 | 0.9999× |
  | `mars` | `smac_hybrid` | `rpp` | TIMEOUT* (0) | 180.09 | 8.938 | 48.52 / 9.27 | 0.775 | 9.724 | 7.337 | 5966.5 | 1.0000× |
  | `flat` | **`smac_hybrid`** | **`dwb`** | **SUCCEEDED (0)** | **17.46** | **0.241** | **5.76 / 6.00** | **∞ (none)** | **0.020** | **0.010** | **169.5** | **0.9999×** |

  *\*`smac_hybrid + rpp` intermittent failure mode:* succeeded on Step 8 (`29.55 s`, GT err `0.413 m`, `506.4 J`, clearance `0.563 m`), but timed out on the final rerun. When small slope-slip variation causes Pure Pursuit to miss the `0.25 m` goal circle by a few cm on pass 1, the 1 Hz forward-only Dubin replanner generates a closed teardrop loop around `(9.0, -0.6)` that Pure Pursuit chases indefinitely — because orbiting exceeds `required_movement_radius: 0.35 m`, `progress_checker` never fires a recovery. `smac_hybrid + dwb` (4/4 runs clean: `27.5–28.3 s`, `0.19–0.27 m` GT err, `405–450 J`, `0` recoveries) avoids this via DWB's `GoalDist` + `RotateToGoal` critics and is chosen as the default combo.
- **Watch:**
  - *Single-seed sensitivity:* as seen on `smac_hybrid + rpp`, contact-driven slope slip creates run-to-run variance on borderline controller/planner pairings; Phase 6 must benchmark multi-seed success rates rather than single trials.
  - *`smac_hybrid + dwb` vs `navfn + dwb` trade-off:* `smac_hybrid + dwb` achieves lower GT error (`0.22 m` vs `0.36 m`), drift (`0.33 m` vs `0.43 m`), lateral slide (`0.25 m` vs `0.31 m`), and energy (`449 J` vs `454 J`), while `navfn + dwb` keeps wider boulder clearance (`0.87 m` vs `0.61 m`). Both clear all boulders safely (`>= 0.42 m` collision threshold).

## 2026-11-25 · Phase 5 · Fix: goals aborted mid-drive under RViz load

- **Decision:** raise `bt_navigator.default_server_timeout` from `20` to `1000` ms in `src/mars_yard_nav/config/nav2_base.yaml`.
- **Alternatives:** lower planner replanning rate or drop RViz displays (treats the symptom; any CPU spike can still abort a goal).
- **Why:** in an interactive session with RViz (3D `/points`, both costmaps) 9 of ~20 goals stopped mid-drive. `~/.ros/log/bt_navigator_*.log` showed every one as `Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose|follow_path` -> `Goal failed`; the planner itself missed its rate only once. 20 ms is the time the BT waits for the action server to *acknowledge* a request (the 1 Hz replan or the controller hand-off), not planning time, so it only fails when the machine is busy. A 1 s acknowledge window does not slow normal operation.
- **Watch:** the headless `tools/check_nav.sh` never hit this (no RViz load), so it is only verified by rerunning interactively.

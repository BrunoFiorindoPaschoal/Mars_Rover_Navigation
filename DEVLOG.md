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

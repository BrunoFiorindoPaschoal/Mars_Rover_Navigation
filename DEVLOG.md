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
- **Spawn convention:** the terrain is shifted so that the ground under (0, 0) is at z = 0. The rover's spawn height in `rover.xml` (0.275 m) is then correct in every world, and the Nav2 map frame sits on the ground.
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

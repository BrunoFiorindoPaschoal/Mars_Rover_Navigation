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
| 4 | Rover MJCF/URDF + contact physics | Explains slip and jitter, and every number in the model | **done**, see Part 4 |
| 5 | Terrain pipeline (DEM -> hfield + slope map) | Planning on orbital data | **done**, see Part 5 |
| 6 | C++ metrics node | rclcpp, CMake, tf2 lookups | planned (Phase 4) |
| 7 | Benchmark runner + analysis | Evaluation protocol | planned (Phase 6) |
| 8 | Docker + CI | Reproducibility | planned (Phase 8) |

Sections for each part are filled in as the code lands.

---

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

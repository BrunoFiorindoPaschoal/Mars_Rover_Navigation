# Mars Yard in MuJoCo

A skid-steer rover on HiRISE Jezero crater terrain, simulated in MuJoCo and driven autonomously by ROS 2 Jazzy Nav2 through `mujoco_ros2_control`.

> Work in progress. This README claims only what the repo currently shows.

![Rover driving open-loop on a 50 m HiRISE Jezero patch (MuJoCo offscreen render)](media/tier_a.gif)

*Current state (Phase 2): a fixed open-loop drive on the real terrain, with no ROS or navigation yet. Render it with `MUJOCO_GL=egl .venv/bin/python tools/render_tier_a.py`.*

## Setup (WSL2, Ubuntu 24.04)

```bash
./tools/setup_wsl.sh
```

Launch the ROS 2 + MuJoCo simulation (`world:=flat` or `world:=mars`):

```bash
ros2 launch mars_yard_bringup sim.launch.py world:=mars
```

## Navigate

Launch MuJoCo, EKF, 3D lidar hazard mapping, Nav2, and RViz2 in one command (default planner/controller: `smac_hybrid` + `dwb`):

```bash
ros2 launch mars_yard_bringup sim.launch.py world:=mars nav:=true
```

In RViz2, select the **2D Goal Pose** tool in the top toolbar, then click and drag anywhere on the map (in frame `map`) to send a goal to `/goal_pose`. Optional overlays: `planner:=navfn|smac_hybrid` and `controller:=dwb|mppi|rpp`.

Headless verification:

```bash
./tools/check_nav.sh mars smac_hybrid dwb
./tools/check_teleport.sh
```

## Benchmark

20 fixed scenarios on the Jezero Mars world (`src/mars_yard_benchmark/config/scenarios.yaml`, 5 per group: `flat`, `slope`, `rocks`, `sand`), evaluated across 6 planner × controller combinations (`navfn | smac_hybrid` × `dwb | mppi | rpp`) using in-sim rover resets (`~1.4 s` per reset vs `~16.8 s` per relaunch).

Run the full 600-run matrix (6 combos × 20 scenarios × 5 seeds, headless, resumable, ~5–6 h overnight):

```bash
./tools/run_matrix.sh --seeds 5
```

Or run a smaller subset (e.g., 1 seed on two scenarios):

```bash
./tools/run_matrix.sh --seeds 1 --scenarios slope_04,rocks_01
```

Produce summary tables (`results/benchmark/summary.md`) and plots (`results/benchmark/plots/`):

```bash
python3 -m mars_yard_benchmark.analysis \
  --csv results/benchmark/benchmark.csv \
  --out-md results/benchmark/summary.md \
  --plots-dir results/benchmark/plots
```

### Results (full 600-run matrix — pending overnight run)

| Planner × Controller | Flat | Slope | Rocks | Sand | Overall | Time (s) | Path ratio | Tracking err (m) | Energy (J) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `navfn + dwb` | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| `navfn + mppi` | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| `navfn + rpp` | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| `smac_hybrid + dwb` | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| `smac_hybrid + mppi` | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| `smac_hybrid + rpp` | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

Success means Nav2 `SUCCEEDED` **and** the ground-truth rover ends within 1.0 m of the goal; otherwise the run is a `goal_miss` (the EKF thought it arrived).

## Robustness

The benchmark under 4 randomization levels: friction, chassis mass and centre of mass, per-wheel motor gain (MJCF), lidar range noise and dropout, gyro noise and bias, command latency (relays). Level 0 is the unchanged benchmark stack.

```bash
ros2 launch mars_yard_bringup sim.launch.py world:=mars nav:=true rand_level:=2   # one randomized world
./tools/check_randomization.sh                                                   # perturbations measured in the running sim
./tools/run_robustness.sh --levels 0,1,2,3 --seeds 2                              # full study, ~11 h (960 runs)
python3 -m mars_yard_benchmark.robustness_analysis                               # results/robustness/summary.md + plots
```

Pilot (planner `smac_hybrid`; scenarios `flat_01`, `slope_04`, `rocks_01`, `sand_01` × 2 seeds; true success):

| Controller | L0 | L1 | L2 | L3 |
|---|---:|---:|---:|---:|
| DWB | 8/8 | 8/8 | 6/8 | 6/8 |
| MPPI | 4/8 | 4/8 | 5/8 | 4/8 |
| RPP | 8/8 | 7/8 | 7/8 | 5/8 |

22 of the 24 failures are `goal_miss` on the 14.5° cross-slope (`slope_04`) and in the rock field: with lower friction the rover slides downhill, and the EKF (wheel speed + gyro) cannot see it. See `results/robustness/plots/` and DEVLOG Phase 7.

## Docs
- [DEVLOG.md](DEVLOG.md): design decisions, with the reasons for each
- [STUDY_GUIDE.md](STUDY_GUIDE.md): the core parts of the code and how to study them

## Credits
HiRISE terrain data: NASA/JPL-Caltech/UArizona.

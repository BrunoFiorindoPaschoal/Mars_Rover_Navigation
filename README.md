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
```

## Docs
- [DEVLOG.md](DEVLOG.md): design decisions, with the reasons for each
- [STUDY_GUIDE.md](STUDY_GUIDE.md): the core parts of the code and how to study them

## Credits
HiRISE terrain data: NASA/JPL-Caltech/UArizona.

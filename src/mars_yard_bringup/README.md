# mars_yard_bringup

One launch file for the whole stack:

```bash
ros2 launch mars_yard_bringup sim.launch.py world:=mars nav:=true [planner:=navfn|smac_hybrid] [controller:=dwb|mppi|rpp] [rand_level:=0..3 rand_seed:=N] [rviz:=false headless:=true]
```

- `config/controllers.yaml`: `diff_drive_controller` (skid-steer), joint state and IMU broadcasters.
- `config/ekf.yaml`: `robot_localization`, 2D, fuses wheel twist + gyro rate (no absolute yaw: no compass on Mars).
- `config/mujoco_plugins.yaml`: the lidar and teleport plugins.
- `scripts/twist_stamper.py`: `/cmd_vel` Twist → TwistStamped for the controller; optional `latency_s`.
- `scripts/noise_relay.py`: lidar and gyro noise, only launched for `rand_level > 0`.

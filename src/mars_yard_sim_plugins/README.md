# mars_yard_sim_plugins

`mujoco_ros2_control` plugins (pluginlib, configured in `mars_yard_bringup/config/mujoco_plugins.yaml`).

- `RayScanPlugin`: lidar from one `mj_multiRay` call at a fixed sim-time rate. 1 ring → `/scan` (LaserScan); N rings → `/points` (PointCloud2, used for hazard detection).
- `TeleportPlugin`: subscribes `/sim/teleport` (`geometry_msgs/Pose2D`), moves the rover in `pre_step` on the physics thread (ground height by `mj_ray`, velocities and warm start zeroed) and acknowledges on `/sim/teleport_done`. Used by the benchmark to reset in ~1.4 s instead of relaunching.

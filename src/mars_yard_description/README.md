# mars_yard_description

The rover model, kept in two hand-written files that a test keeps consistent (`tools/tests/test_model_consistency.py`).

- `mjcf/rover.xml`: MuJoCo physics: 4 wheels on velocity actuators, front axle pivot, free joint `base_free_joint`, lidar and IMU sites.
- `urdf/rover.urdf.xacro`: frames for tf and the `ros2_control` block for `mujoco_ros2_control` (arg `mujoco_model:=` selects the world MJCF).

Every dimension is justified in DEVLOG (Phase 1).

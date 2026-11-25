# mars_yard_metrics

C++ node that grades every run from MuJoCo ground truth. ROS-free core (`metrics_core`, gtest in `test/`) + `metrics_node`.

- Publishes `/ground_truth/odom` and `/metrics/*`: EKF and wheel drift, slip, lateral slide, cross-track error to `/plan`, true distance, energy from wheel effort × velocity.
- Service `/metrics_node/reset` (`std_srvs/Empty`) zeroes accumulators and re-anchors drift at the current pose (used after a teleport).

`colcon test --merge-install --packages-select mars_yard_metrics`

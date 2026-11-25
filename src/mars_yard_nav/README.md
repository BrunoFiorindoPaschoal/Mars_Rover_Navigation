# mars_yard_nav

Nav2 for the rover, without AMCL (static `map → odom`, the EKF provides `odom → base_link`).

- `scripts/hazard_mapper.py` (+ `mars_yard_nav/hazard_core.py`): IMU-levelled `/points`, local plane fit, step height > 0.12 m = hazard; publishes `/hazards` for the costmap obstacle layer. Service `~/clear` empties its memory.
- `config/nav2_base.yaml` + one `planner_*.yaml` + one `controller_*.yaml`: `launch/nav.launch.py` deep-merges them into one params file. Global costmap = slope map (static layer, continuous cost) + hazards + inflation.

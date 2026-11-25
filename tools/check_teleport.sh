#!/usr/bin/env bash
# Headless verification for Stage A in-sim rover teleport and reset.
# Launches sim.launch.py (world:=mars nav:=true, default combo smac_hybrid + dwb)
# once, runs 3 teleports, a Nav2 goal from the 3rd pose, and the Phase 5b reference
# run after teleports, and reports reset wall time vs full relaunch wall time.

set -eo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"

source /opt/ros/jazzy/setup.bash
if [ ! -f "$REPO/install/setup.bash" ]; then
  ./build.sh
fi
source "$REPO/install/setup.bash"

cleanup() {
  pkill -f "ros2_control_node" 2>/dev/null || true
  pkill -f "robot_state_publisher" 2>/dev/null || true
  pkill -f "ekf_node" 2>/dev/null || true
  pkill -f "twist_stamper" 2>/dev/null || true
  pkill -f "static_transform_publisher" 2>/dev/null || true
  pkill -f "metrics_node" 2>/dev/null || true
  pkill -f "hazard_mapper" 2>/dev/null || true
  pkill -f "map_server" 2>/dev/null || true
  pkill -f "planner_server" 2>/dev/null || true
  pkill -f "controller_server" 2>/dev/null || true
  pkill -f "behavior_server" 2>/dev/null || true
  pkill -f "bt_navigator" 2>/dev/null || true
  pkill -f "velocity_smoother" 2>/dev/null || true
  pkill -f "lifecycle_manager" 2>/dev/null || true
  pkill -f "rviz2" 2>/dev/null || true
  pkill -f "mars_yard_bringup" 2>/dev/null || true
  pkill -f "mars_yard_nav" 2>/dev/null || true
  sleep 1
}
trap cleanup EXIT
cleanup

echo "=== check_teleport: world=mars nav=true (default smac_hybrid + dwb) ==="
export MUJOCO_GL=egl
export PYTHONUNBUFFERED=1
LAUNCH_LOG="/tmp/check_teleport_mars.log"
LAUNCH_START_WALL="$(python3 -c 'import time; print(f"{time.time():.6f}")')"

timeout 360 ros2 launch mars_yard_bringup sim.launch.py \
  world:=mars headless:=true rviz:=false use_ekf:=true metrics:=true hazards:=true \
  nav:=true > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

READY=0
for _ in $(seq 1 40); do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "ERROR: ros2 launch exited early. Last 40 lines of $LAUNCH_LOG:"
    tail -n 40 "$LAUNCH_LOG" || true
    exit 1
  fi
  OUT="$(timeout 5 ros2 control list_controllers 2>/dev/null || true)"
  if echo "$OUT" | grep -q "diff_drive_controller.*active" && \
     echo "$OUT" | grep -q "imu_sensor_broadcaster.*active" && \
     echo "$OUT" | grep -q "joint_state_broadcaster.*active"; then
    READY=1
    break
  fi
  sleep 1
done

if [ "$READY" -ne 1 ]; then
  echo "ERROR: ros2_control controllers did not become active. Last 40 lines of $LAUNCH_LOG:"
  tail -n 40 "$LAUNCH_LOG" || true
  exit 1
fi

set +e
timeout 300 python3 "$REPO/tools/check_teleport_runner.py" --launch-start-wall "$LAUNCH_START_WALL"
RC=$?
set -e

if [ "$RC" -ne 0 ]; then
  echo "=== Last 40 lines of $LAUNCH_LOG ==="
  tail -n 40 "$LAUNCH_LOG" || true
fi

kill "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true
exit "$RC"

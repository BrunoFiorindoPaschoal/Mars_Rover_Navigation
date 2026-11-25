#!/usr/bin/env bash
# Headless end-to-end verification for Nav2 on the Mars Yard rover (Phase 5b).
# Usage:
#   ./tools/check_nav.sh [world] [planner] [controller]
# Defaults:
#   world=mars, planner=smac_hybrid, controller=dwb

set -eo pipefail

WORLD="${1:-mars}"
PLANNER="${2:-smac_hybrid}"
CONTROLLER="${3:-dwb}"

if [ "$WORLD" = "flat" ]; then
  GOAL_X="6.0"
  GOAL_Y="0.0"
  GOAL_YAW="0.0"
elif [ "$WORLD" = "mars" ]; then
  GOAL_X="9.0"
  GOAL_Y="-0.6"
  GOAL_YAW="0.0"
else
  echo "Unsupported world '$WORLD'. Expected 'flat' or 'mars'." >&2
  exit 1
fi

SIM_TIMEOUT="180.0"
WALL_TIMEOUT="260"

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

echo "=== check_nav: world=$WORLD planner=$PLANNER controller=$CONTROLLER goal=($GOAL_X, $GOAL_Y, $GOAL_YAW) ==="
export MUJOCO_GL=egl
export PYTHONUNBUFFERED=1
LAUNCH_LOG="/tmp/check_nav_${WORLD}_${PLANNER}_${CONTROLLER}.log"
timeout "$((WALL_TIMEOUT + 40))" ros2 launch mars_yard_bringup sim.launch.py \
  world:="$WORLD" headless:=true rviz:=false use_ekf:=true metrics:=true hazards:=true \
  nav:=true planner:="$PLANNER" controller:="$CONTROLLER" > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

READY=0
for _ in $(seq 1 40); do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "ERROR: ros2 launch exited early. Last 40 lines of $LAUNCH_LOG:"
    tail -n 40 "$LAUNCH_LOG" || true
    exit 1
  fi
  OUT="$("$REPO/tools/list_controllers.sh" 2>/dev/null || true)"
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
python3 "$REPO/tools/check_nav_runner.py" \
  --world "$WORLD" \
  --planner "$PLANNER" \
  --controller "$CONTROLLER" \
  --goal-x "$GOAL_X" \
  --goal-y "$GOAL_Y" \
  --goal-yaw "$GOAL_YAW" \
  --sim-timeout "$SIM_TIMEOUT" \
  --wall-timeout "$WALL_TIMEOUT" \
  --repo "$REPO"
RC=$?
set -e

if [ "$RC" -ne 0 ]; then
  echo "=== Last 40 lines of $LAUNCH_LOG ==="
  tail -n 40 "$LAUNCH_LOG" || true
fi

kill "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true
exit "$RC"

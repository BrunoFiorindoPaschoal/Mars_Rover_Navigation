#!/usr/bin/env bash
# End-to-end verification for Phase 7 domain randomization (Levels 0-3).
#
# (a) Level 0: runs tools/check_nav.sh mars and tools/check_teleport.sh (bit-for-bit nominal).
# (b) Levels 1-3: launches sim.launch.py with rand_level:=L rand_seed:=L, prints params.json,
#     checks noisy topic wiring (noise_relay, EKF, hazard_mapper), verifies lidar noise/dropout,
#     IMU gyro noise/bias, command latency, and per-wheel motor gain inside the live sim,
#     then runs one Nav2 goal per level.
#
# Usage:
#   ./tools/check_randomization.sh [--skip-l0] [--levels 1,2,3]

set -eo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"

SKIP_L0=0
LEVELS_LIST="1,2,3"

while [ $# -gt 0 ]; do
  case "$1" in
    --skip-l0) SKIP_L0=1; shift ;;
    --levels) LEVELS_LIST="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,12p' "$0"
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      exit 1
      ;;
  esac
done

source /opt/ros/jazzy/setup.bash
if [ ! -f "$REPO/install/setup.bash" ]; then
  ./build.sh
fi
source "$REPO/install/setup.bash"

export MUJOCO_GL=egl
export PYTHONUNBUFFERED=1

cleanup() {
  pkill -f "ros2_control_node" 2>/dev/null || true
  pkill -f "robot_state_publisher" 2>/dev/null || true
  pkill -f "ekf_node" 2>/dev/null || true
  pkill -f "twist_stamper" 2>/dev/null || true
  pkill -f "noise_relay" 2>/dev/null || true
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
  pkill -9 -f "ros2_control_node|twist_stamper|noise_relay|ekf_node|hazard_mapper|metrics_node|bt_navigator" 2>/dev/null || true
}
trap cleanup EXIT

if [ "$SKIP_L0" -eq 0 ]; then
  echo "========================================================================"
  echo "Part (a): Level 0 regression checks (nominal stack, no relays)"
  echo "========================================================================"
  ./tools/check_nav.sh mars
  ./tools/check_teleport.sh
fi

IFS=',' read -r -a LEVELS <<< "$LEVELS_LIST"
for LEVEL in "${LEVELS[@]}"; do
  cleanup
  OUT_DIR="/tmp/check_rand_mars_L${LEVEL}_S${LEVEL}"
  rm -rf "$OUT_DIR"
  LAUNCH_LOG="/tmp/check_rand_L${LEVEL}.log"

  echo ""
  echo "========================================================================"
  echo "Part (b): Randomization Level L${LEVEL} (seed=${LEVEL})"
  echo "========================================================================"

  timeout 260 ros2 launch mars_yard_bringup sim.launch.py \
    world:=mars headless:=true rviz:=false use_ekf:=true metrics:=true hazards:=true \
    nav:=true planner:=smac_hybrid controller:=dwb \
    rand_level:="$LEVEL" rand_seed:="$LEVEL" rand_out_dir:="$OUT_DIR" \
    > "$LAUNCH_LOG" 2>&1 &
  LAUNCH_PID=$!

  READY=0
  for _ in $(seq 1 45); do
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      echo "ERROR: sim.launch.py exited early for L${LEVEL}. Tail of $LAUNCH_LOG:"
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
    echo "ERROR: controllers not ready for L${LEVEL}. Tail of $LAUNCH_LOG:"
    tail -n 40 "$LAUNCH_LOG" || true
    exit 1
  fi

  echo "--- params.json ($OUT_DIR/params.json) ---"
  cat "$OUT_DIR/params.json"

  echo "--- Verifying noisy topic wiring ---"
  NODES="$(timeout -k 2 8 ros2 node list --no-daemon 2>/dev/null || true)"
  if ! echo "$NODES" | grep -q "/noise_relay"; then
    echo "ERROR: /noise_relay node not running at L${LEVEL}!"
    exit 1
  fi
  EKF_INFO="$(timeout -k 2 8 ros2 node info --no-daemon /ekf_filter_node 2>/dev/null || true)"
  HM_INFO="$(timeout -k 2 8 ros2 node info --no-daemon /hazard_mapper 2>/dev/null || true)"
  if ! echo "$EKF_INFO" | grep -q "/imu_sensor_broadcaster/imu_noisy"; then
    echo "ERROR: /ekf_filter_node is not subscribed to /imu_sensor_broadcaster/imu_noisy!"
    echo "$EKF_INFO"
    exit 1
  fi
  if ! echo "$HM_INFO" | grep -q "/points_noisy" || ! echo "$HM_INFO" | grep -q "/imu_sensor_broadcaster/imu_noisy"; then
    echo "ERROR: /hazard_mapper is not subscribed to noisy topics!"
    echo "$HM_INFO"
    exit 1
  fi
  echo "Topic wiring OK: /noise_relay active, EKF -> /imu_sensor_broadcaster/imu_noisy, hazard_mapper -> /points_noisy + /imu_sensor_broadcaster/imu_noisy."

  set +e
  timeout 200 python3 "$REPO/tools/check_randomization_runner.py" \
    --level "$LEVEL" \
    --params-json "$OUT_DIR/params.json" \
    --repo "$REPO"
  RC=$?
  set -e

  if [ "$RC" -ne 0 ]; then
    echo "=== Last 40 lines of $LAUNCH_LOG ==="
    tail -n 40 "$LAUNCH_LOG" || true
  fi

  kill "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  cleanup

  if [ "$RC" -ne 0 ]; then
    exit "$RC"
  fi
done

echo ""
echo "=== check_randomization.sh: ALL CHECKS PASSED ==="

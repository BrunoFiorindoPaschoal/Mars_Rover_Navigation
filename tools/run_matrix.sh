#!/usr/bin/env bash
# Run the Mars Yard Nav2 benchmark matrix across planner x controller combos.
#
# Usage:
#   ./tools/run_matrix.sh [OPTIONS]
#   ./tools/run_matrix.sh [seeds] [scenario_filter]
#
# Options:
#   --seeds N             Number of seeds 0..N-1 (default: 5)
#   --scenarios FILTER    Comma-separated scenario IDs or groups (default: all 20)
#   --combos LIST         Comma-separated combos (default: all 6:
#                         navfn+dwb,navfn+mppi,navfn+rpp,smac_hybrid+dwb,smac_hybrid+mppi,smac_hybrid+rpp)
#   --output-csv PATH     CSV output path (default: results/benchmark/benchmark.csv)
#   --bags-dir PATH       MCAP bag directory (default: results/benchmark/bags)
#   --log-file PATH       Combined log path (default: results/benchmark/matrix.log)
#   --sim-timeout SEC     Per-run sim timeout in seconds (default: 120)
#   --max-retries N       Max relaunches per combo on crash or RTF < 0.9 (default: 3)
#   --no-bag              Disable MCAP bag recording

set -eo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"

SEEDS="5"
SCENARIO_FILTER=""
COMBOS_LIST="navfn+dwb,navfn+mppi,navfn+rpp,smac_hybrid+dwb,smac_hybrid+mppi,smac_hybrid+rpp"
OUTPUT_CSV="$REPO/results/benchmark/benchmark.csv"
BAGS_DIR="$REPO/results/benchmark/bags"
LOG_FILE="$REPO/results/benchmark/matrix.log"
SIM_TIMEOUT="120"
MAX_RETRIES="3"
NO_BAG_FLAG=""

if [ $# -gt 0 ] && [[ "$1" != --* ]]; then
  SEEDS="$1"
  shift
  if [ $# -gt 0 ] && [[ "$1" != --* ]]; then
    SCENARIO_FILTER="$1"
    shift
  fi
fi

while [ $# -gt 0 ]; do
  case "$1" in
    --seeds) SEEDS="$2"; shift 2 ;;
    --scenarios) SCENARIO_FILTER="$2"; shift 2 ;;
    --combos) COMBOS_LIST="$2"; shift 2 ;;
    --output-csv) OUTPUT_CSV="$2"; shift 2 ;;
    --bags-dir) BAGS_DIR="$2"; shift 2 ;;
    --log-file) LOG_FILE="$2"; shift 2 ;;
    --sim-timeout) SIM_TIMEOUT="$2"; shift 2 ;;
    --max-retries) MAX_RETRIES="$2"; shift 2 ;;
    --no-bag) NO_BAG_FLAG="--no-bag"; shift ;;
    -h|--help)
      sed -n '2,20p' "$0"
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      exit 1
      ;;
  esac
done

mkdir -p "$(dirname "$OUTPUT_CSV")" "$BAGS_DIR" "$(dirname "$LOG_FILE")"

source /opt/ros/jazzy/setup.bash
if [ ! -f "$REPO/install/setup.bash" ]; then
  ./build.sh
fi
source "$REPO/install/setup.bash"

export MUJOCO_GL=egl
export PYTHONUNBUFFERED=1

cleanup_stack() {
  pkill -f "ros2 bag record" 2>/dev/null || true
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
  # SIGTERM is not always honoured (spawners, lifecycle_manager); a survivor holds the spawner lock
  # or duplicates Nav2 node names in the next launch, so sweep every ROS node process.
  pkill -9 -f -- "--ros-args|controller_manager/spawner" 2>/dev/null || true
  pkill -9 -f "ros2cli.daemon" 2>/dev/null || true
  sleep 1
  # kill -9 leaves FastDDS shared-memory port locks in /dev/shm; the next launch's Nav2 nodes then hang
  # before creating their node. Wipe them, but only when no ROS process is alive.
  if ! pgrep -f -- "--ros-args" >/dev/null; then rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*; fi
}
trap cleanup_stack EXIT

IFS=',' read -r -a COMBOS <<< "$COMBOS_LIST"

exec > >(tee -a "$LOG_FILE") 2>&1
echo "========================================================================"
echo "Mars Yard Benchmark Matrix started: $(date -Iseconds)"
echo "  combos    : ${COMBOS[*]}"
echo "  seeds     : $SEEDS"
echo "  scenarios : ${SCENARIO_FILTER:-ALL}"
echo "  csv       : $OUTPUT_CSV"
echo "  bags      : ${NO_BAG_FLAG:-$BAGS_DIR}"
echo "========================================================================"

OVERALL_RC=0
for COMBO in "${COMBOS[@]}"; do
  PLANNER="${COMBO%%+*}"
  CONTROLLER="${COMBO##*+}"
  echo ""
  echo ">>> Combo: planner=$PLANNER controller=$CONTROLLER"

  ATTEMPT=1
  COMBO_DONE=0
  while [ "$ATTEMPT" -le "$MAX_RETRIES" ]; do
    cleanup_stack
    LAUNCH_LOG="/tmp/matrix_${PLANNER}_${CONTROLLER}_try${ATTEMPT}.log"
    echo "  Launching sim.launch.py (attempt $ATTEMPT/$MAX_RETRIES) -> $LAUNCH_LOG"

    ros2 launch mars_yard_bringup sim.launch.py \
      world:=mars headless:=true rviz:=false use_ekf:=true metrics:=true hazards:=true \
      nav:=true planner:="$PLANNER" controller:="$CONTROLLER" > "$LAUNCH_LOG" 2>&1 &
    LAUNCH_PID=$!

    READY=0
    for _ in $(seq 1 45); do
      if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "  ERROR: ros2 launch exited early on attempt $ATTEMPT. Tail of $LAUNCH_LOG:"
        tail -n 30 "$LAUNCH_LOG" || true
        break
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
      echo "  WARNING: controllers not ready on attempt $ATTEMPT; relaunching..."
      kill "$LAUNCH_PID" 2>/dev/null || true
      wait "$LAUNCH_PID" 2>/dev/null || true
      ATTEMPT=$((ATTEMPT + 1))
      continue
    fi

    set +e
    # Generous outer timeout: enough for up to 100 runs x ~150s, while run_benchmark enforces per-run timeouts.
    python3 -m mars_yard_benchmark.run_benchmark \
      --planner "$PLANNER" \
      --controller "$CONTROLLER" \
      --seeds "$SEEDS" \
      --scenario-filter "$SCENARIO_FILTER" \
      --output-csv "$OUTPUT_CSV" \
      --bags-dir "$BAGS_DIR" \
      --sim-timeout "$SIM_TIMEOUT" \
      --repo "$REPO" \
      $NO_BAG_FLAG
    RC=$?
    set -e

    kill "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
    cleanup_stack

    if [ "$RC" -eq 0 ]; then
      COMBO_DONE=1
      break
    elif [ "$RC" -eq 3 ]; then
      echo "  WARNING: stack unhealthy / RTF < 0.9 for $COMBO (attempt $ATTEMPT/$MAX_RETRIES); relaunching and resuming..."
      ATTEMPT=$((ATTEMPT + 1))
    else
      echo "  ERROR: run_benchmark exited with code $RC for $COMBO (attempt $ATTEMPT/$MAX_RETRIES)."
      tail -n 30 "$LAUNCH_LOG" || true
      ATTEMPT=$((ATTEMPT + 1))
    fi
  done

  if [ "$COMBO_DONE" -ne 1 ]; then
    echo "ERROR: combo $COMBO did not complete after $MAX_RETRIES attempts."
    OVERALL_RC=1
  fi
done

echo ""
echo "Matrix finished at $(date -Iseconds) (rc=$OVERALL_RC)."
exit "$OVERALL_RC"

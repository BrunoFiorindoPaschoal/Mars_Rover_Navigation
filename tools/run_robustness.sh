#!/usr/bin/env bash
# Run the Mars Yard Nav2 robustness study across randomization levels 0-3 and planner x controller combos.
#
# Physics seed policy: seed = level, so every combo at a given level is evaluated on identical
# randomized physics. Sampled parameters are written next to the CSV as params_L<level>.json
# (and combined in params.json). Resumable on (level, combo, scenario, seed).
#
# Usage:
#   ./tools/run_robustness.sh [OPTIONS]
#
# Options:
#   --levels LIST         Comma-separated randomization levels (default: 0,1,2,3)
#   --combos LIST         Comma-separated combos (default: all 6)
#   --scenarios FILTER    Comma-separated scenario IDs or groups (default: all 20)
#   --seeds N             Number of start-pose seeds 0..N-1 (default: 2)
#   --output-csv PATH     CSV output path (default: results/robustness/robustness.csv)
#   --bags-dir PATH       MCAP bag directory (default: results/robustness/bags)
#   --params-dir PATH     Directory for generated randomized MJCFs (default: /tmp/mars_yard_robustness_worlds)
#   --log-file PATH       Combined log path (default: results/robustness/robustness.log)
#   --sim-timeout SEC     Per-run sim timeout in seconds (default: 120)
#   --max-retries N       Max relaunches per (level, combo) on crash or RTF < 0.9 (default: 3)
#   --no-bag              Disable MCAP bag recording

set -eo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"

LEVELS_LIST="0,1,2,3"
COMBOS_LIST="navfn+dwb,navfn+mppi,navfn+rpp,smac_hybrid+dwb,smac_hybrid+mppi,smac_hybrid+rpp"
SCENARIO_FILTER=""
SEEDS="2"
OUTPUT_CSV="$REPO/results/robustness/robustness.csv"
BAGS_DIR="$REPO/results/robustness/bags"
PARAMS_WORK_DIR="/tmp/mars_yard_robustness_worlds"
LOG_FILE="$REPO/results/robustness/robustness.log"
SIM_TIMEOUT="120"
MAX_RETRIES="3"
NO_BAG_FLAG=""

while [ $# -gt 0 ]; do
  case "$1" in
    --levels) LEVELS_LIST="$2"; shift 2 ;;
    --combos) COMBOS_LIST="$2"; shift 2 ;;
    --scenarios) SCENARIO_FILTER="$2"; shift 2 ;;
    --seeds) SEEDS="$2"; shift 2 ;;
    --output-csv) OUTPUT_CSV="$2"; shift 2 ;;
    --bags-dir) BAGS_DIR="$2"; shift 2 ;;
    --params-dir) PARAMS_WORK_DIR="$2"; shift 2 ;;
    --log-file) LOG_FILE="$2"; shift 2 ;;
    --sim-timeout) SIM_TIMEOUT="$2"; shift 2 ;;
    --max-retries) MAX_RETRIES="$2"; shift 2 ;;
    --no-bag) NO_BAG_FLAG="--no-bag"; shift ;;
    -h|--help)
      sed -n '2,23p' "$0"
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      exit 1
      ;;
  esac
done

RESULTS_DIR="$(dirname "$OUTPUT_CSV")"
mkdir -p "$RESULTS_DIR" "$BAGS_DIR" "$PARAMS_WORK_DIR" "$(dirname "$LOG_FILE")"

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

IFS=',' read -r -a LEVELS <<< "$LEVELS_LIST"
IFS=',' read -r -a COMBOS <<< "$COMBOS_LIST"

exec > >(tee -a "$LOG_FILE") 2>&1
echo "========================================================================"
echo "Mars Yard Robustness Study started: $(date -Iseconds)"
echo "  levels    : ${LEVELS[*]}"
echo "  combos    : ${COMBOS[*]}"
echo "  seeds     : $SEEDS"
echo "  scenarios : ${SCENARIO_FILTER:-ALL}"
echo "  csv       : $OUTPUT_CSV"
echo "  bags      : ${NO_BAG_FLAG:-$BAGS_DIR}"
echo "========================================================================"

# Helper: returns 0 if all (level, combo, scenario, seed) rows already exist in OUTPUT_CSV.
combo_already_complete() {
  local level="$1"
  local combo="$2"
  python3 - "$OUTPUT_CSV" "$level" "$combo" "$SCENARIO_FILTER" "$SEEDS" "$REPO" <<'PYEOF'
import csv, pathlib, sys, yaml
csv_path = pathlib.Path(sys.argv[1])
level = int(sys.argv[2])
combo = sys.argv[3]
sc_filter = sys.argv[4]
n_seeds = int(sys.argv[5])
repo = pathlib.Path(sys.argv[6])

sc_data = yaml.safe_load((repo / "src/mars_yard_benchmark/config/scenarios.yaml").read_text(encoding="utf-8"))
scenarios = sc_data["scenarios"]
if sc_filter.strip():
    tokens = {t.strip() for t in sc_filter.split(",") if t.strip()}
    scenarios = [sc for sc in scenarios if sc["id"] in tokens or sc["group"] in tokens]

needed = {(level, combo, sc["id"], s) for sc in scenarios for s in range(n_seeds)}
done = set()
if csv_path.exists() and csv_path.stat().st_size > 0:
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        for row in csv.DictReader(f):
            try:
                done.add((int(row["level"]), row["combo"], row["scenario"], int(row["seed"])))
            except Exception:
                continue
sys.exit(0 if needed.issubset(done) else 1)
PYEOF
}

OVERALL_RC=0
for LEVEL in "${LEVELS[@]}"; do
  RAND_SEED="$LEVEL"
  LEVEL_DIR="$PARAMS_WORK_DIR/L${LEVEL}_S${RAND_SEED}"
  mkdir -p "$LEVEL_DIR"
  python3 "$REPO/tools/randomize_world.py" \
    --world mars --level "$LEVEL" --seed "$RAND_SEED" --out "$LEVEL_DIR" > /dev/null
  cp "$LEVEL_DIR/params.json" "$RESULTS_DIR/params_L${LEVEL}.json"

  echo ""
  echo "------------------------------------------------------------------------"
  echo ">>> Randomization Level L${LEVEL} (physics seed=${RAND_SEED}, params=$RESULTS_DIR/params_L${LEVEL}.json)"
  echo "------------------------------------------------------------------------"

  for COMBO in "${COMBOS[@]}"; do
    PLANNER="${COMBO%%+*}"
    CONTROLLER="${COMBO##*+}"

    if combo_already_complete "$LEVEL" "$COMBO"; then
      echo "  [L${LEVEL} ${COMBO}] Already complete in $OUTPUT_CSV; skipping launch."
      continue
    fi

    echo ""
    echo ">>> Level=L${LEVEL} Combo: planner=$PLANNER controller=$CONTROLLER"

    ATTEMPT=1
    COMBO_DONE=0
    while [ "$ATTEMPT" -le "$MAX_RETRIES" ]; do
      cleanup_stack
      LAUNCH_LOG="/tmp/robustness_L${LEVEL}_${PLANNER}_${CONTROLLER}_try${ATTEMPT}.log"
      echo "  Launching sim.launch.py (L${LEVEL}, seed=${RAND_SEED}, attempt $ATTEMPT/$MAX_RETRIES) -> $LAUNCH_LOG"

      ros2 launch mars_yard_bringup sim.launch.py \
        world:=mars headless:=true rviz:=false use_ekf:=true metrics:=true hazards:=true \
        nav:=true planner:="$PLANNER" controller:="$CONTROLLER" \
        rand_level:="$LEVEL" rand_seed:="$RAND_SEED" rand_out_dir:="$LEVEL_DIR" \
        > "$LAUNCH_LOG" 2>&1 &
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
      python3 -m mars_yard_benchmark.run_benchmark \
        --level "$LEVEL" \
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
        echo "  WARNING: stack unhealthy / RTF < 0.9 for L${LEVEL} $COMBO (attempt $ATTEMPT/$MAX_RETRIES); relaunching and resuming..."
        ATTEMPT=$((ATTEMPT + 1))
      else
        echo "  ERROR: run_benchmark exited with code $RC for L${LEVEL} $COMBO (attempt $ATTEMPT/$MAX_RETRIES)."
        tail -n 30 "$LAUNCH_LOG" || true
        ATTEMPT=$((ATTEMPT + 1))
      fi
    done

    if [ "$COMBO_DONE" -ne 1 ]; then
      echo "ERROR: L${LEVEL} $COMBO did not complete after $MAX_RETRIES attempts."
      OVERALL_RC=1
    fi
  done
done

# Write combined params.json next to robustness.csv for convenience.
python3 - "$RESULTS_DIR" <<'PYEOF'
import json, pathlib, sys
res_dir = pathlib.Path(sys.argv[1])
combined = {}
for p in sorted(res_dir.glob("params_L*.json")):
    d = json.loads(p.read_text(encoding="utf-8"))
    combined[f"L{d['level']}"] = d
if combined:
    (res_dir / "params.json").write_text(json.dumps(combined, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PYEOF

echo ""
echo "Robustness study finished at $(date -Iseconds) (rc=$OVERALL_RC)."
exit "$OVERALL_RC"

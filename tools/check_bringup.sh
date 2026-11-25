#!/usr/bin/env bash
# Reusable headless verification for mars_yard_bringup.
# Launches sim.launch.py headless with a timeout, checks active controllers,
# drives forward (0.4 m/s) then turns (0.5 rad/s) on /cmd_vel, and prints
# ground-truth / EKF / wheel odometry poses, topic/control rates, /scan stats
# at start and end poses, and real-time factor.
#
# Usage:
#   ./tools/check_bringup.sh [flat|mars] [fwd_sim_sec] [turn_sim_sec] [launch_timeout_sec]
# Examples:
#   ./tools/check_bringup.sh flat          # full 10s fwd + 10s turn (~25s wall)
#   ./tools/check_bringup.sh mars          # full 10s fwd + 10s turn on mars

set -eo pipefail

WORLD="${1:-flat}"
FWD_SEC="${2:-10.0}"
TURN_SEC="${3:-10.0}"
if [ -n "${4:-}" ]; then
  LAUNCH_TIMEOUT="$4"
elif [ "$WORLD" = "mars" ]; then
  LAUNCH_TIMEOUT=180
else
  LAUNCH_TIMEOUT=120
fi

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
  pkill -f "rviz2" 2>/dev/null || true
  pkill -f "mars_yard_bringup" 2>/dev/null || true
  sleep 1
}
trap cleanup EXIT
cleanup

echo "=== check_bringup: world=$WORLD fwd=${FWD_SEC}s turn=${TURN_SEC}s timeout=${LAUNCH_TIMEOUT}s ==="
export MUJOCO_GL=egl
LAUNCH_LOG="/tmp/check_bringup_${WORLD}.log"
timeout "$LAUNCH_TIMEOUT" ros2 launch mars_yard_bringup sim.launch.py \
  world:="$WORLD" headless:=true rviz:=false use_ekf:=true > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

READY=0
for _ in $(seq 1 40); do
  OUT="$("$REPO/tools/list_controllers.sh" 2>/dev/null || true)"
  if echo "$OUT" | grep -q "diff_drive_controller.*active" && \
     echo "$OUT" | grep -q "imu_sensor_broadcaster.*active" && \
     echo "$OUT" | grep -q "joint_state_broadcaster.*active"; then
    READY=1
    break
  fi
  sleep 1
done

echo "--- Active controllers ---"
"$REPO/tools/list_controllers.sh"
if [ "$READY" -ne 1 ]; then
  echo "ERROR: Not all 3 controllers became active. Last 40 lines of $LAUNCH_LOG:"
  tail -n 40 "$LAUNCH_LOG" || true
  exit 1
fi

python3 - "$WORLD" "$FWD_SEC" "$TURN_SEC" "$LAUNCH_TIMEOUT" <<'PYEOF'
import math
import sys
import time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, LaserScan

world = sys.argv[1]
fwd_s = float(sys.argv[2])
turn_s = float(sys.argv[3])
wall_budget = max(30.0, float(sys.argv[4]) - 30.0)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_deg(rad):
    return math.degrees(math.atan2(math.sin(rad), math.cos(rad)))


def stamp_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def scan_stats(m):
    valid = [r for r in m.ranges if math.isfinite(r) and m.range_min <= r <= m.range_max]
    pos_inf = [r for r in m.ranges if math.isinf(r) and r > 0]
    neg_inf = [r for r in m.ranges if math.isinf(r) and r < 0]
    return valid, pos_inf, neg_inf


class RateTracker:
    def __init__(self):
        self.reset()

    def reset(self):
        self.count = 0
        self.first_s = None
        self.last_s = None
        self.first_w = None
        self.last_w = None

    def record(self, sim_t):
        now_w = time.monotonic()
        if self.count == 0:
            self.first_s = sim_t
            self.first_w = now_w
        self.last_s = sim_t
        self.last_w = now_w
        self.count += 1

    def rates(self):
        if self.count < 2:
            return 0.0, 0.0
        ds = self.last_s - self.first_s
        dw = self.last_w - self.first_w
        hz_s = (self.count - 1) / ds if ds > 0 else 0.0
        hz_w = (self.count - 1) / dw if dw > 0 else 0.0
        return hz_s, hz_w


class BringupProbe(Node):
    def __init__(self):
        super().__init__(
            "check_bringup_probe",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom = {}
        self.rates = {
            "imu": RateTracker(),
            "joint_states": RateTracker(),
            "wheel_odom": RateTracker(),
            "scan": RateTracker(),
        }
        self.recording = False
        self.create_subscription(Odometry, "/odometry/filtered", lambda m: self._set("ekf", m), 50)
        self.create_subscription(Odometry, "/diff_drive_controller/odom", self._on_wheel, 100)
        self.create_subscription(Odometry, "/simulator/floating_base_state", lambda m: self._set("gt", m), 50)
        self.create_subscription(Imu, "/imu_sensor_broadcaster/imu", self._on_imu, 200)
        self.create_subscription(JointState, "/joint_states", self._on_js, 100)
        self.scans = []
        self.last_scan = None
        self.create_subscription(LaserScan, "/scan", self._on_scan, 50)
        self.sim_time = None
        self.create_subscription(Clock, "/clock", self._on_clock, 50)

    def _set(self, key, msg):
        self.odom[key] = msg

    def _on_wheel(self, msg):
        self.odom["wheel"] = msg
        if self.recording:
            self.rates["wheel_odom"].record(stamp_sec(msg))

    def _on_imu(self, msg):
        if self.recording:
            self.rates["imu"].record(stamp_sec(msg))

    def _on_js(self, msg):
        if self.recording:
            self.rates["joint_states"].record(stamp_sec(msg))

    def _on_scan(self, msg):
        self.last_scan = msg
        if self.sim_time is not None:
            self.scans.append((self.sim_time, msg))
            if self.recording:
                self.rates["scan"].record(stamp_sec(msg))

    def _on_clock(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def drive_for(self, vx, wz, sim_dur, wall_timeout):
        t0_s = self.sim_time
        t0_w = time.monotonic()
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        last_pub = 0.0
        while (self.sim_time - t0_s) < sim_dur:
            now_w = time.monotonic()
            if now_w - last_pub >= 0.05:
                self.cmd_pub.publish(msg)
                last_pub = now_w
            rclpy.spin_once(self, timeout_sec=0.002)
            if now_w - t0_w > wall_timeout:
                print(f"WARNING: wall timeout after {self.sim_time - t0_s:.3f}s sim", flush=True)
                break
        return (self.sim_time - t0_s), (time.monotonic() - t0_w)

    def snapshot(self):
        out = {}
        for k in ("gt", "ekf", "wheel"):
            m = self.odom[k]
            p = m.pose.pose.position
            out[k] = (p.x, p.y, p.z, yaw_from_quat(m.pose.pose.orientation))
        return out


rclpy.init()
node = BringupProbe()
t0 = time.monotonic()
while (node.sim_time is None or len(node.odom) < 3 or node.last_scan is None) and (time.monotonic() - t0 < 40.0):
    rclpy.spin_once(node, timeout_sec=0.02)

if node.sim_time is None or len(node.odom) < 3 or node.last_scan is None:
    print(f"ERROR: missing topics: sim_time={node.sim_time}, odom={list(node.odom.keys())}, scans={len(node.scans)}")
    sys.exit(1)

settle_s = 1.0
node.drive_for(0.0, 0.0, settle_s, wall_timeout=min(40.0, wall_budget * 0.2))
start_scan = node.last_scan
node.scans.clear()
for tr in node.rates.values():
    tr.reset()
node.recording = True

sim_t0 = node.sim_time
wall_t0 = time.monotonic()
p_start = node.snapshot()

sim_d1, wall_d1 = node.drive_for(0.4, 0.0, fwd_s, wall_timeout=wall_budget * 0.45)
p_fwd = node.snapshot()

sim_d2, wall_d2 = node.drive_for(0.0, 0.5, turn_s, wall_timeout=wall_budget * 0.45)
node.drive_for(0.0, 0.0, 0.2, wall_timeout=min(20.0, wall_budget * 0.1))
p_end = node.snapshot()
node.recording = False
end_scan = node.last_scan

sim_tot = node.sim_time - sim_t0
wall_tot = time.monotonic() - wall_t0
print(f"\n=== Results ({world}) ===")
print(f"Sim: {sim_tot:.3f} s | Wall: {wall_tot:.3f} s | RTF: {sim_tot / wall_tot:.4f}x")
print(f"  fwd : sim={sim_d1:.3f}s wall={wall_d1:.3f}s RTF={sim_d1 / wall_d1:.4f}x")
print(f"  turn: sim={sim_d2:.3f}s wall={wall_d2:.3f}s RTF={sim_d2 / wall_d2:.4f}x")

print("\n[TOPIC RATES]")
for label, key in [
    ("/imu_sensor_broadcaster/imu (cfg 100 Hz)", "imu"),
    ("/joint_states               (cfg  50 Hz)", "joint_states"),
    ("/diff_drive_controller/odom (cfg  50 Hz)", "wheel_odom"),
    ("/scan                       (cfg  10 Hz)", "scan"),
]:
    tr = node.rates[key]
    hz_s, hz_w = tr.rates()
    print(f"  {label}: n={tr.count:4d}, sim_rate={hz_s:6.2f} Hz, wall_rate={hz_w:6.2f} Hz")

for label, snap in [("START", p_start), ("AFTER_FWD", p_fwd), ("END", p_end)]:
    print(f"\n[{label}]")
    for k in ("gt", "ekf", "wheel"):
        x, y, z, yaw = snap[k]
        print(f"  {k:5s}: x={x:+.4f} m, y={y:+.4f} m, z={z:+.4f} m, yaw={math.degrees(yaw):+.2f} deg")

print("\n[DELTAS & DRIFT VS GT]")
for seg, snap in [("AFTER_FWD", p_fwd), ("END", p_end)]:
    gx, gy, _, gyaw = snap["gt"]
    gx0, gy0, _, gyaw0 = p_start["gt"]
    g_dist = math.hypot(gx - gx0, gy - gy0)
    print(f"  {seg}:")
    for k in ("gt", "ekf", "wheel"):
        x0, y0, _, yaw0 = p_start[k]
        x, y, _, yaw = snap[k]
        dist = math.hypot(x - x0, y - y0)
        dyaw = wrap_deg(yaw - yaw0)
        pos_err = math.hypot(x - gx, y - gy)
        yaw_err = wrap_deg(yaw - gyaw)
        print(
            f"    {k:5s}: pos=({x:+.4f}, {y:+.4f}) m, yaw={math.degrees(yaw):+.2f} deg "
            f"(dist_from_start={dist:.4f} m, dyaw={dyaw:+.2f} deg | err_pos={pos_err:.4f} m, err_yaw={yaw_err:+.2f} deg)"
        )

for tag, m in [("START", start_scan), ("END", end_scan)]:
    if m is not None:
        valid, pos_inf, neg_inf = scan_stats(m)
        rng_str = f", valid range=[{min(valid):.3f}, {max(valid):.3f}] m" if valid else ""
        print(f"\n[/scan {tag}] frame={m.header.frame_id}, ranges={len(m.ranges)}, valid={len(valid)}, +inf={len(pos_inf)}, -inf={len(neg_inf)}{rng_str}")

node.destroy_node()
rclpy.shutdown()
PYEOF

kill "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true

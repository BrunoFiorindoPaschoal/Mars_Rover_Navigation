#!/usr/bin/env bash
# Headless verification for mars_yard_metrics.
# Launches sim.launch.py headless (rviz:=false, metrics:=true), publishes a straight
# /plan along +x from (0,0) to (10,0) every 0.1 m, drives 10 s forward (0.4 m/s)
# then 10 s turn (0.5 rad/s) on /cmd_vel, and prints all /metrics/* topics plus
# GT/EKF/wheel poses after each segment.
#
# Usage:
#   ./tools/check_metrics.sh [flat|mars] [fwd_sim_sec] [turn_sim_sec] [launch_timeout_sec]

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
  pkill -f "metrics_node" 2>/dev/null || true
  pkill -f "rviz2" 2>/dev/null || true
  pkill -f "mars_yard_bringup" 2>/dev/null || true
  sleep 1
}
trap cleanup EXIT
cleanup

echo "=== check_metrics: world=$WORLD fwd=${FWD_SEC}s turn=${TURN_SEC}s timeout=${LAUNCH_TIMEOUT}s ==="
export MUJOCO_GL=egl
export PYTHONUNBUFFERED=1
LAUNCH_LOG="/tmp/check_metrics_${WORLD}.log"
timeout "$LAUNCH_TIMEOUT" ros2 launch mars_yard_bringup sim.launch.py \
  world:="$WORLD" headless:=true rviz:=false use_ekf:=true metrics:=true > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

READY=0
for _ in $(seq 1 40); do
  OUT="$(ros2 control list_controllers 2>/dev/null || true)"
  if echo "$OUT" | grep -q "diff_drive_controller.*active" && \
     echo "$OUT" | grep -q "imu_sensor_broadcaster.*active" && \
     echo "$OUT" | grep -q "joint_state_broadcaster.*active"; then
    READY=1
    break
  fi
  sleep 1
done

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
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

world = sys.argv[1]
fwd_s = float(sys.argv[2])
turn_s = float(sys.argv[3])
wall_budget = max(30.0, float(sys.argv[4]) - 30.0)

METRIC_TOPICS = [
    "ekf_pos_drift",
    "ekf_yaw_drift",
    "wheel_pos_drift",
    "wheel_yaw_drift",
    "slip_ratio",
    "slip_instant",
    "lateral_slide",
    "true_distance",
    "energy",
    "cross_track_error",
]
WHEEL_JOINTS = ("wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint")


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_deg(rad):
    return math.degrees(math.atan2(math.sin(rad), math.cos(rad)))


class MetricsProbe(Node):
    def __init__(self):
        super().__init__(
            "check_metrics_probe",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.plan_pub = self.create_publisher(Path, "/plan", 10)

        self.odom = {}
        self.metrics = {}
        self.sim_time = None
        self.energy_monotonic = True
        self.prev_energy = None
        self.max_abs_effort = 0.0
        self.effort_nonzero_samples = 0
        self.effort_total_samples = 0
        self.recording_drive = False
        self.metric_ticks = 0

        self.create_subscription(Odometry, "/ground_truth/odom", lambda m: self._set_odom("gt", m), 50)
        self.create_subscription(Odometry, "/odometry/filtered", lambda m: self._set_odom("ekf", m), 50)
        self.create_subscription(Odometry, "/diff_drive_controller/odom", lambda m: self._set_odom("wheel", m), 50)
        self.create_subscription(JointState, "/joint_states", self._on_js, 100)
        self.create_subscription(Clock, "/clock", self._on_clock, 50)

        for name in METRIC_TOPICS:
            self.create_subscription(
                Float64,
                f"/metrics/{name}",
                lambda m, n=name: self._on_metric(n, m.data),
                20,
            )

    def _set_odom(self, key, msg):
        self.odom[key] = msg

    def _on_js(self, msg):
        if not self.recording_drive:
            return
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        for jn in WHEEL_JOINTS:
            idx = name_to_idx.get(jn)
            if idx is not None and idx < len(msg.effort) and math.isfinite(msg.effort[idx]):
                self.effort_total_samples += 1
                ae = abs(msg.effort[idx])
                if ae > self.max_abs_effort:
                    self.max_abs_effort = ae
                if ae > 1e-6:
                    self.effort_nonzero_samples += 1

    def _on_metric(self, name, value):
        self.metrics[name] = value
        if name == "true_distance":
            self.metric_ticks += 1
        if name == "energy" and math.isfinite(value):
            if self.prev_energy is not None and value + 1e-9 < self.prev_energy:
                self.energy_monotonic = False
            self.prev_energy = value

    def wait_metric_ticks_at_least(self, target, timeout=0.5):
        t0 = time.monotonic()
        while self.metric_ticks < target and (time.monotonic() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.002)

    def _on_clock(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def publish_straight_plan(self):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for i in range(101):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = 0.1 * i
            ps.pose.position.y = 0.0
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.plan_pub.publish(path)

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
        poses = {}
        for k in ("gt", "ekf", "wheel"):
            m = self.odom[k]
            p = m.pose.pose.position
            v = m.twist.twist.linear
            poses[k] = (p.x, p.y, p.z, yaw_from_quat(m.pose.pose.orientation), v.x, v.y)
        return poses, dict(self.metrics)


rclpy.init()
node = MetricsProbe()
t0 = time.monotonic()
while (
    node.sim_time is None
    or len(node.odom) < 3
    or len(node.metrics) < len(METRIC_TOPICS)
    or node.plan_pub.get_subscription_count() < 1
) and (time.monotonic() - t0 < 40.0):
    rclpy.spin_once(node, timeout_sec=0.02)

if node.sim_time is None or len(node.odom) < 3 or len(node.metrics) < len(METRIC_TOPICS):
    print(
        f"ERROR: missing topics: sim_time={node.sim_time}, odom={list(node.odom.keys())}, "
        f"metrics={sorted(node.metrics.keys())}, plan_subs={node.plan_pub.get_subscription_count()}"
    )
    sys.exit(1)

cte_before_plan = node.metrics["cross_track_error"]
node.publish_straight_plan()
# Wait for settle and align start with a 10 Hz metrics tick
node.drive_for(0.0, 0.0, 1.0, wall_timeout=min(40.0, wall_budget * 0.2))
node.wait_metric_ticks_at_least(node.metric_ticks + 1)

sim_t0 = node.sim_time
wall_t0 = time.monotonic()
start_ticks = node.metric_ticks
p_start, m_start = node.snapshot()

node.recording_drive = True
sim_d1, wall_d1 = node.drive_for(0.4, 0.0, fwd_s, wall_timeout=wall_budget * 0.45)
node.wait_metric_ticks_at_least(start_ticks + int(round(fwd_s * 10.0)))
p_fwd, m_fwd = node.snapshot()

sim_d2, wall_d2 = node.drive_for(0.0, 0.5, turn_s, wall_timeout=wall_budget * 0.45)
node.drive_for(0.0, 0.0, 0.2, wall_timeout=min(20.0, wall_budget * 0.1))
node.recording_drive = False
node.wait_metric_ticks_at_least(node.metric_ticks + 1)
p_end, m_end = node.snapshot()

sim_tot = node.sim_time - sim_t0
wall_tot = time.monotonic() - wall_t0

gt_msg = node.odom["gt"]
print(f"\n=== check_metrics results ({world}) ===")
print(f"Sim: {sim_tot:.3f} s | Wall: {wall_tot:.3f} s | RTF: {sim_tot / wall_tot:.4f}x")
print(f"  fwd : sim={sim_d1:.3f}s wall={wall_d1:.3f}s RTF={sim_d1 / wall_d1:.4f}x")
print(f"  turn: sim={sim_d2:.3f}s wall={wall_d2:.3f}s RTF={sim_d2 / wall_d2:.4f}x")
print(
    f"/ground_truth/odom frames: frame_id='{gt_msg.header.frame_id}', "
    f"child_frame_id='{gt_msg.child_frame_id}'"
)
print(f"cross_track_error before /plan: {cte_before_plan}")
print(
    f"Wheel effort check: nonzero={node.effort_nonzero_samples}/{node.effort_total_samples}, "
    f"max_abs={node.max_abs_effort:.3f} N*m | energy_monotonic={node.energy_monotonic}"
)

for label, poses, metrics in [
    ("START", p_start, m_start),
    ("AFTER_FWD", p_fwd, m_fwd),
    ("AFTER_TURN", p_end, m_end),
]:
    print(f"\n[{label} POSES]")
    for k in ("gt", "ekf", "wheel"):
        x, y, z, yaw, vx, vy = poses[k]
        print(
            f"  {k:5s}: x={x:+.4f} m, y={y:+.4f} m, z={z:+.4f} m, "
            f"yaw={math.degrees(yaw):+.2f} deg | v_body=({vx:+.3f}, {vy:+.3f}) m/s"
        )
    print(f"[{label} METRICS]")
    for name in METRIC_TOPICS:
        val = metrics[name]
        extra = f" ({math.degrees(val):+.2f} deg)" if "yaw" in name and math.isfinite(val) else ""
        print(f"  /metrics/{name:20s}: {val:+.6f}{extra}")

e_fwd = m_fwd["energy"] - m_start["energy"]
d_fwd = m_fwd["true_distance"] - m_start["true_distance"]
e_turn = m_end["energy"] - m_fwd["energy"]
d_turn = m_end["true_distance"] - m_fwd["true_distance"]
j_per_m_fwd = e_fwd / d_fwd if d_fwd > 1e-6 else float("nan")
j_per_s_turn = e_turn / sim_d2 if sim_d2 > 1e-6 else float("nan")

print("\n[ENERGY BREAKDOWN]")
print(f"  fwd  segment: dE={e_fwd:.3f} J, dDist={d_fwd:.4f} m -> {j_per_m_fwd:.2f} J/m")
print(f"  turn segment: dE={e_turn:.3f} J, dDist={d_turn:.4f} m -> {j_per_s_turn:.2f} J/s")
print(f"  total       : E={m_end['energy']:.3f} J, dist={m_end['true_distance']:.4f} m")

node.destroy_node()
rclpy.shutdown()
PYEOF

kill "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true

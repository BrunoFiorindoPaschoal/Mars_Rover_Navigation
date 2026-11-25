#!/usr/bin/env bash
# Headless verification for 3D lidar + step-height hazard detection (Phase 5a).
# Launches sim.launch.py headless (rviz:=false, hazards:=true), settles on the
# starting terrain, then drives toward the nearest forward boulder (on mars) or
# straight ahead (on flat). Reports:
#   - Physics RTF
#   - /points rate and point count
#   - /hazards rate and hazard/free cell counts
#   - Settled-at-spawn false hazard count
#   - Per-rock detection within 6 m (from obstacles.json) and false hazard count
#
# Usage:
#   ./tools/check_hazards.sh [flat|mars] [drive_sim_sec] [launch_timeout_sec]

set -eo pipefail

WORLD="${1:-mars}"
DRIVE_SEC="${2:-8.0}"
if [ -n "${3:-}" ]; then
  LAUNCH_TIMEOUT="$3"
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
  pkill -f "hazard_mapper" 2>/dev/null || true
  pkill -f "rviz2" 2>/dev/null || true
  pkill -f "mars_yard_bringup" 2>/dev/null || true
  sleep 1
}
trap cleanup EXIT
cleanup

echo "=== check_hazards: world=$WORLD drive=${DRIVE_SEC}s timeout=${LAUNCH_TIMEOUT}s ==="
export MUJOCO_GL=egl
export PYTHONUNBUFFERED=1
LAUNCH_LOG="/tmp/check_hazards_${WORLD}.log"
timeout "$LAUNCH_TIMEOUT" ros2 launch mars_yard_bringup sim.launch.py \
  world:="$WORLD" headless:=true rviz:=false use_ekf:=true metrics:=false hazards:=true > "$LAUNCH_LOG" 2>&1 &
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

if [ "$READY" -ne 1 ]; then
  echo "ERROR: Not all 3 controllers became active. Last 40 lines of $LAUNCH_LOG:"
  tail -n 40 "$LAUNCH_LOG" || true
  exit 1
fi

python3 - "$WORLD" "$DRIVE_SEC" "$LAUNCH_TIMEOUT" "$REPO" <<'PYEOF'
import json
import math
import pathlib
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2, PointField

world = sys.argv[1]
drive_s = float(sys.argv[2])
wall_budget = max(30.0, float(sys.argv[3]) - 30.0)
repo = pathlib.Path(sys.argv[4])


def stamp_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def rpy_from_quat(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def parse_xyz(msg: PointCloud2) -> np.ndarray:
    n = msg.width * msg.height
    if n == 0 or not msg.data:
        return np.empty((0, 3), dtype=np.float32)
    offsets = {f.name: f.offset for f in msg.fields if f.name in ("x", "y", "z") and f.datatype == PointField.FLOAT32}
    if len(offsets) != 3:
        return np.empty((0, 3), dtype=np.float32)
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": ["<f4", "<f4", "<f4"],
        "offsets": [offsets["x"], offsets["y"], offsets["z"]],
        "itemsize": msg.point_step,
    })
    arr = np.frombuffer(msg.data, dtype=dtype, count=n)
    return np.column_stack((arr["x"], arr["y"], arr["z"])).astype(np.float64)


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
        return (self.count - 1) / ds if ds > 0 else 0.0, (self.count - 1) / dw if dw > 0 else 0.0


class HazardProbe(Node):
    def __init__(self):
        super().__init__(
            "check_hazards_probe",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sim_time = None
        self.ekf = None
        self.gt = None
        self.imu = None
        self.last_points = None
        self.last_hazards = None
        self.last_grid = None
        self.recording = False
        self.points_rate = RateTracker()
        self.hazards_rate = RateTracker()
        self.point_counts = []

        self.create_subscription(Clock, "/clock", self._on_clock, 50)
        self.create_subscription(Odometry, "/odometry/filtered", lambda m: setattr(self, "ekf", m), 50)
        self.create_subscription(Odometry, "/simulator/floating_base_state", lambda m: setattr(self, "gt", m), 50)
        self.create_subscription(Imu, "/imu_sensor_broadcaster/imu", lambda m: setattr(self, "imu", m), 100)
        self.create_subscription(PointCloud2, "/points", self._on_points, 20)
        self.create_subscription(PointCloud2, "/hazards", self._on_hazards, 20)
        self.create_subscription(OccupancyGrid, "/hazard_grid", lambda m: setattr(self, "last_grid", m), 10)

    def _on_clock(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_points(self, msg):
        self.last_points = msg
        if self.recording:
            self.points_rate.record(stamp_sec(msg))
            self.point_counts.append(int(msg.width * msg.height))

    def _on_hazards(self, msg):
        self.last_hazards = msg
        if self.recording:
            self.hazards_rate.record(stamp_sec(msg))

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

    def drive_toward(self, target_xy, stop_dist, max_sim_dur, wall_timeout):
        t0_s = self.sim_time
        t0_w = time.monotonic()
        last_pub = 0.0
        while (self.sim_time - t0_s) < max_sim_dur:
            now_w = time.monotonic()
            p = self.ekf.pose.pose.position
            _, _, yaw = rpy_from_quat(self.ekf.pose.pose.orientation)
            dx = target_xy[0] - p.x
            dy = target_xy[1] - p.y
            dist = math.hypot(dx, dy)
            if dist <= stop_dist:
                break
            err = math.atan2(dy, dx) - yaw
            err = math.atan2(math.sin(err), math.cos(err))
            msg = Twist()
            msg.linear.x = 0.40 if abs(err) < 0.4 else 0.15
            msg.angular.z = max(-0.4, min(0.4, 0.8 * err))
            if now_w - last_pub >= 0.05:
                self.cmd_pub.publish(msg)
                last_pub = now_w
            rclpy.spin_once(self, timeout_sec=0.002)
            if now_w - t0_w > wall_timeout:
                print(f"WARNING: wall timeout after {self.sim_time - t0_s:.3f}s sim", flush=True)
                break
        return (self.sim_time - t0_s), (time.monotonic() - t0_w)


def evaluate_hazards(haz_msg: PointCloud2, rocks, ref_xy):
    pts = parse_xyz(haz_msg)
    if pts.shape[0] == 0:
        haz_xy = np.empty((0, 2), dtype=np.float64)
        free_n = 0
    else:
        haz_xy = pts[pts[:, 2] > 0.1, :2]
        free_n = int(np.sum(pts[:, 2] <= 0.1))

    # False hazards: hazard cells > 0.5 m from every rock/boulder surface (and also report center distance).
    false_surf = 0
    false_ctr = 0
    if haz_xy.shape[0] > 0:
        if not rocks:
            false_surf = int(haz_xy.shape[0])
            false_ctr = int(haz_xy.shape[0])
        else:
            r_xy = np.array([[r["x"], r["y"]] for r in rocks], dtype=np.float64)
            r_rad = np.array([max(r["size"][0], r["size"][1]) for r in rocks], dtype=np.float64)
            dists_ctr = np.hypot(haz_xy[:, None, 0] - r_xy[None, :, 0], haz_xy[:, None, 1] - r_xy[None, :, 1])
            dists_surf = np.maximum(0.0, dists_ctr - r_rad[None, :])
            false_surf = int(np.sum(dists_surf.min(axis=1) > 0.5))
            false_ctr = int(np.sum(dists_ctr.min(axis=1) > 0.5))

    # Per-rock check for all known rocks/boulders within 6 m of ref_xy.
    near_report = []
    for idx, r in enumerate(rocks):
        d_rover = math.hypot(r["x"] - ref_xy[0], r["y"] - ref_xy[1])
        if d_rover <= 6.0:
            rad = max(r["size"][0], r["size"][1])
            exp_h = round(0.7 * r["size"][2], 3)
            name = r.get("name", f"rock_{idx}")
            kind = r.get("kind", "rock")
            if haz_xy.shape[0] > 0:
                dc = np.hypot(haz_xy[:, 0] - r["x"], haz_xy[:, 1] - r["y"])
                ds = np.maximum(0.0, dc - rad)
                min_ds = float(ds.min())
                min_dc = float(dc.min())
                n_near = int(np.sum(ds <= 0.3))
                detected = min_ds <= 0.3
            else:
                min_ds, min_dc, n_near, detected = math.inf, math.inf, 0, False
            near_report.append({
                "name": name,
                "kind": kind,
                "x": r["x"],
                "y": r["y"],
                "rad": rad,
                "exp_h": exp_h,
                "d_rover": d_rover,
                "detected": detected,
                "min_surf_dist": min_ds,
                "min_ctr_dist": min_dc,
                "cells_within_03": n_near,
            })
    near_report.sort(key=lambda item: item["d_rover"])
    return int(haz_xy.shape[0]), free_n, false_surf, false_ctr, near_report


rocks = []
if world == "mars":
    obs = json.loads((repo / "src/mars_yard_worlds/terrain/obstacles.json").read_text())
    rocks = obs["rocks"]

rclpy.init()
node = HazardProbe()
t0 = time.monotonic()
while (
    node.sim_time is None
    or node.ekf is None
    or node.gt is None
    or node.imu is None
    or node.last_points is None
    or node.last_hazards is None
    or node.last_grid is None
) and (time.monotonic() - t0 < 40.0):
    rclpy.spin_once(node, timeout_sec=0.02)

if node.last_points is None or node.last_hazards is None or node.last_grid is None:
    print(
        f"ERROR: missing topics: sim_time={node.sim_time}, points={node.last_points is not None}, "
        f"hazards={node.last_hazards is not None}, grid={node.last_grid is not None}"
    )
    sys.exit(1)

# 1. Settle at spawn
node.drive_for(0.0, 0.0, 1.5, wall_timeout=min(30.0, wall_budget * 0.25))
r_settle, p_settle, _ = rpy_from_quat(node.imu.orientation)
ep0 = node.ekf.pose.pose.position
gp0 = node.gt.pose.pose.position
sh_n, sf_n, sfalse_surf, sfalse_ctr, snear = evaluate_hazards(node.last_hazards, rocks, (ep0.x, ep0.y))

# 2. Pick nearby forward boulder on mars (or drive straight on flat)
target_boulder = None
if world == "mars":
    boulders = [r for r in rocks if r.get("kind") == "boulder"]
    # Prefer closest boulder in forward hemisphere
    fwd_b = [b for b in boulders if b["x"] > 0.0]
    cand = fwd_b if fwd_b else boulders
    target_boulder = min(cand, key=lambda b: math.hypot(b["x"] - ep0.x, b["y"] - ep0.y))

node.points_rate.reset()
node.hazards_rate.reset()
node.point_counts.clear()
node.recording = True
sim_t0 = node.sim_time
wall_t0 = time.monotonic()

if target_boulder is not None:
    sim_d, wall_d = node.drive_toward(
        (target_boulder["x"], target_boulder["y"]),
        stop_dist=2.2,
        max_sim_dur=drive_s,
        wall_timeout=wall_budget * 0.65,
    )
else:
    sim_d, wall_d = node.drive_for(0.4, 0.0, min(drive_s, 6.0), wall_timeout=wall_budget * 0.65)

# Brief stop to observe settled pose near boulder
node.drive_for(0.0, 0.0, 1.0, wall_timeout=min(20.0, wall_budget * 0.15))
node.recording = False
sim_tot = node.sim_time - sim_t0
wall_tot = time.monotonic() - wall_t0

ep1 = node.ekf.pose.pose.position
gp1 = node.gt.pose.pose.position
eh_n, ef_n, efalse_surf, efalse_ctr, enear = evaluate_hazards(node.last_hazards, rocks, (ep1.x, ep1.y))
grid_haz = int(np.sum(np.asarray(node.last_grid.data, dtype=np.int8) == 100))
grid_free = int(np.sum(np.asarray(node.last_grid.data, dtype=np.int8) == 0))

pts_hz_s, pts_hz_w = node.points_rate.rates()
haz_hz_s, haz_hz_w = node.hazards_rate.rates()
pt_mean = float(np.mean(node.point_counts)) if node.point_counts else 0.0
pt_min = int(np.min(node.point_counts)) if node.point_counts else 0
pt_max = int(np.max(node.point_counts)) if node.point_counts else 0

print(f"\n=== check_hazards results ({world}) ===")
print(f"Sim: {sim_tot:.3f} s | Wall: {wall_tot:.3f} s | RTF: {sim_tot / wall_tot:.4f}x")
print(f"/points  : n={node.points_rate.count:3d}, sim_rate={pts_hz_s:5.2f} Hz, wall_rate={pts_hz_w:5.2f} Hz, pts/scan mean={pt_mean:.1f} [min={pt_min}, max={pt_max}]")
print(f"/hazards : n={node.hazards_rate.count:3d}, sim_rate={haz_hz_s:5.2f} Hz, wall_rate={haz_hz_w:5.2f} Hz")
print(f"Settled at spawn: IMU roll={math.degrees(r_settle):+.2f} deg, pitch={math.degrees(p_settle):+.2f} deg | "
      f"hazard_cells={sh_n}, free_cells={sf_n}, false_hazards(>0.5m surf)={sfalse_surf} (>0.5m ctr={sfalse_ctr})")
if target_boulder is not None:
    print(f"Drive target: {target_boulder['name']} at ({target_boulder['x']:+.2f}, {target_boulder['y']:+.2f}), "
          f"exp_h={target_boulder['exposed_height']:.3f} m")
print(f"Final pose: EKF=({ep1.x:+.3f}, {ep1.y:+.3f}) m | GT=({gp1.x:+.3f}, {gp1.y:+.3f}) m "
      f"(ekf_err={math.hypot(ep1.x - gp1.x, ep1.y - gp1.y):.3f} m)")
print(f"Final /hazards: hazard_cells={eh_n}, free_cells={ef_n} | /hazard_grid: hazard={grid_haz}, free={grid_free}")
print(f"Final FALSE hazards (>0.5 m from any rock surface): {efalse_surf} (>0.5 m from center: {efalse_ctr})")

print("\n[KNOWN ROCKS/BOULDERS WITHIN 6.0 m OF FINAL ROVER POSE]")
if not enear:
    print("  (none)")
else:
    for item in enear:
        flag = "DETECTED" if item["detected"] else "missed  "
        print(
            f"  {item['name']:10s} ({item['kind']:7s}) pos=({item['x']:+6.2f},{item['y']:+6.2f}) "
            f"d_rover={item['d_rover']:4.2f}m rad={item['rad']:.2f}m exp_h={item['exp_h']:.3f}m -> "
            f"{flag} (min_surf_dist={item['min_surf_dist']:.3f}m, min_ctr_dist={item['min_ctr_dist']:.3f}m, "
            f"cells<=0.3m={item['cells_within_03']})"
        )

node.destroy_node()
rclpy.shutdown()
PYEOF

kill "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true

#!/usr/bin/env python3
"""Headless verification runner for Stage A in-sim rover teleport and reset.

Verifies:
  1. Three teleports on drivable Mars terrain (moderate, 8-15 deg slope, gentle):
     - settled GT xy within 0.10 m of request and yaw within 3 deg
     - EKF pose within 0.05 m of GT right after /set_pose
     - every cumulative/drift /metrics/* topic back to ~0 after reset
     - rover not tipped (|roll|, |pitch| < 30 deg)
  2. Nav2 goal 8.0 m away from the 3rd teleport pose -> SUCCEEDED.
  3. Phase 5b reference run (spawn (0,0,0) -> goal (9.0,-0.6,0.0)) after teleports,
     compared against DEVLOG baseline (28.15 s, 0.222 m GT error).
  4. Wall time per reset vs wall time of a full relaunch.
"""

import argparse
import math
import sys
import threading
import time

from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64

from mars_yard_benchmark.sim_reset import SimResetter, quat_from_yaw, rpy_from_quat, wrap_angle


METRIC_TOPICS = (
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
)

# Cumulative / drift metrics that must be ~0 immediately after /metrics_node/reset.
ZERO_METRICS = (
    "ekf_pos_drift",
    "ekf_yaw_drift",
    "wheel_pos_drift",
    "wheel_yaw_drift",
    "slip_ratio",
    "lateral_slide",
    "true_distance",
    "energy",
)

TELEPORT_POSES = (
    ("pose1_moderate", 10.0, 5.0, -1.2),
    ("pose2_slope13deg", 4.0, -15.0, 2.1),
    ("pose3_gentle", -4.0, 11.0, 0.0),
)
POSE3_GOAL = (4.0, 11.0, 0.0)
REF_SPAWN = (0.0, 0.0, 0.0)
REF_GOAL = (9.0, -0.6, 0.0)
DEVLOG_REF_TIME_S = 28.15
DEVLOG_REF_GT_ERR_M = 0.222


class TeleportMonitor(Node):
    def __init__(self) -> None:
        super().__init__("check_teleport_monitor")
        self.lock = threading.Lock()
        self.sim_time = None
        self.gt = None
        self.ekf = None
        self.metrics = {}

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Clock, "/clock", self._on_clock, qos)
        self.create_subscription(Odometry, "/ground_truth/odom", self._on_gt, qos)
        self.create_subscription(Odometry, "/odometry/filtered", self._on_ekf, qos)
        for name in METRIC_TOPICS:
            self.create_subscription(
                Float64,
                f"/metrics/{name}",
                lambda msg, n=name: self._on_metric(n, msg.data),
                qos,
            )

    def _on_clock(self, msg: Clock) -> None:
        with self.lock:
            self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_gt(self, msg: Odometry) -> None:
        with self.lock:
            self.gt = msg

    def _on_ekf(self, msg: Odometry) -> None:
        with self.lock:
            self.ekf = msg

    def _on_metric(self, name: str, value: float) -> None:
        with self.lock:
            self.metrics[name] = value


def send_and_wait_goal(nav: BasicNavigator, mon: TeleportMonitor, gx: float, gy: float, gyaw: float, sim_timeout: float = 120.0):
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = nav.get_clock().now().to_msg()
    goal.pose.position.x = float(gx)
    goal.pose.position.y = float(gy)
    qx, qy, qz, qw = quat_from_yaw(gyaw)
    goal.pose.orientation.x = qx
    goal.pose.orientation.y = qy
    goal.pose.orientation.z = qz
    goal.pose.orientation.w = qw

    with mon.lock:
        sim_t0 = mon.sim_time
    wall_t0 = time.monotonic()

    nav.goToPose(goal)
    timed_out = False
    max_rec = 0
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb is not None:
            max_rec = max(max_rec, int(fb.number_of_recoveries))
        with mon.lock:
            sim_elapsed = mon.sim_time - sim_t0
        wall_elapsed = time.monotonic() - wall_t0
        if sim_elapsed >= sim_timeout or wall_elapsed >= max(160.0, sim_timeout * 1.6):
            timed_out = True
            nav.cancelTask()
            tc = time.monotonic()
            while not nav.isTaskComplete() and (time.monotonic() - tc) < 5.0:
                time.sleep(0.05)
            break

    with mon.lock:
        sim_elapsed = mon.sim_time - sim_t0
        gt = mon.gt
        ekf = mon.ekf
        metrics = dict(mon.metrics)
    wall_elapsed = time.monotonic() - wall_t0

    res = nav.getResult()
    if timed_out:
        res_str = "TIMEOUT"
    elif res == TaskResult.SUCCEEDED:
        res_str = "SUCCEEDED"
    elif res == TaskResult.FAILED:
        res_str = "FAILED"
    elif res == TaskResult.CANCELED:
        res_str = "CANCELED"
    else:
        res_str = f"UNKNOWN({res})"

    gp = gt.pose.pose.position
    ep = ekf.pose.pose.position
    gt_err = math.hypot(gp.x - gx, gp.y - gy)
    ekf_err = math.hypot(ep.x - gx, ep.y - gy)
    rtf = sim_elapsed / wall_elapsed if wall_elapsed > 0 else 0.0
    return {
        "result": res_str,
        "recoveries": max_rec,
        "sim_s": sim_elapsed,
        "wall_s": wall_elapsed,
        "rtf": rtf,
        "gt_xy": (gp.x, gp.y),
        "gt_err_m": gt_err,
        "ekf_err_m": ekf_err,
        "metrics": metrics,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify Stage A teleport and reset.")
    parser.add_argument("--launch-start-wall", type=float, required=True,
                        help="Epoch time (time.time()) when ros2 launch started, for full-relaunch timing.")
    args = parser.parse_args()

    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    nav = BasicNavigator()
    mon = TeleportMonitor()
    resetter = SimResetter()

    stop_spin = threading.Event()
    executor = SingleThreadedExecutor()
    executor.add_node(mon)

    def _spin():
        while not stop_spin.is_set() and rclpy.ok():
            executor.spin_once(timeout_sec=0.02)

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    all_ok = True
    try:
        # Wait for Nav2 active (with bounded warmup for FastDDS startup discovery).
        t0 = time.monotonic()
        cli = nav.create_client(GetState, "bt_navigator/get_state")
        while (time.monotonic() - t0) < 45.0:
            if not cli.wait_for_service(timeout_sec=1.0):
                continue
            fut = cli.call_async(GetState.Request())
            rclpy.spin_until_future_complete(nav, fut, timeout_sec=2.0)
            if fut.done() and fut.result() is not None and fut.result().current_state.label == "active":
                break
            time.sleep(0.3)
        nav.destroy_client(cli)
        nav.waitUntilNav2Active(navigator="bt_navigator", localizer="robot_localization")

        # Wait for topics and resetter ready.
        resetter.wait_ready(timeout_s=20.0)
        t_wait = time.monotonic()
        while (time.monotonic() - t_wait) < 15.0:
            with mon.lock:
                if mon.sim_time is not None and mon.gt is not None and mon.ekf is not None and len(mon.metrics) >= len(METRIC_TOPICS):
                    break
            time.sleep(0.05)

        relaunch_wall_s = time.time() - args.launch_start_wall
        print(f"Full launch-to-ready wall time: {relaunch_wall_s:.2f} s", flush=True)

        reset_times = []
        print("\n=== Test 1: Three drivable Mars teleports ===", flush=True)
        for label, tx, ty, ttheta in TELEPORT_POSES:
            pose = resetter.reset(tx, ty, ttheta, nav=nav)
            reset_times.append(pose.reset_wall_s)

            with mon.lock:
                metrics = dict(mon.metrics)

            xy_err = math.hypot(pose.x - tx, pose.y - ty)
            yaw_err_deg = math.degrees(abs(wrap_angle(pose.yaw - ttheta)))
            roll_deg = math.degrees(pose.roll)
            pitch_deg = math.degrees(pose.pitch)

            zero_bad = {}
            for k in ZERO_METRICS:
                val = metrics.get(k, float("nan"))
                if not math.isfinite(val) or abs(val) > 0.02:
                    zero_bad[k] = val

            ok_pose = (
                xy_err <= 0.10
                and yaw_err_deg <= 3.0
                and pose.ekf_pos_err_m <= 0.05
                and abs(roll_deg) < 30.0
                and abs(pitch_deg) < 30.0
                and len(zero_bad) == 0
            )
            if not ok_pose:
                all_ok = False

            status = "PASS" if ok_pose else "FAIL"
            print(
                f"  [{status}] {label}: req=({tx:+.2f},{ty:+.2f},{math.degrees(ttheta):+.1f}deg) "
                f"GT=({pose.x:+.3f},{pose.y:+.3f},z={pose.z:+.3f},{math.degrees(pose.yaw):+.1f}deg) "
                f"xy_err={xy_err:.4f}m yaw_err={yaw_err_deg:.2f}deg "
                f"rp=({roll_deg:+.1f},{pitch_deg:+.1f})deg "
                f"ekf_err={pose.ekf_pos_err_m:.4f}m "
                f"settle={pose.settle_sim_s:.2f}s_sim wall={pose.reset_wall_s:.2f}s "
                f"metrics_nonzero={zero_bad}",
                flush=True,
            )

        print("\n=== Test 2: Nav2 goal (8.0 m) from 3rd teleport pose ===", flush=True)
        nav_pose3 = send_and_wait_goal(nav, mon, *POSE3_GOAL, sim_timeout=120.0)
        ok_nav3 = nav_pose3["result"] == "SUCCEEDED"
        if not ok_nav3:
            all_ok = False
        print(
            f"  [{'PASS' if ok_nav3 else 'FAIL'}] pose3->({POSE3_GOAL[0]:.1f},{POSE3_GOAL[1]:.1f}): "
            f"result={nav_pose3['result']} rec={nav_pose3['recoveries']} "
            f"sim={nav_pose3['sim_s']:.2f}s wall={nav_pose3['wall_s']:.2f}s RTF={nav_pose3['rtf']:.4f}x "
            f"GT_err={nav_pose3['gt_err_m']:.3f}m EKF_err={nav_pose3['ekf_err_m']:.3f}m "
            f"energy={nav_pose3['metrics'].get('energy', float('nan')):.1f}J "
            f"dist={nav_pose3['metrics'].get('true_distance', float('nan')):.2f}m",
            flush=True,
        )

        print("\n=== Test 3: Phase 5b reference run after teleports (spawn -> (9.0, -0.6)) ===", flush=True)
        ref_pose = resetter.reset(*REF_SPAWN, nav=nav)
        reset_times.append(ref_pose.reset_wall_s)
        with mon.lock:
            ref_pre_metrics = {k: mon.metrics.get(k, float("nan")) for k in ZERO_METRICS}
        print(
            f"  Reset to spawn: GT=({ref_pose.x:+.3f},{ref_pose.y:+.3f},{math.degrees(ref_pose.yaw):+.2f}deg) "
            f"ekf_err={ref_pose.ekf_pos_err_m:.4f}m wall={ref_pose.reset_wall_s:.2f}s "
            f"energy_after_reset={ref_pre_metrics['energy']:.4f}J dist_after_reset={ref_pre_metrics['true_distance']:.4f}m",
            flush=True,
        )
        ref_run = send_and_wait_goal(nav, mon, *REF_GOAL, sim_timeout=120.0)
        ok_ref = ref_run["result"] == "SUCCEEDED" and ref_run["gt_err_m"] < 0.60
        if not ok_ref:
            all_ok = False
        print(
            f"  [{'PASS' if ok_ref else 'FAIL'}] reference run: "
            f"result={ref_run['result']} rec={ref_run['recoveries']} "
            f"sim={ref_run['sim_s']:.2f}s (DEVLOG={DEVLOG_REF_TIME_S:.2f}s, delta={ref_run['sim_s']-DEVLOG_REF_TIME_S:+.2f}s) "
            f"GT_err={ref_run['gt_err_m']:.3f}m (DEVLOG={DEVLOG_REF_GT_ERR_M:.3f}m, delta={ref_run['gt_err_m']-DEVLOG_REF_GT_ERR_M:+.3f}m) "
            f"RTF={ref_run['rtf']:.4f}x "
            f"energy={ref_run['metrics'].get('energy', float('nan')):.1f}J "
            f"ekf_drift={ref_run['metrics'].get('ekf_pos_drift', float('nan')):.3f}m "
            f"lat_slide={ref_run['metrics'].get('lateral_slide', float('nan')):.3f}m",
            flush=True,
        )

        mean_reset_wall = sum(reset_times) / len(reset_times)
        speedup = relaunch_wall_s / mean_reset_wall if mean_reset_wall > 0 else 0.0
        print("\n=== Timing summary ===", flush=True)
        print(f"  Full relaunch wall time : {relaunch_wall_s:.2f} s", flush=True)
        print(
            f"  Reset wall times        : {[round(t, 2) for t in reset_times]} s "
            f"(mean={mean_reset_wall:.2f} s, speedup={speedup:.1f}x)",
            flush=True,
        )
        print(f"\nOVERALL STAGE A CHECK: {'PASS' if all_ok else 'FAIL'}", flush=True)
        return 0 if all_ok else 2
    finally:
        stop_spin.set()
        spin_thread.join(timeout=2.0)
        executor.remove_node(mon)
        mon.destroy_node()
        resetter.destroy()
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

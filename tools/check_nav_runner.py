#!/usr/bin/env python3
"""Headless Nav2 verification runner using nav2_simple_commander.

Sends a single NavigateToPose goal, monitors GT pose, EKF pose, /plan, /metrics/*,
and /clock on a dedicated spinner thread (zero queue lag), and computes obstacle
clearance against src/mars_yard_worlds/terrain/obstacles.json.
"""

import argparse
import json
import math
import pathlib
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64


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


def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quat_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def path_length(path_msg: Path) -> float:
    poses = path_msg.poses
    if len(poses) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(poses)):
        p0 = poses[i - 1].pose.position
        p1 = poses[i].pose.position
        total += math.hypot(p1.x - p0.x, p1.y - p0.y)
    return total


class NavMonitor(Node):
    def __init__(self) -> None:
        super().__init__("check_nav_monitor")
        self.lock = threading.Lock()
        self.sim_time = None
        self.gt = None
        self.ekf = None
        self.map_received = False
        self.global_cm_received = False
        self.local_cm_received = False

        self.recording = False
        self.gt_trace = []
        self.first_plan_len = None
        self.last_plan_len = None
        self.plan_count = 0
        self.metrics = {}

        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        latest_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(Clock, "/clock", self._on_clock, latest_qos)
        self.create_subscription(Odometry, "/ground_truth/odom", self._on_gt, latest_qos)
        self.create_subscription(Odometry, "/odometry/filtered", self._on_ekf, latest_qos)
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self._on_global_cm, map_qos
        )
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self._on_local_cm, latest_qos
        )

        for name in METRIC_TOPICS:
            self.create_subscription(
                Float64,
                f"/metrics/{name}",
                lambda msg, n=name: self._on_metric(n, msg.data),
                latest_qos,
            )

    def _on_clock(self, msg: Clock) -> None:
        with self.lock:
            self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_gt(self, msg: Odometry) -> None:
        with self.lock:
            self.gt = msg
            if self.recording:
                p = msg.pose.pose.position
                yaw = yaw_from_quat(msg.pose.pose.orientation)
                if not self.gt_trace:
                    self.gt_trace.append((p.x, p.y, yaw))
                else:
                    lx, ly, _ = self.gt_trace[-1]
                    if math.hypot(p.x - lx, p.y - ly) >= 0.01:
                        self.gt_trace.append((p.x, p.y, yaw))

    def _on_ekf(self, msg: Odometry) -> None:
        with self.lock:
            self.ekf = msg

    def _on_plan(self, msg: Path) -> None:
        plen = path_length(msg)
        with self.lock:
            if not self.recording or plen <= 0.0:
                return
            self.plan_count += 1
            if self.first_plan_len is None:
                self.first_plan_len = plen
            self.last_plan_len = plen

    def _on_metric(self, name: str, value: float) -> None:
        with self.lock:
            self.metrics[name] = value

    def _on_map(self, _msg: OccupancyGrid) -> None:
        with self.lock:
            self.map_received = True

    def _on_global_cm(self, _msg: OccupancyGrid) -> None:
        with self.lock:
            self.global_cm_received = True

    def _on_local_cm(self, _msg: OccupancyGrid) -> None:
        with self.lock:
            self.local_cm_received = True


def compute_obstacle_clearances(gt_trace, rocks):
    """Return per-obstacle min distance from GT footprint centre to obstacle surface."""
    if not gt_trace or not rocks:
        return []
    pts = np.array([(p[0], p[1]) for p in gt_trace], dtype=np.float64)
    report = []
    for idx, r in enumerate(rocks):
        rx, ry = float(r["x"]), float(r["y"])
        rad = float(max(r["size"][0], r["size"][1]))
        exp_h = round(0.7 * float(r["size"][2]), 3)
        d_ctr = np.hypot(pts[:, 0] - rx, pts[:, 1] - ry)
        min_idx = int(np.argmin(d_ctr))
        min_dc = float(d_ctr[min_idx])
        min_ds = max(0.0, min_dc - rad)
        if min_ds <= 2.0:
            report.append(
                {
                    "name": r.get("name", f"rock_{idx}"),
                    "kind": r.get("kind", "rock"),
                    "x": rx,
                    "y": ry,
                    "rad": rad,
                    "exp_h": exp_h,
                    "min_surf_dist": min_ds,
                    "min_ctr_dist": min_dc,
                    "closest_gt": (float(pts[min_idx, 0]), float(pts[min_idx, 1])),
                }
            )
    report.sort(key=lambda item: item["min_surf_dist"])
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a single Nav2 goal check.")
    parser.add_argument("--world", required=True, choices=("flat", "mars"))
    parser.add_argument("--planner", required=True)
    parser.add_argument("--controller", required=True)
    parser.add_argument("--goal-x", type=float, required=True)
    parser.add_argument("--goal-y", type=float, required=True)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--sim-timeout", type=float, default=180.0)
    parser.add_argument("--wall-timeout", type=float, default=300.0)
    parser.add_argument("--repo", required=True)
    args = parser.parse_args()

    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    nav = BasicNavigator()
    mon = NavMonitor()

    stop_spin = threading.Event()
    executor = SingleThreadedExecutor()
    executor.add_node(mon)

    def _spin_mon():
        while not stop_spin.is_set() and rclpy.ok():
            executor.spin_once(timeout_sec=0.02)

    spin_thread = threading.Thread(target=_spin_mon, daemon=True)
    spin_thread.start()

    try:
        # 1. Wait for Nav2 lifecycle nodes to become ACTIVE (no AMCL -> robot_localization).
        # Warm up /bt_navigator/get_state with bounded retries so a dropped discovery packet
        # cannot stall BasicNavigator.waitUntilNav2Active() indefinitely.
        t_wait0 = time.monotonic()
        print("Waiting for Nav2 lifecycle nodes to become active...", flush=True)
        state_cli = nav.create_client(GetState, "bt_navigator/get_state")
        while (time.monotonic() - t_wait0) < 40.0:
            if not state_cli.wait_for_service(timeout_sec=1.0):
                continue
            fut = state_cli.call_async(GetState.Request())
            rclpy.spin_until_future_complete(nav, fut, timeout_sec=2.0)
            if fut.done() and fut.result() is not None and fut.result().current_state.label == "active":
                break
            time.sleep(0.5)
        nav.destroy_client(state_cli)
        nav.waitUntilNav2Active(navigator="bt_navigator", localizer="robot_localization")
        print(f"Nav2 active after {time.monotonic() - t_wait0:.2f} s wall.", flush=True)

        # 2. Wait for sim clock, GT, EKF, map, costmaps, and metrics.
        t_ready0 = time.monotonic()
        while (time.monotonic() - t_ready0) < 30.0:
            with mon.lock:
                ready = (
                    mon.sim_time is not None
                    and mon.gt is not None
                    and mon.ekf is not None
                    and mon.map_received
                    and mon.global_cm_received
                    and mon.local_cm_received
                    and len(mon.metrics) >= len(METRIC_TOPICS)
                )
            if ready:
                break
            time.sleep(0.05)

        with mon.lock:
            sim_t = mon.sim_time
            has_gt = mon.gt is not None
            has_ekf = mon.ekf is not None
            has_map = mon.map_received
            has_gcm = mon.global_cm_received
            has_lcm = mon.local_cm_received
            n_met = len(mon.metrics)

        if sim_t is None or not has_gt or not has_ekf:
            print(
                f"ERROR: missing topics: sim_time={sim_t}, gt={has_gt}, ekf={has_ekf}, "
                f"map={has_map}, global_cm={has_gcm}, local_cm={has_lcm}, "
                f"metrics={n_met}/{len(METRIC_TOPICS)}",
                flush=True,
            )
            return 1

        # Brief settle (~1.5 s sim) so rover rests on terrain and initial /hazards populate costmaps.
        t_settle0 = sim_t
        while (time.monotonic() - t_ready0) < 40.0:
            with mon.lock:
                if mon.sim_time - t_settle0 >= 1.5:
                    break
            time.sleep(0.02)

        # 3. Send goal
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = nav.get_clock().now().to_msg()
        goal.pose.position.x = args.goal_x
        goal.pose.position.y = args.goal_y
        qx, qy, qz, qw = quat_from_yaw(args.goal_yaw)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        with mon.lock:
            mon.recording = True
            p0 = mon.gt.pose.pose.position
            mon.gt_trace.append((p0.x, p0.y, yaw_from_quat(mon.gt.pose.pose.orientation)))
            sim_t0 = mon.sim_time
        wall_t0 = time.monotonic()

        print(
            f"Sending goal ({args.goal_x:.2f}, {args.goal_y:.2f}, yaw={math.degrees(args.goal_yaw):.1f} deg) "
            f"from GT=({p0.x:+.3f}, {p0.y:+.3f}) at sim_t={sim_t0:.2f}s...",
            flush=True,
        )
        nav.goToPose(goal)

        timed_out = False
        max_recoveries = 0
        last_log_wall = wall_t0

        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            now_w = time.monotonic()
            with mon.lock:
                sim_elapsed = mon.sim_time - sim_t0
                gp = mon.gt.pose.pose.position
                ep = mon.ekf.pose.pose.position
            wall_elapsed = now_w - wall_t0
            if fb is not None:
                max_recoveries = max(max_recoveries, int(fb.number_of_recoveries))
            if now_w - last_log_wall >= 5.0:
                last_log_wall = now_w
                dist_gt = math.hypot(args.goal_x - gp.x, args.goal_y - gp.y)
                rtf_now = sim_elapsed / wall_elapsed if wall_elapsed > 0 else 0.0
                print(
                    f"  [sim={sim_elapsed:5.1f}s wall={wall_elapsed:5.1f}s RTF={rtf_now:.3f}x] "
                    f"GT=({gp.x:+.2f},{gp.y:+.2f}) EKF=({ep.x:+.2f},{ep.y:+.2f}) "
                    f"dist_gt={dist_gt:.2f}m recoveries={max_recoveries}",
                    flush=True,
                )
            if sim_elapsed >= args.sim_timeout or wall_elapsed >= args.wall_timeout:
                timed_out = True
                print(
                    f"TIMEOUT reached (sim={sim_elapsed:.1f}s/{args.sim_timeout:.1f}s, "
                    f"wall={wall_elapsed:.1f}s/{args.wall_timeout:.1f}s); canceling goal...",
                    flush=True,
                )
                nav.cancelTask()
                t_c = time.monotonic()
                while not nav.isTaskComplete() and (time.monotonic() - t_c) < 5.0:
                    time.sleep(0.05)
                break

        # Capture exact completion time before brief settle for final metrics tick
        with mon.lock:
            sim_elapsed = mon.sim_time - sim_t0
        wall_elapsed = time.monotonic() - wall_t0

        time.sleep(0.25)

        with mon.lock:
            mon.recording = False
            gt_msg = mon.gt
            ekf_msg = mon.ekf
            p_end = gt_msg.pose.pose.position
            mon.gt_trace.append((p_end.x, p_end.y, yaw_from_quat(gt_msg.pose.pose.orientation)))
            gt_trace = list(mon.gt_trace)
            first_plan_len = mon.first_plan_len
            last_plan_len = mon.last_plan_len
            plan_count = mon.plan_count
            metrics = dict(mon.metrics)

        task_res = nav.getResult()
        if timed_out:
            result_str = "TIMEOUT"
        elif task_res == TaskResult.SUCCEEDED:
            result_str = "SUCCEEDED"
        elif task_res == TaskResult.FAILED:
            result_str = "FAILED"
        elif task_res == TaskResult.CANCELED:
            result_str = "CANCELED"
        else:
            result_str = f"UNKNOWN({task_res})"

        gt_path_len = 0.0
        for i in range(1, len(gt_trace)):
            gt_path_len += math.hypot(
                gt_trace[i][0] - gt_trace[i - 1][0],
                gt_trace[i][1] - gt_trace[i - 1][1],
            )

        gt_yaw = yaw_from_quat(gt_msg.pose.pose.orientation)
        ekf_p = ekf_msg.pose.pose.position
        ekf_yaw = yaw_from_quat(ekf_msg.pose.pose.orientation)
        gt_pos_err = math.hypot(p_end.x - args.goal_x, p_end.y - args.goal_y)
        ekf_pos_err = math.hypot(ekf_p.x - args.goal_x, ekf_p.y - args.goal_y)
        rtf = sim_elapsed / wall_elapsed if wall_elapsed > 0 else 0.0

        rocks = []
        if args.world == "mars":
            obs_path = pathlib.Path(args.repo) / "src/mars_yard_worlds/terrain/obstacles.json"
            rocks = json.loads(obs_path.read_text(encoding="utf-8"))["rocks"]
        near_obs = compute_obstacle_clearances(gt_trace, rocks)
        boulder_clearances = [o["min_surf_dist"] for o in near_obs if o["kind"] == "boulder"]
        min_boulder_surf = min(boulder_clearances) if boulder_clearances else math.inf
        min_any_surf = near_obs[0]["min_surf_dist"] if near_obs else math.inf
        collision = any(o["min_surf_dist"] < 0.42 for o in near_obs if o["exp_h"] >= 0.12)

        print(
            f"\n=== check_nav results (world={args.world}, planner={args.planner}, controller={args.controller}) ===",
            flush=True,
        )
        print(f"Result             : {result_str} (recoveries={max_recoveries})", flush=True)
        print(
            f"Goal               : ({args.goal_x:+.2f}, {args.goal_y:+.2f}, yaw={math.degrees(args.goal_yaw):+.1f} deg)",
            flush=True,
        )
        print(
            f"Time               : sim={sim_elapsed:.2f} s | wall={wall_elapsed:.2f} s | RTF={rtf:.4f}x",
            flush=True,
        )
        print(
            f"Final GT pose      : ({p_end.x:+.3f}, {p_end.y:+.3f}) m, yaw={math.degrees(gt_yaw):+.2f} deg | "
            f"GT pos err={gt_pos_err:.3f} m",
            flush=True,
        )
        print(
            f"Final EKF pose     : ({ekf_p.x:+.3f}, {ekf_p.y:+.3f}) m, yaw={math.degrees(ekf_yaw):+.2f} deg | "
            f"EKF pos err={ekf_pos_err:.3f} m",
            flush=True,
        )
        first_pl = f"{first_plan_len:.3f} m" if first_plan_len is not None else "N/A"
        last_pl = f"{last_plan_len:.3f} m" if last_plan_len is not None else "N/A"
        print(
            f"Path length        : GT={gt_path_len:.3f} m | initial /plan={first_pl} | "
            f"final /plan={last_pl} (plans={plan_count})",
            flush=True,
        )
        print(
            f"Obstacle clearance : min_boulder_surf={min_boulder_surf:.3f} m | "
            f"min_any_surf={min_any_surf:.3f} m | collision(<0.42m, exp_h>=0.12m)={collision}",
            flush=True,
        )

        print("\n[OBSTACLES WITHIN 2.0 m OF GT PATH]", flush=True)
        if not near_obs:
            print("  (none)", flush=True)
        else:
            for o in near_obs:
                tag = "COLLISION" if o["min_surf_dist"] < 0.42 else "clear    "
                print(
                    f"  {o['name']:10s} ({o['kind']:7s}) pos=({o['x']:+6.2f},{o['y']:+6.2f}) "
                    f"rad={o['rad']:.2f}m exp_h={o['exp_h']:.3f}m -> "
                    f"min_surf_dist={o['min_surf_dist']:.3f}m (ctr={o['min_ctr_dist']:.3f}m) [{tag}]",
                    flush=True,
                )

        print("\n[FINAL /metrics/*]", flush=True)
        for name in METRIC_TOPICS:
            val = metrics.get(name, float("nan"))
            extra = (
                f" ({math.degrees(val):+.2f} deg)"
                if "yaw" in name and math.isfinite(val)
                else ""
            )
            print(f"  /metrics/{name:20s}: {val:+.6f}{extra}", flush=True)

        return 0 if (result_str == "SUCCEEDED" and not collision and rtf >= 0.9) else 2
    finally:
        stop_spin.set()
        spin_thread.join(timeout=2.0)
        executor.remove_node(mon)
        mon.destroy_node()
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

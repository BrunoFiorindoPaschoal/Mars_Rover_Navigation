#!/usr/bin/env python3
"""Run Mars Yard Nav2 benchmark scenarios for a single launched planner x controller combo.

For each scenario x seed (skipping rows already recorded in the output CSV):
  1. Resets the rover in-sim via SimResetter (teleport, settle, EKF set_pose, clear
     hazards and costmaps, reset metrics) with reproducible start perturbation for seed > 0.
  2. Starts `ros2 bag record -s mcap` on telemetry topics (excluding /points and /scan), plus any
     topics in the BAG_EXTRA_TOPICS environment variable (space-separated).
  3. Sends the NavigateToPose goal via nav2_simple_commander and monitors for completion
     or failure (timeout, stuck, collision, tipover, aborted).
  4. Stops the bag cleanly and immediately appends one CSV row (crash-safe).

Exits with code 0 on normal completion, or code 3 if the simulation stack appears unhealthy
(RTF < 0.9 or /clock stalled) so the outer matrix runner can relaunch and resume.
"""

import argparse
from collections import deque
import csv
import json
import math
import os
import pathlib
import signal
import subprocess
import sys
import threading
import time
from typing import Dict, List, Optional, Set, Tuple

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64
import yaml

from mars_yard_benchmark.geometry import (
    build_hazard_rock_contours,
    point_to_polyline_dist,
    signed_footprint_clearance,
)
from mars_yard_benchmark.sim_reset import (
    SimResetter,
    quat_from_yaw,
    rpy_from_quat,
    stamp_to_sec,
    wrap_angle,
)


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

BAG_TOPICS = (
    "/ground_truth/odom",
    "/odometry/filtered",
    "/plan",
    "/cmd_vel",
    "/metrics/ekf_pos_drift",
    "/metrics/ekf_yaw_drift",
    "/metrics/wheel_pos_drift",
    "/metrics/wheel_yaw_drift",
    "/metrics/slip_ratio",
    "/metrics/slip_instant",
    "/metrics/lateral_slide",
    "/metrics/true_distance",
    "/metrics/energy",
    "/metrics/cross_track_error",
    "/joint_states",
    "/tf",
)

CSV_COLUMNS = [
    "combo",
    "scenario",
    "group",
    "seed",
    "result",
    "failure_type",
    "time_s",
    "path_len_gt",
    "straight_dist",
    "path_ratio",
    "gt_goal_err",
    "tracking_err",
    "slip_ratio",
    "lateral_slide",
    "ekf_drift",
    "energy_J",
    "min_obstacle_clearance",
    "recoveries",
    "rtf",
]


class BenchmarkMonitor(Node):
    """Dedicated background-spun monitor for zero-lag pose, plan, clock, and metrics tracking."""

    def __init__(self, rock_pts: np.ndarray, rock_meta: list) -> None:
        super().__init__("benchmark_monitor")
        self.lock = threading.Lock()
        self.rock_pts = rock_pts
        self.rock_meta = rock_meta

        self.sim_time: Optional[float] = None
        self.last_clock_wall: float = time.monotonic()
        self.gt: Optional[Odometry] = None
        self.ekf: Optional[Odometry] = None
        self.metrics: Dict[str, float] = {}

        self.recording: bool = False
        self.plan_pts: np.ndarray = np.empty((0, 2), dtype=np.float64)
        self.gt_trace: List[Tuple[float, float, float, float, float, float]] = []
        self.recent_window: deque = deque()
        self.tracking_errs: List[float] = []
        self.min_clearance: float = math.inf
        self.max_abs_tilt_deg: float = 0.0
        self.tipped: bool = False
        self.hard_collision: bool = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Clock, "/clock", self._on_clock, qos)
        self.create_subscription(Odometry, "/ground_truth/odom", self._on_gt, qos)
        self.create_subscription(Odometry, "/odometry/filtered", self._on_ekf, qos)
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        for name in METRIC_TOPICS:
            self.create_subscription(
                Float64,
                f"/metrics/{name}",
                lambda msg, n=name: self._on_metric(n, msg.data),
                qos,
            )

    def start_run(self) -> float:
        with self.lock:
            self.recording = True
            self.plan_pts = np.empty((0, 2), dtype=np.float64)
            self.gt_trace = []
            self.recent_window = deque()
            self.tracking_errs = []
            self.min_clearance = math.inf
            self.max_abs_tilt_deg = 0.0
            self.tipped = False
            self.hard_collision = False
            if self.gt is not None and self.sim_time is not None:
                p = self.gt.pose.pose.position
                r, pit, yaw = rpy_from_quat(self.gt.pose.pose.orientation)
                self.gt_trace.append((self.sim_time, p.x, p.y, r, pit, yaw))
                self.recent_window.append((self.sim_time, p.x, p.y))
                clr = signed_footprint_clearance(p.x, p.y, yaw, self.rock_pts, self.rock_meta)
                self.min_clearance = min(self.min_clearance, clr)
            return float(self.sim_time or 0.0)

    def stop_run(self) -> Dict:
        with self.lock:
            self.recording = False
            return {
                "sim_time": float(self.sim_time or 0.0),
                "gt": self.gt,
                "ekf": self.ekf,
                "metrics": dict(self.metrics),
                "gt_trace": list(self.gt_trace),
                "tracking_errs": list(self.tracking_errs),
                "min_clearance": float(self.min_clearance),
                "max_abs_tilt_deg": float(self.max_abs_tilt_deg),
                "tipped": bool(self.tipped),
                "hard_collision": bool(self.hard_collision),
            }

    def check_live_status(self, sim_t0: float, stuck_window_s: float = 20.0, stuck_min_move_m: float = 0.30):
        with self.lock:
            sim_t = float(self.sim_time or 0.0)
            sim_elapsed = sim_t - sim_t0
            clock_age_wall = time.monotonic() - self.last_clock_wall
            tipped = self.tipped
            hard_collision = self.hard_collision
            min_clr = self.min_clearance

            stuck = False
            if sim_elapsed >= stuck_window_s and len(self.recent_window) >= 2:
                # Prune samples older than stuck_window_s
                while len(self.recent_window) > 2 and (sim_t - self.recent_window[1][0]) >= stuck_window_s:
                    self.recent_window.popleft()
                t_old, x0, y0 = self.recent_window[0]
                if (sim_t - t_old) >= (stuck_window_s - 0.5):
                    max_disp = 0.0
                    for _, xi, yi in self.recent_window:
                        d = math.hypot(xi - x0, yi - y0)
                        if d > max_disp:
                            max_disp = d
                    if max_disp < stuck_min_move_m:
                        stuck = True
            return sim_elapsed, clock_age_wall, tipped, hard_collision, stuck, min_clr

    def _on_clock(self, msg: Clock) -> None:
        with self.lock:
            self.sim_time = stamp_to_sec(msg.clock)
            self.last_clock_wall = time.monotonic()

    def _on_ekf(self, msg: Odometry) -> None:
        with self.lock:
            self.ekf = msg

    def _on_plan(self, msg: Path) -> None:
        if not msg.poses:
            return
        arr = np.array([[ps.pose.position.x, ps.pose.position.y] for ps in msg.poses], dtype=np.float64)
        with self.lock:
            if self.recording:
                self.plan_pts = arr

    def _on_metric(self, name: str, value: float) -> None:
        with self.lock:
            self.metrics[name] = value

    def _on_gt(self, msg: Odometry) -> None:
        with self.lock:
            self.gt = msg
            if not self.recording or self.sim_time is None:
                return
            p = msg.pose.pose.position
            roll, pitch, yaw = rpy_from_quat(msg.pose.pose.orientation)
            t = self.sim_time

            # Downsample trace to >= 0.01 m or >= 0.1 s sim so memory stays tiny while preserving accuracy.
            record_sample = False
            if not self.gt_trace:
                record_sample = True
            else:
                lt, lx, ly, _, _, _ = self.gt_trace[-1]
                if (t - lt) >= 0.10 or math.hypot(p.x - lx, p.y - ly) >= 0.02:
                    record_sample = True

            if record_sample:
                self.gt_trace.append((t, p.x, p.y, roll, pitch, yaw))
                self.recent_window.append((t, p.x, p.y))
                while len(self.recent_window) > 2 and (t - self.recent_window[1][0]) > 22.0:
                    self.recent_window.popleft()

                tilt_deg = max(abs(math.degrees(roll)), abs(math.degrees(pitch)))
                if tilt_deg > self.max_abs_tilt_deg:
                    self.max_abs_tilt_deg = tilt_deg
                if tilt_deg > 45.0:
                    self.tipped = True

                clr = signed_footprint_clearance(p.x, p.y, yaw, self.rock_pts, self.rock_meta)
                if clr < self.min_clearance:
                    self.min_clearance = clr
                if clr < -0.05:
                    self.hard_collision = True

                if self.plan_pts.shape[0] >= 2:
                    cte = point_to_polyline_dist(p.x, p.y, self.plan_pts)
                    if math.isfinite(cte):
                        self.tracking_errs.append(cte)


def load_completed_keys(csv_path: pathlib.Path, include_level: bool = False) -> Set[Tuple]:
    done: Set[Tuple] = set()
    if not csv_path.exists() or csv_path.stat().st_size == 0:
        return done
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                if include_level:
                    done.add((int(row["level"]), row["combo"], row["scenario"], int(row["seed"])))
                else:
                    done.add((row["combo"], row["scenario"], int(row["seed"])))
            except Exception:
                continue
    return done


def append_csv_row(csv_path: pathlib.Path, row: Dict, fieldnames: Optional[List[str]] = None) -> None:
    cols = fieldnames if fieldnames is not None else CSV_COLUMNS
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    write_header = (not csv_path.exists()) or (csv_path.stat().st_size == 0)
    with csv_path.open("a", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=cols)
        if write_header:
            writer.writeheader()
        writer.writerow(row)
        f.flush()
        os.fsync(f.fileno())


def start_bag_record(bag_dir: pathlib.Path) -> Optional[subprocess.Popen]:
    if bag_dir.exists():
        # Avoid ros2 bag failure on pre-existing output folder from an interrupted run.
        import shutil
        shutil.rmtree(bag_dir, ignore_errors=True)
    bag_dir.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "ros2",
        "bag",
        "record",
        "-s",
        "mcap",
        "--disable-keyboard-controls",
        "-o",
        str(bag_dir),
        *BAG_TOPICS,
        # e.g. BAG_EXTRA_TOPICS="/global_costmap/costmap /hazards" for one figure run; off by default (size)
        *os.environ.get("BAG_EXTRA_TOPICS", "").split(),
    ]
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    time.sleep(0.35)
    return proc


def stop_bag_record(proc: Optional[subprocess.Popen]) -> None:
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=6.0)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=3.0)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass


def compute_perturbed_start(scenario: dict, sc_index: int, seed: int) -> Tuple[float, float, float]:
    sx = float(scenario["start"]["x"])
    sy = float(scenario["start"]["y"])
    syaw = float(scenario["start"]["yaw"])
    if seed == 0:
        return sx, sy, syaw
    rng = np.random.default_rng(seed * 1000 + sc_index)
    dx = float(rng.uniform(-0.05, 0.05))
    dy = float(rng.uniform(-0.05, 0.05))
    dyaw = math.radians(float(rng.uniform(-2.0, 2.0)))
    return sx + dx, sy + dy, wrap_angle(syaw + dyaw)


def filter_scenarios(scenarios: List[dict], filter_str: str) -> List[Tuple[int, dict]]:
    indexed = list(enumerate(scenarios))
    if not filter_str.strip():
        return indexed
    tokens = {t.strip() for t in filter_str.split(",") if t.strip()}
    return [(i, sc) for i, sc in indexed if sc["id"] in tokens or sc["group"] in tokens]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Run Mars Yard Nav2 benchmark for one planner x controller combo.")
    parser.add_argument("--planner", required=True, choices=("navfn", "smac_hybrid"))
    parser.add_argument("--controller", required=True, choices=("dwb", "mppi", "rpp"))
    parser.add_argument("--scenarios", default="", help="Path to scenarios.yaml (defaults to package share).")
    parser.add_argument("--scenario-filter", default="", help="Comma-separated scenario IDs or groups.")
    parser.add_argument("--seeds", type=int, default=5, help="Number of seeds 0..seeds-1 (default 5).")
    parser.add_argument("--seed-list", default="", help="Optional comma-separated explicit seed integers.")
    parser.add_argument("--output-csv", default="results/benchmark/benchmark.csv")
    parser.add_argument("--bags-dir", default="results/benchmark/bags")
    parser.add_argument("--no-bag", action="store_true", help="Disable MCAP bag recording.")
    parser.add_argument("--sim-timeout", type=float, default=120.0)
    parser.add_argument("--repo", default="", help="Repo root (for obstacles.json fallback).")
    parser.add_argument(
        "--level",
        type=int,
        default=None,
        choices=(0, 1, 2, 3),
        help="Optional domain randomization level (adds 'level' column to CSV and bag prefix).",
    )
    args = parser.parse_args(argv)

    combo = f"{args.planner}+{args.controller}"
    combo_safe = f"{args.planner}_{args.controller}"
    use_level = args.level is not None
    csv_columns = (CSV_COLUMNS + ["level"]) if use_level else CSV_COLUMNS
    log_tag = f"L{args.level} {combo}" if use_level else combo

    if args.scenarios:
        scenarios_path = pathlib.Path(args.scenarios)
    else:
        scenarios_path = pathlib.Path(get_package_share_directory("mars_yard_benchmark")) / "config" / "scenarios.yaml"

    if args.repo:
        obs_path = pathlib.Path(args.repo) / "src/mars_yard_worlds/terrain/obstacles.json"
    else:
        obs_path = pathlib.Path(get_package_share_directory("mars_yard_worlds")) / "terrain" / "obstacles.json"

    sc_data = yaml.safe_load(scenarios_path.read_text(encoding="utf-8"))
    all_scenarios = sc_data["scenarios"]
    selected = filter_scenarios(all_scenarios, args.scenario_filter)
    if not selected:
        print(f"ERROR: scenario filter '{args.scenario_filter}' matched 0 scenarios.", flush=True)
        return 1

    if args.seed_list.strip():
        seeds = [int(s.strip()) for s in args.seed_list.split(",") if s.strip()]
    else:
        seeds = list(range(args.seeds))

    csv_path = pathlib.Path(args.output_csv)
    bags_root = pathlib.Path(args.bags_dir)
    completed = load_completed_keys(csv_path, include_level=use_level)

    if use_level:
        todo = [
            (idx, sc, sd)
            for idx, sc in selected
            for sd in seeds
            if (args.level, combo, sc["id"], sd) not in completed
        ]
    else:
        todo = [
            (idx, sc, sd)
            for idx, sc in selected
            for sd in seeds
            if (combo, sc["id"], sd) not in completed
        ]
    if not todo:
        print(f"[{log_tag}] All {len(selected) * len(seeds)} requested runs already present in {csv_path}.", flush=True)
        return 0

    rocks = json.loads(obs_path.read_text(encoding="utf-8"))["rocks"]
    rock_pts, rock_meta = build_hazard_rock_contours(rocks, min_exposed_height=0.12)

    print(
        f"[{log_tag}] Starting benchmark: {len(todo)} runs to execute "
        f"({len(selected) * len(seeds) - len(todo)} already in {csv_path}), "
        f"sim_timeout={args.sim_timeout:.0f}s, bags={'off' if args.no_bag else str(bags_root)}",
        flush=True,
    )

    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    nav = BasicNavigator()
    mon = BenchmarkMonitor(rock_pts, rock_meta)
    resetter = SimResetter(node_name="benchmark_reset_helper")

    stop_spin = threading.Event()
    executor = SingleThreadedExecutor()
    executor.add_node(mon)

    def _spin():
        while not stop_spin.is_set() and rclpy.ok():
            executor.spin_once(timeout_sec=0.02)

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    exit_code = 0
    try:
        # Wait for Nav2 active with bounded retries against FastDDS startup discovery races.
        t_wait = time.monotonic()
        state_cli = nav.create_client(GetState, "bt_navigator/get_state")
        active = False
        while (time.monotonic() - t_wait) < 45.0:
            if not state_cli.wait_for_service(timeout_sec=1.0):
                continue
            fut = state_cli.call_async(GetState.Request())
            rclpy.spin_until_future_complete(nav, fut, timeout_sec=2.0)
            if fut.done() and fut.result() is not None and fut.result().current_state.label == "active":
                active = True
                break
            time.sleep(0.3)
        nav.destroy_client(state_cli)
        if not active:
            print(f"[{combo}] ERROR: bt_navigator did not become active.", flush=True)
            return 3
        nav.waitUntilNav2Active(navigator="bt_navigator", localizer="robot_localization")

        if not resetter.wait_ready(timeout_s=20.0):
            print(f"[{combo}] ERROR: simulation topics not ready.", flush=True)
            return 3

        for run_num, (sc_idx, sc, seed) in enumerate(todo, start=1):
            sc_id = sc["id"]
            group = sc["group"]
            straight_dist = float(sc["straight_dist"])
            gx = float(sc["goal"]["x"])
            gy = float(sc["goal"]["y"])
            gyaw = float(sc["goal"]["yaw"])
            sx, sy, syaw = compute_perturbed_start(sc, sc_idx, seed)

            try:
                settled = resetter.reset(sx, sy, syaw, nav=nav)
            except Exception as exc:
                print(f"[{combo}] ERROR during reset for {sc_id} seed={seed}: {exc}", flush=True)
                exit_code = 3
                break

            bag_proc = None
            if not args.no_bag:
                bag_name = f"L{args.level}__{combo_safe}__{sc_id}__s{seed}" if use_level else f"{combo_safe}__{sc_id}__s{seed}"
                bag_dir = bags_root / bag_name
                bag_proc = start_bag_record(bag_dir)

            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = nav.get_clock().now().to_msg()
            goal.pose.position.x = gx
            goal.pose.position.y = gy
            qx, qy, qz, qw = quat_from_yaw(gyaw)
            goal.pose.orientation.x = qx
            goal.pose.orientation.y = qy
            goal.pose.orientation.z = qz
            goal.pose.orientation.w = qw

            sim_t0 = mon.start_run()
            wall_t0 = time.monotonic()
            nav.goToPose(goal)

            failure_trigger = ""
            max_recoveries = 0
            stack_unhealthy = False
            wall_limit = max(180.0, args.sim_timeout * 1.8)

            while not nav.isTaskComplete():
                fb = nav.getFeedback()
                if fb is not None:
                    max_recoveries = max(max_recoveries, int(fb.number_of_recoveries))
                sim_elapsed, clock_age, tipped, hard_col, stuck, _ = mon.check_live_status(sim_t0)
                wall_elapsed = time.monotonic() - wall_t0

                if clock_age > 10.0:
                    print(f"[{combo}] ERROR: /clock stalled for {clock_age:.1f}s during {sc_id} s{seed}.", flush=True)
                    stack_unhealthy = True
                    break
                if tipped:
                    failure_trigger = "tipover"
                    break
                if hard_col:
                    failure_trigger = "collision"
                    break
                if stuck:
                    failure_trigger = "stuck"
                    break
                if sim_elapsed >= args.sim_timeout or wall_elapsed >= wall_limit:
                    failure_trigger = "timeout"
                    break
                time.sleep(0.05)

            if failure_trigger or stack_unhealthy:
                try:
                    nav.cancelTask()
                    t_c = time.monotonic()
                    while not nav.isTaskComplete() and (time.monotonic() - t_c) < 4.0:
                        time.sleep(0.05)
                except Exception:
                    pass

            snap = mon.stop_run()
            wall_elapsed = time.monotonic() - wall_t0
            sim_elapsed = max(0.0, snap["sim_time"] - sim_t0)
            stop_bag_record(bag_proc)

            if stack_unhealthy:
                exit_code = 3
                break

            task_res = nav.getResult()
            min_clr = snap["min_clearance"]
            if not failure_trigger:
                if min_clr < 0.0:
                    failure_trigger = "collision"
                elif snap["tipped"]:
                    failure_trigger = "tipover"
                elif task_res == TaskResult.SUCCEEDED:
                    failure_trigger = "none"
                elif task_res == TaskResult.FAILED:
                    failure_trigger = "aborted"
                else:
                    failure_trigger = "aborted"

            result_str = "SUCCESS" if failure_trigger == "none" else "FAILURE"

            trace = snap["gt_trace"]
            path_len_gt = 0.0
            for i in range(1, len(trace)):
                path_len_gt += math.hypot(trace[i][1] - trace[i - 1][1], trace[i][2] - trace[i - 1][2])
            path_ratio = path_len_gt / straight_dist if straight_dist > 1e-6 else float("nan")

            gp = snap["gt"].pose.pose.position
            gt_goal_err = math.hypot(gp.x - gx, gp.y - gy)
            tracking_err = float(np.mean(snap["tracking_errs"])) if snap["tracking_errs"] else float("nan")
            metrics = snap["metrics"]
            rtf = sim_elapsed / wall_elapsed if wall_elapsed > 1e-6 else 0.0

            row = {
                "combo": combo,
                "scenario": sc_id,
                "group": group,
                "seed": seed,
                "result": result_str,
                "failure_type": failure_trigger,
                "time_s": round(sim_elapsed, 3),
                "path_len_gt": round(path_len_gt, 4),
                "straight_dist": round(straight_dist, 4),
                "path_ratio": round(path_ratio, 4),
                "gt_goal_err": round(gt_goal_err, 4),
                "tracking_err": round(tracking_err, 4) if math.isfinite(tracking_err) else "",
                "slip_ratio": round(float(metrics.get("slip_ratio", float("nan"))), 5),
                "lateral_slide": round(float(metrics.get("lateral_slide", float("nan"))), 4),
                "ekf_drift": round(float(metrics.get("ekf_pos_drift", float("nan"))), 4),
                "energy_J": round(float(metrics.get("energy", float("nan"))), 2),
                "min_obstacle_clearance": round(min_clr, 4) if math.isfinite(min_clr) else 99.0,
                "recoveries": max_recoveries,
                "rtf": round(rtf, 4),
            }
            if use_level:
                row["level"] = args.level
            append_csv_row(csv_path, row, fieldnames=csv_columns)

            print(
                f"  [{run_num:3d}/{len(todo)}] {log_tag:19s} {sc_id:8s} ({group:5s}) s{seed} -> "
                f"{result_str:7s} ({failure_trigger:9s}) t={sim_elapsed:6.2f}s ratio={path_ratio:.2f} "
                f"err={gt_goal_err:.3f}m cte={row['tracking_err']} clr={row['min_obstacle_clearance']:.2f}m "
                f"E={row['energy_J']:.0f}J rec={max_recoveries} RTF={rtf:.3f}x (reset={settled.reset_wall_s:.2f}s)",
                flush=True,
            )

            # Health check: if RTF dropped below 0.9 on a real drive, exit with code 3 so matrix relaunches.
            if sim_elapsed >= 5.0 and rtf < 0.9:
                print(f"[{combo}] WARNING: RTF={rtf:.3f}x < 0.9; signaling stack relaunch.", flush=True)
                exit_code = 3
                break

        return exit_code
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

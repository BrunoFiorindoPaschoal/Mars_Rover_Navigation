#!/usr/bin/env python3
"""In-simulation verification runner for Phase 7 domain randomization (L1-L3).

Inside a running sim.launch.py (world:=mars nav:=true rand_level:=L rand_seed:=L),
this script verifies:
  1. /points_noisy vs /points: radial noise sigma and point dropout fraction.
  2. /imu_sensor_broadcaster/imu_noisy vs /imu_sensor_broadcaster/imu: gyro noise sigma & z bias.
  3. twist_stamper command latency from /cmd_vel step publish sim time vs /diff_drive_controller/cmd_vel header.stamp.
  4. Per-wheel motor gain & asymmetry via a short open-loop straight drive (0.4 m/s for 3 s) reading /joint_states & GT yaw.
  5. Resets rover to (0, 0, 0) via SimResetter and runs one Nav2 goal (9.0, -0.6, 0.0).
"""

import argparse
import json
import math
import pathlib
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, PointCloud2, PointField
from std_msgs.msg import Float64

from mars_yard_benchmark.sim_reset import SimResetter, quat_from_yaw, rpy_from_quat, stamp_to_sec, wrap_angle


WHEEL_NAMES = ("wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint")
WHEEL_RADIUS = 0.12


def _parse_xyz(msg: PointCloud2) -> np.ndarray:
    n_pts = msg.width * msg.height
    if n_pts == 0 or not msg.data:
        return np.empty((0, 3), dtype=np.float32)
    offsets = {}
    for f in msg.fields:
        if f.name in ("x", "y", "z") and f.datatype == PointField.FLOAT32:
            offsets[f.name] = f.offset
    if len(offsets) != 3:
        return np.empty((0, 3), dtype=np.float32)
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": ["<f4", "<f4", "<f4"],
            "offsets": [offsets["x"], offsets["y"], offsets["z"]],
            "itemsize": msg.point_step,
        }
    )
    arr = np.frombuffer(msg.data, dtype=dtype, count=n_pts)
    return np.column_stack((arr["x"], arr["y"], arr["z"]))


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class RandProbe(Node):
    def __init__(self) -> None:
        super().__init__("check_randomization_probe")
        self.lock = threading.Lock()
        self.sim_time: Optional[float] = None
        self.gt: Optional[Odometry] = None
        self.ekf: Optional[Odometry] = None
        self.metrics: Dict[str, float] = {}

        # Sensor pair buffers keyed by exact header stamp (ns)
        self.collect_sensors = False
        self.raw_points: Dict[int, np.ndarray] = {}
        self.noisy_points: Dict[int, np.ndarray] = {}
        self.raw_imu: Dict[int, Tuple[float, float, float]] = {}
        self.noisy_imu: Dict[int, Tuple[float, float, float]] = {}

        # Command latency capture
        self.watch_latency = False
        self.cmd_events: List[Tuple[float, float]] = []  # (stamp_s, vx)

        # Joint velocity capture during open-loop drive
        self.collect_joints = False
        self.wheel_vels: Dict[str, List[float]] = {w: [] for w in WHEEL_NAMES}

        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_cloud = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Clock, "/clock", self._on_clock, qos_rel)
        self.create_subscription(Odometry, "/ground_truth/odom", self._on_gt, qos_rel)
        self.create_subscription(Odometry, "/odometry/filtered", self._on_ekf, qos_rel)
        self.create_subscription(PointCloud2, "/points", self._on_raw_pts, qos_cloud)
        self.create_subscription(PointCloud2, "/points_noisy", self._on_noisy_pts, qos_cloud)
        self.create_subscription(Imu, "/imu_sensor_broadcaster/imu", self._on_raw_imu, 100)
        self.create_subscription(Imu, "/imu_sensor_broadcaster/imu_noisy", self._on_noisy_imu, 100)
        self.create_subscription(TwistStamped, "/diff_drive_controller/cmd_vel", self._on_cmd_out, 50)
        self.create_subscription(JointState, "/joint_states", self._on_js, 50)

        for name in ("ekf_pos_drift", "ekf_yaw_drift", "lateral_slide", "energy", "true_distance"):
            self.create_subscription(
                Float64,
                f"/metrics/{name}",
                lambda msg, n=name: self._on_metric(n, msg.data),
                qos_rel,
            )

    def _on_clock(self, msg: Clock) -> None:
        with self.lock:
            self.sim_time = stamp_to_sec(msg.clock)

    def _on_gt(self, msg: Odometry) -> None:
        with self.lock:
            self.gt = msg

    def _on_ekf(self, msg: Odometry) -> None:
        with self.lock:
            self.ekf = msg

    def _on_metric(self, name: str, val: float) -> None:
        with self.lock:
            self.metrics[name] = float(val)

    def _on_raw_pts(self, msg: PointCloud2) -> None:
        with self.lock:
            if not self.collect_sensors:
                return
        pts = _parse_xyz(msg)
        with self.lock:
            if self.collect_sensors and len(self.raw_points) < 40:
                self.raw_points[_stamp_ns(msg.header.stamp)] = pts

    def _on_noisy_pts(self, msg: PointCloud2) -> None:
        with self.lock:
            if not self.collect_sensors:
                return
        pts = _parse_xyz(msg)
        with self.lock:
            if self.collect_sensors and len(self.noisy_points) < 40:
                self.noisy_points[_stamp_ns(msg.header.stamp)] = pts

    def _on_raw_imu(self, msg: Imu) -> None:
        with self.lock:
            if self.collect_sensors and len(self.raw_imu) < 600:
                w = msg.angular_velocity
                self.raw_imu[_stamp_ns(msg.header.stamp)] = (w.x, w.y, w.z)

    def _on_noisy_imu(self, msg: Imu) -> None:
        with self.lock:
            if self.collect_sensors and len(self.noisy_imu) < 600:
                w = msg.angular_velocity
                self.noisy_imu[_stamp_ns(msg.header.stamp)] = (w.x, w.y, w.z)

    def _on_cmd_out(self, msg: TwistStamped) -> None:
        with self.lock:
            if self.watch_latency:
                self.cmd_events.append((stamp_to_sec(msg.header.stamp), float(msg.twist.linear.x)))

    def _on_js(self, msg: JointState) -> None:
        with self.lock:
            if not self.collect_joints:
                return
            for name, vel in zip(msg.name, msg.velocity):
                if name in self.wheel_vels:
                    self.wheel_vels[name].append(float(vel))


def _match_cloud_pair(raw: np.ndarray, noisy: np.ndarray) -> Tuple[int, int, np.ndarray]:
    """Match subsequence noisy rays to raw rays in O(N) and return (n_raw, n_noisy, dr_array)."""
    n_raw = raw.shape[0]
    n_noisy = noisy.shape[0]
    if n_raw == 0 or n_noisy == 0:
        return n_raw, n_noisy, np.empty(0, dtype=np.float64)

    r_raw = np.linalg.norm(raw.astype(np.float64), axis=1)
    r_noisy = np.linalg.norm(noisy.astype(np.float64), axis=1)
    u_raw = raw.astype(np.float64) / np.maximum(r_raw[:, None], 1e-9)
    u_noisy = noisy.astype(np.float64) / np.maximum(r_noisy[:, None], 1e-9)

    drs: List[float] = []
    j = 0
    for i in range(n_noisy):
        ui = u_noisy[i]
        while j < n_raw:
            if float(np.dot(ui, u_raw[j])) > 0.9999:
                drs.append(float(r_noisy[i] - r_raw[j]))
                j += 1
                break
            j += 1
    return n_raw, n_noisy, np.asarray(drs, dtype=np.float64)


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify Phase 7 domain randomization inside live sim.")
    parser.add_argument("--level", type=int, required=True, choices=(1, 2, 3))
    parser.add_argument("--params-json", type=pathlib.Path, required=True)
    parser.add_argument("--repo", required=True)
    parser.add_argument("--goal-x", type=float, default=9.0)
    parser.add_argument("--goal-y", type=float, default=-0.6)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--sim-timeout", type=float, default=120.0)
    args = parser.parse_args()

    params = json.loads(args.params_json.read_text(encoding="utf-8"))
    print(f"=== Verifying L{args.level} (seed={params['seed']}) ===", flush=True)

    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    nav = BasicNavigator()
    probe = RandProbe()
    resetter = SimResetter(node_name="check_rand_reset")

    stop_spin = threading.Event()
    executor = SingleThreadedExecutor()
    executor.add_node(probe)

    def _spin():
        while not stop_spin.is_set() and rclpy.ok():
            executor.spin_once(timeout_sec=0.005)

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    failures: List[str] = []
    try:
        # 1. Wait for Nav2 active + sim ready
        t0 = time.monotonic()
        state_cli = nav.create_client(GetState, "bt_navigator/get_state")
        while (time.monotonic() - t0) < 40.0:
            if not state_cli.wait_for_service(timeout_sec=1.0):
                continue
            fut = state_cli.call_async(GetState.Request())
            rclpy.spin_until_future_complete(nav, fut, timeout_sec=2.0)
            if fut.done() and fut.result() is not None and fut.result().current_state.label == "active":
                break
            time.sleep(0.3)
        nav.destroy_client(state_cli)
        nav.waitUntilNav2Active(navigator="bt_navigator", localizer="robot_localization")
        if not resetter.wait_ready(timeout_s=20.0):
            print("ERROR: simulation topics not ready.", flush=True)
            return 1

        # 2. Collect raw vs noisy /points and IMU for 3.2 s sim
        with probe.lock:
            probe.collect_sensors = True
            t_start = probe.sim_time
        while (probe.sim_time - t_start) < 3.2:
            time.sleep(0.02)
        with probe.lock:
            probe.collect_sensors = False
            raw_pts_map = dict(probe.raw_points)
            noisy_pts_map = dict(probe.noisy_points)
            raw_imu_map = dict(probe.raw_imu)
            noisy_imu_map = dict(probe.noisy_imu)

        # Analyze lidar noise & dropout
        common_pts = sorted(set(raw_pts_map) & set(noisy_pts_map))
        total_raw = 0
        total_noisy = 0
        all_drs = []
        for ts in common_pts:
            nr, nn, dr = _match_cloud_pair(raw_pts_map[ts], noisy_pts_map[ts])
            total_raw += nr
            total_noisy += nn
            if dr.size > 0:
                all_drs.append(dr)
        dr_cat = np.concatenate(all_drs) if all_drs else np.empty(0, dtype=np.float64)
        meas_dropout = (1.0 - total_noisy / total_raw) if total_raw > 0 else float("nan")
        meas_lidar_sigma = float(np.std(dr_cat)) if dr_cat.size > 10 else float("nan")

        exp_dropout = float(params["lidar_dropout"])
        exp_lidar_sigma = float(params["lidar_noise_sigma_m"])
        print(
            f"[LIDAR] paired_scans={len(common_pts)}, matched_pts={dr_cat.size}/{total_raw} | "
            f"dropout: meas={meas_dropout:.4f} vs exp={exp_dropout:.4f} | "
            f"range_sigma: meas={meas_lidar_sigma:.4f} m vs exp={exp_lidar_sigma:.4f} m",
            flush=True,
        )
        if len(common_pts) < 5 or dr_cat.size < 1000:
            failures.append(f"Too few paired lidar scans/points ({len(common_pts)} scans, {dr_cat.size} pts)")
        else:
            if abs(meas_dropout - exp_dropout) > max(0.015, 0.25 * exp_dropout):
                failures.append(f"Lidar dropout mismatch: meas={meas_dropout:.4f} vs exp={exp_dropout:.4f}")
            if abs(meas_lidar_sigma - exp_lidar_sigma) > max(0.002, 0.15 * exp_lidar_sigma):
                failures.append(f"Lidar sigma mismatch: meas={meas_lidar_sigma:.4f} vs exp={exp_lidar_sigma:.4f}")

        # Analyze IMU gyro noise & z bias
        common_imu = sorted(set(raw_imu_map) & set(noisy_imu_map))
        diffs = np.array(
            [
                [
                    noisy_imu_map[ts][0] - raw_imu_map[ts][0],
                    noisy_imu_map[ts][1] - raw_imu_map[ts][1],
                    noisy_imu_map[ts][2] - raw_imu_map[ts][2],
                ]
                for ts in common_imu
            ],
            dtype=np.float64,
        )
        meas_gyro_sigma = float(np.std(diffs[:, :2])) if diffs.shape[0] > 20 else float("nan")
        meas_gyro_z_sigma = float(np.std(diffs[:, 2])) if diffs.shape[0] > 20 else float("nan")
        meas_gyro_z_bias = float(np.mean(diffs[:, 2])) if diffs.shape[0] > 20 else float("nan")

        exp_gyro_sigma = float(params["imu_gyro_sigma_rad_s"])
        exp_gyro_z_bias = float(params["imu_gyro_z_bias_rad_s"])
        print(
            f"[IMU]   paired_samples={len(common_imu)} | "
            f"gyro_sigma(xy/z): meas={meas_gyro_sigma:.5f}/{meas_gyro_z_sigma:.5f} vs exp={exp_gyro_sigma:.5f} rad/s | "
            f"z_bias: meas={meas_gyro_z_bias:+.5f} vs exp={exp_gyro_z_bias:+.5f} rad/s",
            flush=True,
        )
        if len(common_imu) < 100:
            failures.append(f"Too few paired IMU samples ({len(common_imu)})")
        else:
            if abs(meas_gyro_sigma - exp_gyro_sigma) > max(0.0008, 0.25 * exp_gyro_sigma):
                failures.append(f"IMU gyro sigma mismatch: meas={meas_gyro_sigma:.5f} vs exp={exp_gyro_sigma:.5f}")
            # Standard error of z mean is sigma / sqrt(N) ~ 0.01/sqrt(300) = 0.0006
            bias_tol = max(0.001, 4.0 * exp_gyro_sigma / math.sqrt(len(common_imu)))
            if abs(meas_gyro_z_bias - exp_gyro_z_bias) > bias_tol:
                failures.append(f"IMU z_bias mismatch: meas={meas_gyro_z_bias:+.5f} vs exp={exp_gyro_z_bias:+.5f} (tol={bias_tol:.5f})")

        # 3. Command latency + open-loop straight drive (motor gain & asymmetry check)
        zero = Twist()
        for _ in range(5):
            probe.cmd_pub.publish(zero)
            time.sleep(0.02)
        time.sleep(0.25)

        with probe.lock:
            probe.cmd_events.clear()
            probe.watch_latency = True
            gt0 = probe.gt
            p0 = gt0.pose.pose.position
            _, _, yaw0 = rpy_from_quat(gt0.pose.pose.orientation)
            t_step_pub = probe.sim_time

        cmd_fwd = Twist()
        cmd_fwd.linear.x = 0.4
        probe.cmd_pub.publish(cmd_fwd)

        # Wait until delayed command appears on /diff_drive_controller/cmd_vel, while continuing to drive 3.0 s
        t_drive0 = t_step_pub
        measured_latency = None
        last_pub = time.monotonic()
        while True:
            now_w = time.monotonic()
            if now_w - last_pub >= 0.05:
                probe.cmd_pub.publish(cmd_fwd)
                last_pub = now_w
            with probe.lock:
                sim_el = probe.sim_time - t_drive0
                if measured_latency is None:
                    for ts_out, vx_out in probe.cmd_events:
                        if vx_out >= 0.35:
                            measured_latency = ts_out - t_step_pub
                            probe.watch_latency = False
                            break
                if 1.2 <= sim_el <= 2.8:
                    probe.collect_joints = True
                elif sim_el > 2.8:
                    probe.collect_joints = False
            if sim_el >= 3.0:
                break
            time.sleep(0.005)

        probe.cmd_pub.publish(zero)
        with probe.lock:
            probe.watch_latency = False
            probe.collect_joints = False
            gt1 = probe.gt
            p1 = gt1.pose.pose.position
            _, _, yaw1 = rpy_from_quat(gt1.pose.pose.orientation)
            w_vels = {w: list(probe.wheel_vels[w]) for w in WHEEL_NAMES}

        exp_latency = float(params["cmd_latency_s"])
        if measured_latency is None:
            print("[LATENCY] ERROR: did not observe step on /diff_drive_controller/cmd_vel", flush=True)
            failures.append("Command latency step not observed")
        else:
            print(
                f"[LATENCY] meas={measured_latency * 1000.0:.1f} ms vs exp={exp_latency * 1000.0:.1f} ms",
                flush=True,
            )
            if abs(measured_latency - exp_latency) > 0.020:
                failures.append(
                    f"Command latency mismatch: meas={measured_latency*1000:.1f}ms vs exp={exp_latency*1000:.1f}ms"
                )

        omega_cmd = 0.4 / WHEEL_RADIUS
        exp_gains = params["motor_gain_scale"]
        meas_gains = {}
        for w in WHEEL_NAMES:
            meas_gains[w] = (float(np.mean(w_vels[w])) / omega_cmd) if w_vels[w] else float("nan")
        dist_fwd = math.hypot(p1.x - p0.x, p1.y - p0.y)
        dyaw_deg = math.degrees(wrap_angle(yaw1 - yaw0))
        print(
            f"[DRIVE] 3.0s open-loop @0.4 m/s: dist={dist_fwd:.3f} m (nom ~1.18 m), dyaw={dyaw_deg:+.2f} deg | "
            f"mass={params['total_mass_kg']:.2f} kg, fric={params['wheel_sliding_friction']:.3f}",
            flush=True,
        )
        max_gain_err = 0.0
        for w in WHEEL_NAMES:
            err = abs(meas_gains[w] - exp_gains[w])
            max_gain_err = max(max_gain_err, err)
            print(
                f"        {w:14s}: meas_gain={meas_gains[w]:.4f} vs exp={exp_gains[w]:.4f} (diff={err:.4f})",
                flush=True,
            )
        if not math.isfinite(max_gain_err) or max_gain_err > 0.06:
            failures.append(f"Motor gain tracking mismatch: max_err={max_gain_err:.4f} > 0.06")

        # 4. Reset back to (0, 0, 0) and run one Nav2 goal
        settled = resetter.reset(0.0, 0.0, 0.0, nav=nav)
        print(
            f"[RESET] settled at ({settled.x:+.3f}, {settled.y:+.3f}, yaw={math.degrees(settled.yaw):+.2f} deg) "
            f"in {settled.reset_wall_s:.2f}s wall (ekf_err={settled.ekf_pos_err_m:.4f}m)",
            flush=True,
        )

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

        with probe.lock:
            sim_t0 = probe.sim_time
        wall_t0 = time.monotonic()
        nav.goToPose(goal)

        timed_out = False
        max_rec = 0
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb is not None:
                max_rec = max(max_rec, int(fb.number_of_recoveries))
            with probe.lock:
                sim_el = probe.sim_time - sim_t0
            wall_el = time.monotonic() - wall_t0
            if sim_el >= args.sim_timeout or wall_el >= args.sim_timeout * 1.8:
                timed_out = True
                nav.cancelTask()
                t_c = time.monotonic()
                while not nav.isTaskComplete() and (time.monotonic() - t_c) < 5.0:
                    time.sleep(0.05)
                break
            time.sleep(0.05)

        with probe.lock:
            sim_el = probe.sim_time - sim_t0
            gp = probe.gt.pose.pose.position
            ep = probe.ekf.pose.pose.position
            met = dict(probe.metrics)
        wall_el = time.monotonic() - wall_t0
        rtf = sim_el / wall_el if wall_el > 0 else 0.0
        task_res = nav.getResult()
        if timed_out:
            res_str = "TIMEOUT"
        elif task_res == TaskResult.SUCCEEDED:
            res_str = "SUCCEEDED"
        else:
            res_str = str(task_res)
        gt_err = math.hypot(gp.x - args.goal_x, gp.y - args.goal_y)
        ekf_err = math.hypot(ep.x - args.goal_x, ep.y - args.goal_y)
        print(
            f"[NAV2]  L{args.level} (smac_hybrid+dwb -> ({args.goal_x},{args.goal_y})): "
            f"result={res_str} (rec={max_rec}) sim={sim_el:.2f}s RTF={rtf:.4f}x | "
            f"GT_err={gt_err:.3f}m EKF_err={ekf_err:.3f}m ekf_drift={met.get('ekf_pos_drift', float('nan')):.3f}m "
            f"E={met.get('energy', float('nan')):.1f}J",
            flush=True,
        )
        if rtf < 0.9:
            failures.append(f"RTF too low during Nav2 goal: {rtf:.3f}x")

        if failures:
            print(f"L{args.level} VERIFICATION FAILED:", flush=True)
            for f in failures:
                print(f"  - {f}", flush=True)
            return 2

        print(f"L{args.level} VERIFICATION PASSED.", flush=True)
        return 0
    finally:
        stop_spin.set()
        spin_thread.join(timeout=2.0)
        executor.remove_node(probe)
        probe.destroy_node()
        resetter.destroy()
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

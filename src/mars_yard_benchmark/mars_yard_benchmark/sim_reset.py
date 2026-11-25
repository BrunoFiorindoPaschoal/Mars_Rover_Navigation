"""In-simulation rover teleport and stack reset helper for back-to-back benchmark runs.

Reset sequence:
  1. Cancel any active Nav2 task.
  2. Publish zero Twist on /cmd_vel and /cmd_vel_nav to stop the wheels and smoother.
  3. Publish Pose2D on /sim/teleport and wait for /sim/teleport_done (Header).
  4. Wait until /ground_truth/odom linear speed < 0.02 m/s and |wz| < 0.02 rad/s
     after landing (max 5 s sim).
  5. Publish the true settled pose on /set_pose (frame: odom) to reset the EKF and
     confirm /odometry/filtered converges within 0.05 m.
  6. Clear hazard_mapper rolling memory (/hazard_mapper/clear) and clear both Nav2
     costmaps (global + local ClearEntireCostmap).
  7. Wait briefly for fresh local perception, then reset /metrics_node/reset.
"""

from dataclasses import dataclass
import math
import time
from typing import Optional, Tuple

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped, Twist
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header
from std_srvs.srv import Empty


def wrap_angle(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


def quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def rpy_from_quat(q) -> Tuple[float, float, float]:
    n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if n < 1e-12:
        return 0.0, 0.0, 0.0
    x, y, z, w = q.x / n, q.y / n, q.z / n, q.w / n
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def sec_to_stamp(sec: float) -> TimeMsg:
    ns = max(0, int(round(sec * 1e9)))
    msg = TimeMsg()
    msg.sec = int(ns // 1_000_000_000)
    msg.nanosec = int(ns % 1_000_000_000)
    return msg


@dataclass
class SettledPose:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    sim_time: float
    settle_sim_s: float
    reset_wall_s: float
    ekf_pos_err_m: float
    ekf_yaw_err_rad: float
    settled: bool


class SimResetter:
    """Reusable ROS 2 helper that performs a full in-sim reset between runs."""

    def __init__(self, node_name: str = "sim_reset_helper") -> None:
        self.node = Node(
            node_name,
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._cmd_vel_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self._cmd_vel_nav_pub = self.node.create_publisher(Twist, "/cmd_vel_nav", 10)
        self._teleport_pub = self.node.create_publisher(Pose2D, "/sim/teleport", qos)
        self._set_pose_pub = self.node.create_publisher(PoseWithCovarianceStamped, "/set_pose", qos)

        self._clear_global_cli = self.node.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap"
        )
        self._clear_local_cli = self.node.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap"
        )
        self._clear_hazards_cli = self.node.create_client(Empty, "/hazard_mapper/clear")
        self._reset_metrics_cli = self.node.create_client(Empty, "/metrics_node/reset")

        self._sim_time: Optional[float] = None
        self._gt: Optional[Odometry] = None
        self._ekf: Optional[Odometry] = None
        self._done_count: int = 0
        self._last_done_stamp: float = -1.0

        self.node.create_subscription(Clock, "/clock", self._on_clock, qos)
        self.node.create_subscription(Odometry, "/ground_truth/odom", self._on_gt, qos)
        self.node.create_subscription(Odometry, "/odometry/filtered", self._on_ekf, qos)
        self.node.create_subscription(Header, "/sim/teleport_done", self._on_done, qos)

    def destroy(self) -> None:
        self.node.destroy_node()

    def _on_clock(self, msg: Clock) -> None:
        self._sim_time = stamp_to_sec(msg.clock)

    def _on_gt(self, msg: Odometry) -> None:
        self._gt = msg

    def _on_ekf(self, msg: Odometry) -> None:
        self._ekf = msg

    def _on_done(self, msg: Header) -> None:
        self._done_count += 1
        self._last_done_stamp = stamp_to_sec(msg.stamp)

    def _spin_for_wall(self, duration_s: float, step_s: float = 0.01) -> None:
        t0 = time.monotonic()
        while (time.monotonic() - t0) < duration_s:
            rclpy.spin_once(self.node, timeout_sec=step_s)

    def _spin_for_sim(self, duration_s: float, max_wall_s: float = 5.0, publish_zero: bool = False) -> None:
        t_wall0 = time.monotonic()
        while self._sim_time is None and (time.monotonic() - t_wall0) < max_wall_s:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        if self._sim_time is None:
            return
        t_sim0 = self._sim_time
        last_pub = 0.0
        zero = Twist()
        while (self._sim_time - t_sim0) < duration_s and (time.monotonic() - t_wall0) < max_wall_s:
            now_w = time.monotonic()
            if publish_zero and (now_w - last_pub) >= 0.05:
                self._cmd_vel_nav_pub.publish(zero)
                self._cmd_vel_pub.publish(zero)
                last_pub = now_w
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def _publish_zero_cmd(self, repeats: int = 4, wall_span: float = 0.12) -> None:
        zero = Twist()
        dt = wall_span / max(1, repeats)
        for _ in range(repeats):
            self._cmd_vel_nav_pub.publish(zero)
            self._cmd_vel_pub.publish(zero)
            self._spin_for_wall(dt)

    def _call_service(self, client, request, name: str, timeout_s: float = 5.0) -> bool:
        if not client.wait_for_service(timeout_sec=timeout_s):
            self.node.get_logger().warn(f"Service '{name}' not available within {timeout_s:.1f}s.")
            return False
        fut = client.call_async(request)
        t0 = time.monotonic()
        while not fut.done() and (time.monotonic() - t0) < timeout_s:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        if not fut.done() or fut.result() is None:
            self.node.get_logger().warn(f"Service '{name}' call timed out after {timeout_s:.1f}s.")
            return False
        return True

    def _publish_ekf_set_pose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "odom"
        if self._sim_time is not None:
            msg.header.stamp = sec_to_stamp(self._sim_time)
        else:
            msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quat_from_yaw(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        cov = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            cov[i] = 1e-4
        msg.pose.covariance = cov
        self._set_pose_pub.publish(msg)

    def wait_ready(self, timeout_s: float = 20.0) -> bool:
        t0 = time.monotonic()
        while (time.monotonic() - t0) < timeout_s:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if (
                self._sim_time is not None
                and self._gt is not None
                and self._ekf is not None
                and self._teleport_pub.get_subscription_count() >= 1
                and self._set_pose_pub.get_subscription_count() >= 1
            ):
                return True
        return self._sim_time is not None and self._gt is not None and self._ekf is not None

    def reset(
        self,
        x: float,
        y: float,
        theta: float,
        nav=None,
        clear_costmaps: bool = True,
        clear_hazards: bool = True,
        reset_metrics: bool = True,
        max_settle_sim_s: float = 5.0,
        min_drop_sim_s: float = 0.45,
        stable_window_sim_s: float = 0.20,
        speed_tol: float = 0.02,
        wz_tol: float = 0.02,
        post_clear_sim_s: float = 0.35,
    ) -> SettledPose:
        """Teleport rover to (x, y, theta), settle, reset EKF/costmaps/hazards/metrics."""
        t_wall0 = time.monotonic()

        if not self.wait_ready(timeout_s=15.0):
            raise RuntimeError("SimResetter: timed out waiting for /clock, /ground_truth/odom, /odometry/filtered.")

        # 1. Cancel any active Nav2 task.
        if nav is not None:
            try:
                if not nav.isTaskComplete():
                    nav.cancelTask()
                    t_c = time.monotonic()
                    while not nav.isTaskComplete() and (time.monotonic() - t_c) < 5.0:
                        rclpy.spin_once(self.node, timeout_sec=0.02)
            except Exception:
                pass

        # 2. Publish zero /cmd_vel (and /cmd_vel_nav to flush velocity_smoother).
        self._publish_zero_cmd(repeats=4, wall_span=0.12)

        # 3. Publish /sim/teleport and wait for /sim/teleport_done.
        prev_done = self._done_count
        req_msg = Pose2D(x=float(x), y=float(y), theta=float(theta))
        self._teleport_pub.publish(req_msg)

        t_tp0 = time.monotonic()
        retried = False
        while self._done_count <= prev_done:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            elapsed = time.monotonic() - t_tp0
            if not retried and elapsed >= 1.5:
                self._teleport_pub.publish(req_msg)
                retried = True
            if elapsed >= 6.0:
                raise TimeoutError("Timed out waiting for /sim/teleport_done.")

        t_done_sim = self._last_done_stamp if self._last_done_stamp >= 0.0 else self._sim_time

        # 4. Wait for rover to land and settle (speed < 0.02 m/s, |wz| < 0.02 rad/s, max 5 s sim).
        settled = False
        stable_since: Optional[float] = None
        last_zero_pub = 0.0
        zero_twist = Twist()
        t_settle_wall0 = time.monotonic()

        while True:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            now_w = time.monotonic()
            if now_w - last_zero_pub >= 0.05:
                self._cmd_vel_nav_pub.publish(zero_twist)
                self._cmd_vel_pub.publish(zero_twist)
                last_zero_pub = now_w

            sim_since_tp = self._sim_time - t_done_sim
            if sim_since_tp >= min_drop_sim_s and self._gt is not None:
                lin = self._gt.twist.twist.linear
                ang = self._gt.twist.twist.angular
                lin_speed = math.sqrt(lin.x * lin.x + lin.y * lin.y + lin.z * lin.z)
                wz_abs = abs(ang.z)
                ang_max = max(abs(ang.x), abs(ang.y), wz_abs)
                if lin_speed < speed_tol and wz_abs < wz_tol and ang_max < 0.05:
                    if stable_since is None:
                        stable_since = self._sim_time
                    elif (self._sim_time - stable_since) >= stable_window_sim_s:
                        settled = True
                        break
                else:
                    stable_since = None

            if sim_since_tp >= max_settle_sim_s or (now_w - t_settle_wall0) >= max(12.0, max_settle_sim_s * 3.0):
                break

        settle_sim_s = self._sim_time - t_done_sim

        # 5. Read true settled GT pose and publish /set_pose to the EKF.
        gt_p = self._gt.pose.pose.position
        gt_roll, gt_pitch, gt_yaw = rpy_from_quat(self._gt.pose.pose.orientation)

        self._publish_ekf_set_pose(gt_p.x, gt_p.y, gt_yaw)
        t_ekf0 = time.monotonic()
        last_setpose_pub = t_ekf0
        ekf_pos_err = math.inf
        ekf_yaw_err = math.inf
        while (time.monotonic() - t_ekf0) < 2.5:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if self._ekf is not None and self._gt is not None:
                gp = self._gt.pose.pose.position
                _, _, gy = rpy_from_quat(self._gt.pose.pose.orientation)
                ep = self._ekf.pose.pose.position
                _, _, ey = rpy_from_quat(self._ekf.pose.pose.orientation)
                ekf_pos_err = math.hypot(ep.x - gp.x, ep.y - gp.y)
                ekf_yaw_err = abs(wrap_angle(ey - gy))
                if ekf_pos_err < 0.03 and ekf_yaw_err < math.radians(2.0):
                    break
            now_w = time.monotonic()
            if (now_w - last_setpose_pub) >= 0.3:
                gp = self._gt.pose.pose.position
                _, _, gy = rpy_from_quat(self._gt.pose.pose.orientation)
                self._publish_ekf_set_pose(gp.x, gp.y, gy)
                last_setpose_pub = now_w

        # 6. Clear hazard memory first, then clear global and local Nav2 costmaps.
        if clear_hazards:
            self._call_service(self._clear_hazards_cli, Empty.Request(), "/hazard_mapper/clear", timeout_s=5.0)

        if clear_costmaps:
            self._call_service(
                self._clear_global_cli,
                ClearEntireCostmap.Request(),
                "/global_costmap/clear_entirely_global_costmap",
                timeout_s=5.0,
            )
            self._call_service(
                self._clear_local_cli,
                ClearEntireCostmap.Request(),
                "/local_costmap/clear_entirely_local_costmap",
                timeout_s=5.0,
            )

        # Let 1-2 fresh 5 Hz hazard scans at the new pose arrive before metrics reset.
        if post_clear_sim_s > 0.0:
            self._spin_for_sim(post_clear_sim_s, max_wall_s=3.0, publish_zero=True)

        # 7. Reset metrics accumulators and drift references.
        if reset_metrics:
            self._call_service(self._reset_metrics_cli, Empty.Request(), "/metrics_node/reset", timeout_s=5.0)
            self._spin_for_sim(0.15, max_wall_s=2.0, publish_zero=True)

        # Refresh final settled pose and EKF agreement.
        gt_p = self._gt.pose.pose.position
        gt_roll, gt_pitch, gt_yaw = rpy_from_quat(self._gt.pose.pose.orientation)
        ep = self._ekf.pose.pose.position
        _, _, ey = rpy_from_quat(self._ekf.pose.pose.orientation)
        ekf_pos_err = math.hypot(ep.x - gt_p.x, ep.y - gt_p.y)
        ekf_yaw_err = abs(wrap_angle(ey - gt_yaw))

        return SettledPose(
            x=float(gt_p.x),
            y=float(gt_p.y),
            z=float(gt_p.z),
            roll=float(gt_roll),
            pitch=float(gt_pitch),
            yaw=float(gt_yaw),
            sim_time=float(self._sim_time),
            settle_sim_s=float(settle_sim_s),
            reset_wall_s=float(time.monotonic() - t_wall0),
            ekf_pos_err_m=float(ekf_pos_err),
            ekf_yaw_err_rad=float(ekf_yaw_err),
            settled=settled,
        )

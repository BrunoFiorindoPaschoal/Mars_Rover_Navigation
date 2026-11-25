#!/usr/bin/env python3
"""Sensor noise relay for Mars Yard domain randomization (Phase 7).

Only launched when rand_level > 0. At level 0 this node is not in the graph at all.

Subscribes:
  - /points (sensor_msgs/PointCloud2)
  - /imu_sensor_broadcaster/imu (sensor_msgs/Imu)

Publishes:
  - /points_noisy (sensor_msgs/PointCloud2): Gaussian range noise along each ray + random dropout
  - /imu_sensor_broadcaster/imu_noisy (sensor_msgs/Imu): Gaussian gyro noise + constant z bias
"""

from typing import Tuple

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu, PointCloud2, PointField
except ImportError:  # pragma: no cover - allows importing pure noise helpers without ROS
    rclpy = None
    Node = object
    HistoryPolicy = QoSProfile = ReliabilityPolicy = None
    Imu = PointCloud2 = PointField = None


def apply_lidar_noise(
    pts: np.ndarray,
    range_sigma_m: float,
    dropout_prob: float,
    rng: np.random.Generator,
    min_range_m: float = 0.05,
) -> np.ndarray:
    """Apply radial Gaussian range noise along each ray and random point dropout."""
    if pts.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    pts_f = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
    n = pts_f.shape[0]

    if dropout_prob > 0.0:
        keep = rng.random(n) >= dropout_prob
        pts_f = pts_f[keep]
        if pts_f.shape[0] == 0:
            return np.empty((0, 3), dtype=np.float32)

    if range_sigma_m > 0.0:
        ranges = np.linalg.norm(pts_f, axis=1)
        valid = ranges > 1e-6
        dr = rng.normal(0.0, range_sigma_m, size=pts_f.shape[0])
        new_ranges = np.maximum(min_range_m, ranges + dr)
        scale = np.ones_like(ranges)
        scale[valid] = new_ranges[valid] / ranges[valid]
        pts_f = pts_f * scale[:, None]

    return np.ascontiguousarray(pts_f, dtype=np.float32)


def apply_gyro_noise(
    wx: float,
    wy: float,
    wz: float,
    gyro_sigma_rad_s: float,
    gyro_z_bias_rad_s: float,
    rng: np.random.Generator,
) -> Tuple[float, float, float]:
    """Apply zero-mean Gaussian noise to (wx, wy, wz) plus constant z bias."""
    if gyro_sigma_rad_s > 0.0:
        n = rng.normal(0.0, gyro_sigma_rad_s, size=3)
        wx += float(n[0])
        wy += float(n[1])
        wz += float(n[2])
    wz += float(gyro_z_bias_rad_s)
    return float(wx), float(wy), float(wz)


def _parse_xyz_pointcloud2(msg: PointCloud2) -> np.ndarray:
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


def _pack_xyz_pointcloud2(pts: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    pts_f32 = np.ascontiguousarray(pts, dtype=np.float32).reshape(-1, 3)
    n = pts_f32.shape[0]
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = n
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * n
    msg.is_dense = True
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = pts_f32.tobytes()
    return msg


class NoiseRelayNode(Node):
    def __init__(self) -> None:
        super().__init__("noise_relay")

        self.declare_parameter("points_in", "/points")
        self.declare_parameter("points_out", "/points_noisy")
        self.declare_parameter("imu_in", "/imu_sensor_broadcaster/imu")
        self.declare_parameter("imu_out", "/imu_sensor_broadcaster/imu_noisy")

        self.declare_parameter("lidar_noise_sigma", 0.0)
        self.declare_parameter("lidar_dropout", 0.0)
        self.declare_parameter("imu_gyro_sigma", 0.0)
        self.declare_parameter("imu_gyro_z_bias", 0.0)
        self.declare_parameter("lidar_seed", 1)
        self.declare_parameter("imu_seed", 2)

        points_in = self.get_parameter("points_in").value
        points_out = self.get_parameter("points_out").value
        imu_in = self.get_parameter("imu_in").value
        imu_out = self.get_parameter("imu_out").value

        self._lidar_sigma = float(self.get_parameter("lidar_noise_sigma").value)
        self._lidar_dropout = float(self.get_parameter("lidar_dropout").value)
        self._imu_sigma = float(self.get_parameter("imu_gyro_sigma").value)
        self._imu_z_bias = float(self.get_parameter("imu_gyro_z_bias").value)

        lidar_seed = int(self.get_parameter("lidar_seed").value)
        imu_seed = int(self.get_parameter("imu_seed").value)
        self._lidar_rng = np.random.default_rng(lidar_seed)
        self._imu_rng = np.random.default_rng(imu_seed)

        qos_cloud = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._points_pub = self.create_publisher(PointCloud2, points_out, qos_cloud)
        self._imu_pub = self.create_publisher(Imu, imu_out, 100)

        self.create_subscription(PointCloud2, points_in, self._on_points, qos_cloud)
        self.create_subscription(Imu, imu_in, self._on_imu, 100)

        self.get_logger().info(
            f"noise_relay started: {points_in}->{points_out} "
            f"(sigma={self._lidar_sigma:.4f}m, dropout={100.0 * self._lidar_dropout:.1f}%, seed={lidar_seed}) | "
            f"{imu_in}->{imu_out} "
            f"(gyro_sigma={self._imu_sigma:.4f} rad/s, z_bias={self._imu_z_bias:+.4f} rad/s, seed={imu_seed})"
        )

    def _on_points(self, msg: PointCloud2) -> None:
        pts = _parse_xyz_pointcloud2(msg)
        noisy = apply_lidar_noise(pts, self._lidar_sigma, self._lidar_dropout, self._lidar_rng)
        self._points_pub.publish(_pack_xyz_pointcloud2(noisy, msg.header.frame_id, msg.header.stamp))

    def _on_imu(self, msg: Imu) -> None:
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        wx, wy, wz = apply_gyro_noise(
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            self._imu_sigma,
            self._imu_z_bias,
            self._imu_rng,
        )
        out.angular_velocity.x = wx
        out.angular_velocity.y = wy
        out.angular_velocity.z = wz
        self._imu_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = NoiseRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

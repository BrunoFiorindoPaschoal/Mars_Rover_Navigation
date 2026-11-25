#!/usr/bin/env python3
"""Per-scan 3D terrain step-height hazard mapper for Mars Yard rover.

Subscribes:
  - /points (sensor_msgs/PointCloud2 in lidar_link)
  - /imu_sensor_broadcaster/imu (sensor_msgs/Imu, used for gravity roll/pitch levelling)
  - tf2 (static base_link <- lidar_link, dynamic odom <- base_link from 2D EKF)

Publishes:
  - /hazards (sensor_msgs/PointCloud2 in odom: hazards at z=0.3, free at z=0.0)
  - /hazard_grid (nav_msgs/OccupancyGrid in odom: -1 unknown, 0 free, 100 hazard)
"""

from array import array
from collections import deque
import math
from typing import Deque, Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2, PointField
import tf2_ros

from mars_yard_nav.hazard_core import (
    HazardConfig,
    RollingHazardMap,
    detect_hazards_scan,
    quat_to_rpy,
    transform_points_to_odom,
)


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _parse_xyz_pointcloud2(msg: PointCloud2) -> np.ndarray:
    """Fast vectorized extraction of float32 (x, y, z) from a PointCloud2 message."""
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
    """Pack an (N, 3) float32 array into a sensor_msgs/PointCloud2 message."""
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


class HazardMapperNode(Node):
    def __init__(self) -> None:
        super().__init__("hazard_mapper")

        self.declare_parameter("points_topic", "/points")
        self.declare_parameter("imu_topic", "/imu_sensor_broadcaster/imu")
        self.declare_parameter("hazards_topic", "/hazards")
        self.declare_parameter("grid_topic", "/hazard_grid")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("cell_size", 0.1)
        self.declare_parameter("step_threshold", 0.12)
        self.declare_parameter("window_radius_m", 0.7)
        self.declare_parameter("ground_inlier_threshold", 0.06)
        self.declare_parameter("min_ground_cells", 8)
        self.declare_parameter("min_plane_eig", 0.015)
        self.declare_parameter("max_plane_slope_deg", 35.0)
        self.declare_parameter("free_margin_ratio", 0.7)
        self.declare_parameter("min_range", 0.65)
        self.declare_parameter("max_range", 7.5)
        self.declare_parameter("grid_size_m", 24.0)
        self.declare_parameter("memory_ttl", 30.0)
        self.declare_parameter("publish_radius_m", 10.0)
        self.declare_parameter("hazard_z", 0.3)
        self.declare_parameter("free_z", 0.0)

        points_topic = self.get_parameter("points_topic").value
        imu_topic = self.get_parameter("imu_topic").value
        hazards_topic = self.get_parameter("hazards_topic").value
        grid_topic = self.get_parameter("grid_topic").value
        self._odom_frame: str = self.get_parameter("odom_frame").value
        self._base_frame: str = self.get_parameter("base_frame").value

        self._cfg = HazardConfig(
            cell_size=float(self.get_parameter("cell_size").value),
            step_threshold=float(self.get_parameter("step_threshold").value),
            window_radius_m=float(self.get_parameter("window_radius_m").value),
            ground_inlier_threshold=float(self.get_parameter("ground_inlier_threshold").value),
            min_ground_cells=int(self.get_parameter("min_ground_cells").value),
            min_plane_eig=float(self.get_parameter("min_plane_eig").value),
            max_plane_slope_deg=float(self.get_parameter("max_plane_slope_deg").value),
            free_margin_ratio=float(self.get_parameter("free_margin_ratio").value),
            min_range=float(self.get_parameter("min_range").value),
            max_range=float(self.get_parameter("max_range").value),
            grid_size_m=float(self.get_parameter("grid_size_m").value),
            memory_ttl=float(self.get_parameter("memory_ttl").value),
            publish_radius_m=float(self.get_parameter("publish_radius_m").value),
            hazard_z=float(self.get_parameter("hazard_z").value),
            free_z=float(self.get_parameter("free_z").value),
        )
        self._map = RollingHazardMap(self._cfg)

        # Ring buffer of (sim_time_sec, roll, pitch) from IMU for scan-time matching.
        self._imu_buf: Deque[Tuple[float, float, float]] = deque(maxlen=400)
        self._static_lidar_tf: Optional[Tuple[str, np.ndarray, Tuple[float, float, float, float]]] = None

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._hazards_pub = self.create_publisher(PointCloud2, hazards_topic, qos_reliable)
        self._grid_pub = self.create_publisher(OccupancyGrid, grid_topic, qos_reliable)

        self.create_subscription(Imu, imu_topic, self._on_imu, 100)
        self.create_subscription(PointCloud2, points_topic, self._on_points, qos_reliable)

        self._warned_no_imu = False
        self._warned_no_tf = False
        self.get_logger().info(
            f"hazard_mapper started: points='{points_topic}', imu='{imu_topic}', "
            f"hazards='{hazards_topic}', grid='{grid_topic}', "
            f"cell={self._cfg.cell_size:.2f}m, step_thresh={self._cfg.step_threshold:.2f}m, "
            f"win_r={self._cfg.window_radius_m:.2f}m, range=[{self._cfg.min_range:.2f}, {self._cfg.max_range:.2f}]m"
        )

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        roll, pitch, _ = quat_to_rpy(q.x, q.y, q.z, q.w)
        self._imu_buf.append((_stamp_to_sec(msg.header.stamp), roll, pitch))

    def _lookup_imu_rp(self, target_sec: float) -> Optional[Tuple[float, float]]:
        if not self._imu_buf:
            return None
        # Find IMU sample closest in sim time to the lidar scan timestamp.
        best_dt = math.inf
        best_rp = (0.0, 0.0)
        for t, r, p in reversed(self._imu_buf):
            dt = abs(t - target_sec)
            if dt <= best_dt:
                best_dt = dt
                best_rp = (r, p)
            elif t < target_sec - 0.1:
                break
        return best_rp

    def _get_lidar_to_base(self, lidar_frame: str):
        if self._static_lidar_tf is not None and self._static_lidar_tf[0] == lidar_frame:
            return self._static_lidar_tf[1], self._static_lidar_tf[2]
        try:
            tf = self._tf_buffer.lookup_transform(self._base_frame, lidar_frame, Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        t_vec = np.array([t.x, t.y, t.z], dtype=np.float64)
        q_tup = (q.x, q.y, q.z, q.w)
        self._static_lidar_tf = (lidar_frame, t_vec, q_tup)
        return t_vec, q_tup

    def _get_odom_pose(self, stamp):
        scan_time = Time.from_msg(stamp)
        try:
            tf = self._tf_buffer.lookup_transform(self._odom_frame, self._base_frame, scan_time)
        except Exception:
            try:
                tf = self._tf_buffer.lookup_transform(self._odom_frame, self._base_frame, Time())
            except Exception:
                return None
        t = tf.transform.translation
        q = tf.transform.rotation
        _, _, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        return float(t.x), float(t.y), float(yaw)

    def _on_points(self, msg: PointCloud2) -> None:
        scan_sec = _stamp_to_sec(msg.header.stamp)
        rp = self._lookup_imu_rp(scan_sec)
        if rp is None:
            if not self._warned_no_imu:
                self.get_logger().warn("Waiting for IMU messages before mapping hazards...")
                self._warned_no_imu = True
            return

        lidar_tf = self._get_lidar_to_base(msg.header.frame_id)
        odom_pose = self._get_odom_pose(msg.header.stamp)
        if lidar_tf is None or odom_pose is None:
            if not self._warned_no_tf:
                self.get_logger().warn(
                    f"Waiting for TF ({self._odom_frame} -> {self._base_frame} -> {msg.header.frame_id})..."
                )
                self._warned_no_tf = True
            return

        t_bl, q_bl = lidar_tf
        odom_x, odom_y, odom_yaw = odom_pose
        roll, pitch = rp

        pts_lidar = _parse_xyz_pointcloud2(msg)
        pts_odom = transform_points_to_odom(
            pts_lidar,
            t_bl,
            q_bl,
            roll,
            pitch,
            odom_x,
            odom_y,
            odom_yaw,
            min_range=self._cfg.min_range,
            max_range=self._cfg.max_range,
        )

        hazard_ij, free_ij, _ = detect_hazards_scan(pts_odom, self._cfg)
        self._map.update(odom_x, odom_y, scan_sec, hazard_ij, free_ij)

        # Publish /hazards PointCloud2 for Nav2 obstacle layer (z=0.3 hazards, z=0.0 free).
        cloud_pts = self._map.export_cloud_points(odom_x, odom_y)
        self._hazards_pub.publish(_pack_xyz_pointcloud2(cloud_pts, self._odom_frame, msg.header.stamp))

        # Publish /hazard_grid OccupancyGrid for RViz/debug.
        ox, oy = self._map.origin_world()
        grid = OccupancyGrid()
        grid.header.stamp = msg.header.stamp
        grid.header.frame_id = self._odom_frame
        grid.info.map_load_time = msg.header.stamp
        grid.info.resolution = float(self._map.cell_size)
        grid.info.width = self._map.dim
        grid.info.height = self._map.dim
        grid.info.origin.position.x = float(ox)
        grid.info.origin.position.y = float(oy)
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = array("b", self._map.state.tobytes())
        self._grid_pub.publish(grid)


def main() -> None:
    rclpy.init()
    node = HazardMapperNode()
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

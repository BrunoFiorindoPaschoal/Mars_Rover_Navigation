"""Costmap/hazard bag (MCAP) -> npz for the README sensing figure (tools/plot_costmap_figure.py).

    source /opt/ros/jazzy/setup.bash
    python3 tools/bag_to_costmap.py results/figure/bags/<run> --out media/hero/costmap.npz

The bag must have been recorded with BAG_EXTRA_TOPICS="/global_costmap/costmap /hazards" (see run_benchmark.py).
Times are sim-time header stamps in seconds, like tools/bag_to_trajectory.py.

Output arrays:
    cm_t (C,)  cm (C,H,W) int8 0..100 / -1   cm_origin (2,)  cm_res ()   every /global_costmap/costmap
    hz_t (P,)  hz_xy (P,2)                    every /hazards point, with the stamp of its scan
    gt_t, gt_pos, plan_t, plan_len, plan_xy   as in bag_to_trajectory.py
"""
import argparse
import pathlib

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

TOPICS = ("/global_costmap/costmap", "/hazards", "/ground_truth/odom", "/plan")


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag", type=pathlib.Path)
    parser.add_argument("--out", type=pathlib.Path, required=True)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in TOPICS}
    missing = set(TOPICS) - set(types)
    if missing:
        raise SystemExit(f"{args.bag}: bag has no {sorted(missing)} (record with BAG_EXTRA_TOPICS)")

    cms, hz, gt, plans = [], [], [], []
    frames = set()
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic not in types:
            continue
        msg = deserialize_message(raw, types[topic])
        if topic == "/global_costmap/costmap":
            info = msg.info
            cms.append((stamp(msg), np.array(msg.data, np.int8).reshape(info.height, info.width),
                        (info.origin.position.x, info.origin.position.y), info.resolution))
        elif topic == "/hazards":
            frames.add(msg.header.frame_id)
            pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y"))
            hz += [(stamp(msg), x, y) for x, y in pts]
        elif topic == "/ground_truth/odom":
            p = msg.pose.pose.position
            gt.append((stamp(msg), p.x, p.y))
        elif msg.poses:
            plans.append((stamp(msg), np.array([(q.pose.position.x, q.pose.position.y) for q in msg.poses])))

    if frames - {"map", "odom"}:
        raise SystemExit(f"/hazards in frame {frames}, expected map/odom")  # plotted directly in map
    shapes = {c[1].shape for c in cms}
    assert len(shapes) == 1, f"costmap size changed during the run: {shapes}"
    hz, gt = np.array(hz), np.array(sorted(gt))
    np.savez(args.out, cm_t=np.array([c[0] for c in cms]), cm=np.stack([c[1] for c in cms]),
             cm_origin=np.array(cms[0][2]), cm_res=cms[0][3],
             hz_t=hz[:, 0], hz_xy=hz[:, 1:3], hazard_frames=np.array(sorted(frames)),
             gt_t=gt[:, 0], gt_pos=gt[:, 1:3],
             plan_t=np.array([t for t, _ in plans]), plan_len=np.array([len(p) for _, p in plans]),
             plan_xy=np.concatenate([p for _, p in plans]))
    print(f"wrote {args.out}: {len(cms)} costmaps {shapes.pop()} @ {cms[0][3]} m, origin {cms[0][2]}, "
          f"{len(hz)} hazard points in {sorted(frames)}, {len(plans)} plans, "
          f"costmap times {np.round(np.array([c[0] for c in cms]) - gt[0, 0], 1)}")


if __name__ == "__main__":
    main()

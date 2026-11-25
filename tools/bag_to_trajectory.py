"""Benchmark rosbag (MCAP) -> trajectory .npz, the input of the Phase 9 hero-video replay.

    source /opt/ros/jazzy/setup.bash
    python3 tools/bag_to_trajectory.py results/benchmark/bags/navfn_rpp__slope_04__s0 \
        --out media/hero/trajectory.npz

Runs with the system python3 (it needs rosbag2_py from ROS, not the .venv).
All times are sim-time header stamps in seconds, so the three topics line up with each other.

Output arrays:
    gt_t (N,)  gt_pos (N,3)  gt_quat (N,4, w x y z)  /ground_truth/odom, base_link in map (== MuJoCo world)
    js_t (M,)  js_names (J,) js_pos (M,J)            /joint_states
    plan_t (K,) plan_len (K,) plan_xy (sum(plan_len),2)  every /plan Nav2 published, concatenated
"""
import argparse
import pathlib

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = ("/ground_truth/odom", "/joint_states", "/plan")


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def read_bag(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in TOPICS}
    missing = set(TOPICS) - set(types)
    if missing:
        raise SystemExit(f"{uri}: bag has no {sorted(missing)}")

    gt, js, plans = [], [], []
    js_names = None
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic not in types:
            continue
        msg = deserialize_message(raw, types[topic])
        if topic == "/ground_truth/odom":
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            gt.append((stamp(msg), p.x, p.y, p.z, q.w, q.x, q.y, q.z))
        elif topic == "/joint_states":
            if js_names is None:
                js_names = list(msg.name)
            # joint order can differ between messages; store in the first message's order
            by_name = dict(zip(msg.name, msg.position))
            js.append([stamp(msg)] + [by_name[n] for n in js_names])
        elif msg.poses:
            plans.append((stamp(msg), np.array([(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses])))

    gt, js = np.array(gt), np.array(js)
    gt, js = gt[np.argsort(gt[:, 0], kind="stable")], js[np.argsort(js[:, 0], kind="stable")]
    return dict(
        gt_t=gt[:, 0], gt_pos=gt[:, 1:4], gt_quat=gt[:, 4:8],
        js_t=js[:, 0], js_names=np.array(js_names), js_pos=js[:, 1:],
        plan_t=np.array([t for t, _ in plans]),
        plan_len=np.array([len(xy) for _, xy in plans]),
        plan_xy=np.concatenate([xy for _, xy in plans]),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag", type=pathlib.Path, help="rosbag2 directory (MCAP)")
    parser.add_argument("--out", type=pathlib.Path, required=True)
    args = parser.parse_args()

    traj = read_bag(args.bag)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(args.out, bag=str(args.bag), **traj)
    dist = np.linalg.norm(np.diff(traj["gt_pos"][:, :2], axis=0), axis=1).sum()
    print(f"wrote {args.out}: {len(traj['gt_t'])} poses over {traj['gt_t'][-1] - traj['gt_t'][0]:.1f} s, "
          f"{dist:.2f} m driven, {len(traj['js_t'])} joint states, {len(traj['plan_t'])} plans")


if __name__ == "__main__":
    main()

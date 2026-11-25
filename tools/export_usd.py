"""Replay a recorded Nav2 run in MuJoCo and export it for Blender (Phase 9 hero video).

    .venv/bin/python tools/export_usd.py media/hero/trajectory.npz --out media/hero/export

No physics: each 30 fps frame sets qpos from the bag (ground-truth pose + joint angles) and calls
mj_forward, so every frame is the recorded motion. Writes into --out:

    rover.usdc    rover + rocks, animated, from MuJoCo's USDExporter. The hfield geoms are deleted
                  first: the exporter (mujoco 3.12) has no hfield branch.
    terrain.obj   the Jezero hfield and the sand hfields as meshes with UVs, so the terrain is
                  rebuilt from the same data the sim uses
    texture.png   the HiRISE orthoimage
    paths.npz     fps, frame count, the first and every later Nav2 plan (with the frame it appeared)
                  and the driven track, all draped on the terrain
"""
import argparse
import pathlib
import shutil

import mujoco
import numpy as np
from mujoco.usd import exporter as usd_exporter
from pxr import UsdGeom

ROOT = pathlib.Path(__file__).resolve().parents[1]
WORLD = ROOT / "src/mars_yard_worlds/worlds/mars.xml"
TEXTURE = ROOT / "src/mars_yard_worlds/terrain/texture.png"
FPS = 30
BASE_HEIGHT = 0.275  # base_link above flat ground at rest, rover.xml
DRAPE = 0.03         # lift path curves above the ground so they don't z-fight


def unify_hemisphere(quat):
    """Flip quaternions so neighbours have a positive dot product (q and -q are the same rotation)."""
    quat = quat.copy()
    for i in range(1, len(quat)):
        if np.dot(quat[i], quat[i - 1]) < 0:
            quat[i] = -quat[i]
    return quat


def resample(traj, times):
    """Pose and joint angles at the frame times: linear for positions/angles, nlerp for orientation."""
    pos = np.stack([np.interp(times, traj["gt_t"], traj["gt_pos"][:, k]) for k in range(3)], axis=1)
    q = unify_hemisphere(traj["gt_quat"])
    quat = np.stack([np.interp(times, traj["gt_t"], q[:, k]) for k in range(4)], axis=1)
    quat /= np.linalg.norm(quat, axis=1, keepdims=True)
    joints = {name: np.interp(times, traj["js_t"], traj["js_pos"][:, j])
              for j, name in enumerate(traj["js_names"])}
    return pos, quat, joints


def set_pose(model, data, pos, quat, joints):
    adr = model.joint("base_free_joint").qposadr[0]
    data.qpos[adr:adr + 3], data.qpos[adr + 3:adr + 7] = pos, quat
    for name, value in joints.items():
        data.qpos[model.joint(str(name)).qposadr[0]] = value


class Ground:
    """Terrain height under (x, y) by a downward ray against the visual terrain, sand and rocks."""

    def __init__(self, model):
        self.model, self.data = model, mujoco.MjData(model)
        set_pose(model, self.data, [1e3, 1e3, 0], [1, 0, 0, 0], {})  # park the rover out of the way
        mujoco.mj_forward(model, self.data)
        self.groups = np.array([1, 0, 1, 0, 0, 0], np.uint8)  # 0 = sand/rocks, 2 = visual terrain
        self.geomid = np.zeros(1, np.int32)

    def __call__(self, xy):
        z = np.empty(len(xy))
        for i, (x, y) in enumerate(xy):
            dist = mujoco.mj_ray(self.model, self.data, np.array([x, y, 50.0]), np.array([0, 0, -1.0]),
                                 self.groups, 1, -1, self.geomid)
            if dist < 0:
                raise ValueError(f"no terrain under ({x:.2f}, {y:.2f})")
            z[i] = 50.0 - dist
        return z


def hfield_obj(model, geom_names, path):
    """Write hfield geoms as one OBJ (one object each), same vertex layout MuJoCo uses, UV = 0..1 over the patch."""
    lines, offset = [], 1
    for name in geom_names:
        g = model.geom(name)
        h = g.dataid[0]
        nrow, ncol = model.hfield_nrow[h], model.hfield_ncol[h]
        sx, sy, sz, _ = model.hfield_size[h]
        adr = model.hfield_adr[h]
        height = model.hfield_data[adr:adr + nrow * ncol].reshape(nrow, ncol)
        c, r = np.meshgrid(np.arange(ncol), np.arange(nrow))
        u, v = c / (ncol - 1), r / (nrow - 1)
        verts = np.stack([-sx + 2 * sx * u, -sy + 2 * sy * v, height * sz], axis=-1).reshape(-1, 3) + g.pos
        uv = np.stack([u, v], axis=-1).reshape(-1, 2)
        idx = (r * ncol + c)[:-1, :-1].ravel() + offset
        quads = np.stack([idx, idx + 1, idx + ncol + 1, idx + ncol], axis=1)  # counter-clockwise seen from +z
        lines.append(f"o {name}")
        lines += [f"v {x:.4f} {y:.4f} {z:.4f}" for x, y, z in verts]
        lines += [f"vt {a:.5f} {b:.5f}" for a, b in uv]
        lines += ["f " + " ".join(f"{i}/{i}" for i in q) for q in quads]
        offset += len(verts)
    path.write_text("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("trajectory", type=pathlib.Path, help=".npz from tools/bag_to_trajectory.py")
    parser.add_argument("--out", type=pathlib.Path, required=True)
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)

    traj = np.load(args.trajectory)
    t0, t1 = max(traj["gt_t"][0], traj["js_t"][0]), min(traj["gt_t"][-1], traj["js_t"][-1])
    times = np.arange(t0, t1, 1 / FPS)
    pos, quat, joints = resample(traj, times)

    full = mujoco.MjModel.from_xml_path(str(WORLD))
    hfields = [full.geom(i).name for i in range(full.ngeom) if full.geom_type[i] == mujoco.mjtGeom.mjGEOM_HFIELD]
    visual_hfields = [n for n in hfields if not n.startswith("terrain_tile_")]  # tiles = invisible collision copy

    # self-check: the replayed rover sits on the terrain, so map frame == MuJoCo world and the bag lines up
    ground = Ground(full)
    clearance = pos[:, 2] - ground(pos[:, :2])
    assert np.all(np.abs(clearance - BASE_HEIGHT) < 0.15), f"rover off the ground: {clearance.min():.2f}..{clearance.max():.2f} m"

    spec = mujoco.MjSpec.from_file(str(WORLD))
    for geom in [g for g in spec.geoms if g.type == mujoco.mjtGeom.mjGEOM_HFIELD]:
        spec.delete(geom)
    model = spec.compile()
    data = mujoco.MjData(model)
    usd = usd_exporter.USDExporter(model, output_directory="usd", output_directory_root=str(out), verbose=False)
    for k in range(len(times)):
        set_pose(model, data, pos[k], quat[k], {n: v[k] for n, v in joints.items()})
        mujoco.mj_forward(model, data)
        usd.update_scene(data)
    # the exporter writes metres and one timecode per frame but labels the stage cm @ 60 fps
    UsdGeom.SetStageMetersPerUnit(usd.stage, 1.0)
    usd.stage.SetTimeCodesPerSecond(FPS)
    usd.stage.SetFramesPerSecond(FPS)
    usd.save_scene("usdc")
    frames = out / "usd/frames"
    shutil.copy(frames / f"frame_{usd.frame_count}.usdc", out / "rover.usdc")

    hfield_obj(full, visual_hfields, out / "terrain.obj")
    shutil.copy(TEXTURE, out / "texture.png")

    plan_xy = traj["plan_xy"][:traj["plan_len"][0]]
    plans_xy = traj["plan_xy"]  # every /plan Nav2 published, so the video shows the live replanning
    plans_frame = np.maximum(np.round((traj["plan_t"] - t0) * FPS).astype(int) + 1, 1)
    np.savez(out / "paths.npz", fps=FPS, n_frames=len(times), t0=t0,
             plan_xyz=np.column_stack([plan_xy, ground(plan_xy) + DRAPE]),
             plans_xyz=np.column_stack([plans_xy, ground(plans_xy) + DRAPE]),
             plans_len=traj["plan_len"], plans_frame=plans_frame,
             track_xyz=np.column_stack([pos[:, :2], ground(pos[:, :2]) + DRAPE]),
             rover_pos=pos)
    print(f"wrote {out}: {len(times)} frames ({len(times) / FPS:.1f} s), terrain {visual_hfields}, "
          f"rover clearance {clearance.min():.3f}..{clearance.max():.3f} m")


if __name__ == "__main__":
    main()

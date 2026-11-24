"""Tier-A video: the MuJoCo offscreen renderer, with a camera tracking the rover on Jezero terrain.

    MUJOCO_GL=egl .venv/bin/python tools/render_tier_a.py --out media/tier_a.mp4 [--seconds 20]
    MUJOCO_GL=egl .venv/bin/python tools/render_tier_a.py --out media/tier_a.gif --width 480 --height 270

The rover follows a fixed open-loop drive script, with no ROS involved. This is the early README
visual. The cinematic version (Phase 9) replays a real Nav2 run in Blender instead.
"""
import argparse
import pathlib

import imageio.v2 as imageio
import mujoco
import numpy as np

import drive_test as dt

ROOT = pathlib.Path(__file__).resolve().parents[1]
WORLD = ROOT / "src/mars_yard_worlds/worlds/mars.xml"
FPS = 30


def command(t):
    """Open-loop (v, w) drive script: straight, a long left arc, then straight again."""
    if t < 6:
        return 0.4, 0.0
    if t < 14:
        return 0.35, 0.25
    return 0.4, 0.0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", default=str(ROOT / "media/tier_a.mp4"))
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(WORLD))
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, args.height, args.width)

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = model.body("base_link").id
    cam.distance, cam.elevation = 4.0, -20.0

    scene_opt = mujoco.MjvOption()
    scene_opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False  # hide the 360 lidar rays

    act = {name: model.actuator(name).id for name in dt.WHEELS}
    steps_per_frame = int(round(1 / (FPS * model.opt.timestep)))
    pathlib.Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    # .gif for the README (small), anything else goes through ffmpeg (mp4)
    opts = {"duration": 1000 / FPS, "loop": 0} if args.out.endswith(".gif") else {"fps": FPS, "quality": 7}
    with imageio.get_writer(args.out, **opts) as video:
        for frame in range(int(args.seconds * FPS)):
            left, right = dt.wheel_speeds(*command(data.time))
            for name, side in dt.WHEELS.items():
                data.ctrl[act[name]] = left if side > 0 else right
            for _ in range(steps_per_frame):
                mujoco.mj_step(model, data)
            cam.azimuth = 120 + 40 * np.sin(frame / (FPS * 8))  # slow swing around the rover
            renderer.update_scene(data, cam, scene_opt)
            video.append_data(renderer.render())
    print(f"wrote {args.out}: {args.seconds:.0f} s, rover ended at x={data.qpos[0]:.1f} y={data.qpos[1]:.1f} m")


if __name__ == "__main__":
    main()

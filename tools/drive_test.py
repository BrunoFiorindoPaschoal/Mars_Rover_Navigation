"""Drive the rover in pure MuJoCo (no ROS) with a scripted command sequence.

    .venv/bin/python tools/drive_test.py              # opens the viewer
    .venv/bin/python tools/drive_test.py --headless   # prints metrics, exits non-zero on failure

This is the Phase 1 check that the model drives straight, turns and does not jitter,
before ros2_control is added on top.
"""
import argparse
import pathlib
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
WORLD = ROOT / "src/mars_yard_worlds/worlds/flat.xml"

WHEEL_RADIUS = 0.12  # m, same as rover.xml
TRACK = 0.64         # m, distance between left and right wheel centres
WHEELS = {"wheel_fl_joint": +1, "wheel_rl_joint": +1, "wheel_fr_joint": -1, "wheel_rr_joint": -1}  # +1 = left side

# (duration s, linear m/s, angular rad/s)
SCRIPT = [(1.0, 0.0, 0.0), (4.0, 0.4, 0.0), (4.0, 0.0, 0.5), (4.0, 0.3, 0.3)]


def wheel_speeds(v, w):
    """Differential-drive inverse kinematics: body twist (v, w) -> (left, right) wheel rad/s.

    The same formula diff_drive_controller uses. Skid-steer wheels also slide sideways
    in turns, so the real yaw rate comes out lower than w; we measure that below.
    """
    left = (v - w * TRACK / 2) / WHEEL_RADIUS
    right = (v + w * TRACK / 2) / WHEEL_RADIUS
    return left, right


def yaw(quat):
    """Yaw angle from a MuJoCo quaternion (w, x, y, z)."""
    w, x, y, z = quat
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def run(model, data, viewer=None):
    """Play SCRIPT, return per-segment measured (distance, yaw change, z-velocity std)."""
    act = {name: model.actuator(name).id for name in WHEELS}
    results = []
    for duration, v, w in SCRIPT:
        left, right = wheel_speeds(v, w)
        for name, side in WHEELS.items():
            data.ctrl[act[name]] = left if side > 0 else right
        start_xy, start_yaw = data.qpos[:2].copy(), yaw(data.qpos[3:7])
        vz = []
        for _ in range(int(duration / model.opt.timestep)):
            mujoco.mj_step(model, data)
            vz.append(data.qvel[2])
            if viewer is not None:
                viewer.sync()
                time.sleep(model.opt.timestep)
        dyaw = np.angle(np.exp(1j * (yaw(data.qpos[3:7]) - start_yaw)))
        results.append((np.linalg.norm(data.qpos[:2] - start_xy), dyaw, float(np.std(vz))))
    return results


def check(results):
    """Assert the model behaves like a rover; return a list of failure messages."""
    errors = []
    names = ["settle", "straight", "spin", "arc"]
    for (dist, dyaw, vz_std), (duration, v, w), name in zip(results, SCRIPT, names):
        print(f"{name:9s} commanded {v*duration:5.2f} m {w*duration:5.2f} rad | "
              f"true {dist:5.2f} m {dyaw:5.2f} rad | z-vel std {vz_std*1000:6.2f} mm/s")
        if name != "settle" and vz_std > 0.02:  # settle includes the spawn drop
            errors.append(f"{name}: vertical jitter {vz_std:.3f} m/s")
    straight, spin = results[1], results[2]
    if not 0.85 * 1.6 < straight[0] < 1.05 * 1.6:
        errors.append(f"straight: drove {straight[0]:.2f} m, expected about 1.6 m")
    if abs(straight[1]) > 0.05:
        errors.append(f"straight: drifted {straight[1]:.3f} rad in yaw")
    # Skid-steer wheels scrub sideways in a turn, so the rover yaws slower than the wheel
    # speeds imply. This ratio sets wheel_separation_multiplier in the diff_drive_controller config.
    efficiency = spin[1] / (SCRIPT[2][2] * SCRIPT[2][0])
    print(f"turn efficiency (true yaw / commanded yaw, spin in place): {efficiency:.2f}")
    if efficiency < 0.25:
        errors.append(f"spin: turn efficiency {efficiency:.2f} too low to steer")
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(WORLD))
    data = mujoco.MjData(model)
    if args.headless:
        results = run(model, data)
    else:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            results = run(model, data, viewer)
    errors = check(results)
    for e in errors:
        print("FAIL:", e)
    sys.exit(1 if errors else 0)


if __name__ == "__main__":
    main()

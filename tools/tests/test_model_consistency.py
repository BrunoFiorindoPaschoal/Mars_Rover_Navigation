"""The URDF (ROS frames) and the MJCF (physics) describe the same rover. If they drift apart,
tf shows the wheels in one place while MuJoCo simulates them somewhere else, and odometry is
computed with the wrong wheel radius. This test catches that."""
import pathlib
import subprocess
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
URDF = ROOT / "src/mars_yard_description/urdf/rover.urdf.xacro"
WORLD = ROOT / "src/mars_yard_worlds/worlds/flat.xml"
FRAMES = ["wheel_fl_link", "wheel_fr_link", "wheel_rl_link", "wheel_rr_link", "lidar_link", "front_axle_link"]


@pytest.fixture(scope="module")
def urdf():
    xml = subprocess.run(["xacro", str(URDF)], check=True, capture_output=True, text=True).stdout
    return ET.fromstring(xml)


@pytest.fixture(scope="module")
def mjcf():
    model = mujoco.MjModel.from_xml_path(str(WORLD))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def urdf_pose_in_base(root, link):
    """Position of `link` in base_link, by chaining joint origins (all joints here have zero rotation)."""
    joints = {j.find("child").get("link"): j for j in root.iter("joint")}
    pos = np.zeros(3)
    while link != "base_link":
        joint = joints[link]
        pos += np.array([float(v) for v in joint.find("origin").get("xyz").split()])
        link = joint.find("parent").get("link")
    return pos


@pytest.mark.parametrize("frame", FRAMES)
def test_frame_positions_match(urdf, mjcf, frame):
    model, data = mjcf
    base = model.body("base_link").id
    mj_pos = data.xmat[base].reshape(3, 3).T @ (data.xpos[model.body(frame).id] - data.xpos[base])
    np.testing.assert_allclose(urdf_pose_in_base(urdf, frame), mj_pos, atol=1e-6)


def test_wheel_radius_and_mass(urdf, mjcf):
    model, _ = mjcf
    cyl = urdf.find("link[@name='wheel_fl_link']/collision/geometry/cylinder")
    assert float(cyl.get("radius")) == pytest.approx(model.geom("wheel_fl").size[0])
    assert float(cyl.get("length")) == pytest.approx(2 * model.geom("wheel_fl").size[1])
    urdf_mass = sum(float(m.get("value")) for m in urdf.iter("mass"))
    mj_mass = model.body_subtreemass[model.body("base_link").id]
    assert urdf_mass == pytest.approx(mj_mass)


def test_joint_names_have_actuators(urdf, mjcf):
    """mujoco_ros2_control finds each wheel's actuator by the joint name."""
    model, _ = mjcf
    for j in urdf.iter("joint"):
        if j.get("type") == "continuous":
            assert model.actuator(j.get("name")).trnid[0] == model.joint(j.get("name")).id

"""Configured simulation, TCP IK and ROS descriptions use one physical model."""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pybullet as p
import pybullet_data
import pytest

from lbr_pybullet_tutorial import model
from lbr_pybullet_tutorial.project_config import (
    ProjectConfig, TCP_LINK, inspect_robot, save_project,
)


def activate(config, tmp_path, monkeypatch):
    path = save_project(config, tmp_path / "active_project.json")
    monkeypatch.setenv("LBR_PYBULLET_PROJECT", path)
    return inspect_robot(config)


def joint_indices(body, client):
    return {p.getJointInfo(body, index, physicsClientId=client)[1].decode(): index
            for index in range(p.getNumJoints(body, physicsClientId=client))}


def link_frames(body, client):
    result = {}
    for index in range(p.getNumJoints(body, physicsClientId=client)):
        name = p.getJointInfo(body, index, physicsClientId=client)[12].decode()
        state = p.getLinkState(body, index, computeForwardKinematics=True, physicsClientId=client)
        result[name] = (state[4], state[5])
    return result


def assert_pose(actual, expected, position_tolerance=2e-6, rotation_tolerance=2e-6):
    assert np.linalg.norm(np.asarray(actual[0]) - expected[0]) < position_tolerance
    assert np.linalg.norm(model.orientation_error(expected[1], actual[1])) < rotation_tolerance


def configured_probe():
    return ProjectConfig(
        tool="probe", mount_xyz=[0.012, -0.008, 0.025],
        mount_rpy=[0.22, -0.14, 0.31], tcp_xyz=[0.004, -0.003, 0.179],
        tcp_rpy=[-0.13, 0.18, 0.09],
    )


def imported_iiwa(tmp_path, add_existing_base_frame=False):
    """Same actual iiwa, arbitrary native names/XML order and tighter limits."""
    source = Path(pybullet_data.getDataPath()) / "kuka_iiwa" / "model.urdf"
    root = ET.parse(source).getroot()
    for mesh in root.findall(".//mesh"):
        mesh.set("filename", str((source.parent / mesh.get("filename")).resolve()))
    for index, joint in enumerate(root.findall("joint")):
        joint.set("name", f"native_axis_{(index * 3) % 7}")
        limits = joint.find("limit")
        limits.set("lower", str(float(limits.get("lower")) + 0.01 * (index + 1)))
        limits.set("upper", str(float(limits.get("upper")) - 0.02 * (index + 1)))
    joints = root.findall("joint")
    for joint in joints:
        root.remove(joint)
    root.extend(reversed(joints))
    if add_existing_base_frame:
        for element in root.iter():
            for attribute in ("name", "link"):
                if element.get(attribute) == "lbr_iiwa_link_0":
                    element.set(attribute, "lbr_link_0")
        ET.SubElement(root, "link", {"name": "native_world_base"})
        adapter = ET.SubElement(root, "joint", {"name": "native_base_adapter", "type": "fixed"})
        ET.SubElement(adapter, "parent", {"link": "native_world_base"})
        ET.SubElement(adapter, "child", {"link": "lbr_link_0"})
    target = tmp_path / "imported_native.urdf"
    target.write_text(ET.tostring(root, encoding="unicode"))
    return target


def test_builtin_probe_tcp_fk_composes_flange_mount_and_tcp(tmp_path, monkeypatch):
    config = configured_probe()
    activate(config, tmp_path, monkeypatch)
    client = p.connect(p.DIRECT)
    try:
        source = Path(pybullet_data.getDataPath()) / "kuka_iiwa" / "model.urdf"
        body = p.loadURDF(str(source), useFixedBase=True, physicsClientId=client)
        names = joint_indices(body, client)
        for index, value in enumerate(model.INITIAL_Q, 1):
            p.resetJointState(body, names[f"lbr_iiwa_joint_{index}"], value, physicsClientId=client)
        flange = link_frames(body, client)["lbr_iiwa_link_7"]
        mount = p.multiplyTransforms(*flange, config.mount_xyz, p.getQuaternionFromEuler(config.mount_rpy))
        expected = p.multiplyTransforms(*mount, config.tcp_xyz, p.getQuaternionFromEuler(config.tcp_rpy))
        with model.Kinematics() as kin:
            actual = kin.fk(model.initial_positions())
            assert_pose(actual, expected)
            assert p.getJointInfo(kin.robot, kin.ee_link, physicsClientId=kin.client)[12].decode() == TCP_LINK
            assert np.linalg.norm(np.asarray(actual[0]) - flange[0]) > 0.17
    finally:
        p.disconnect(physicsClientId=client)


def test_builtin_probe_ik_reaches_actual_tcp_with_rotated_mount(tmp_path, monkeypatch):
    activate(configured_probe(), tmp_path, monkeypatch)
    seed = model.initial_positions()
    requested = (np.asarray(seed) + [0.07, -0.06, 0.05, 0.03, -0.04, 0.02, 0.05]).tolist()
    with model.Kinematics() as kin:
        target = kin.fk(requested)
        result = kin.ik(*target, seed, position_tolerance=0.0005, orientation_tolerance=0.002)
        actual = kin.fk(result)
        assert_pose(actual, target, position_tolerance=0.0005, rotation_tolerance=0.002)
        assert model.validate_joints(result) == result


def test_imported_joint_order_limits_fk_and_ik_follow_native_names(tmp_path, monkeypatch):
    path = imported_iiwa(tmp_path)
    config = configured_probe()
    config.robot_urdf = str(path)
    config.initial_positions = list(reversed(model.INITIAL_Q))
    meta = activate(config, tmp_path, monkeypatch)
    assert meta.control_compatible
    assert meta.joint_names == [f"native_axis_{(index * 3) % 7}" for index in reversed(range(7))]
    assert model.model_limits() == (meta.lower_limits, meta.upper_limits)
    assert meta.lower_limits[0] == pytest.approx(model.LOWER_LIMITS[6] + 0.07)
    assert meta.upper_limits[-1] == pytest.approx(model.UPPER_LIMITS[0] - 0.02)
    assert all(Path(mesh.get("filename")).is_file()
               for mesh in ET.parse(model.model_path()).getroot().findall(".//mesh"))
    with model.Kinematics() as kin:
        assert kin.dof_order == list(reversed(range(7)))
        expected_bullet = list(reversed(config.initial_positions))
        assert kin._bullet_order(config.initial_positions) == expected_bullet
        pose = kin.fk(config.initial_positions)
        for name, expected in zip(meta.joint_names, config.initial_positions):
            bullet_index = joint_indices(kin.robot, kin.client)[name]
            assert p.getJointState(kin.robot, bullet_index, physicsClientId=kin.client)[0] == pytest.approx(expected)
        goal = (np.asarray(config.initial_positions) + [0.04, -0.02, 0.03, 0.05, -0.03, 0.02, 0.06]).tolist()
        target = kin.fk(goal)
        assert np.linalg.norm(np.asarray(target[0]) - pose[0]) > 0.001
        actual_q = kin.ik(*target, config.initial_positions, position_tolerance=0.0005, orientation_tolerance=0.002)
        assert_pose(kin.fk(actual_q), target, position_tolerance=0.0005, rotation_tolerance=0.002)


@pytest.mark.parametrize("builtin", (True, False))
def test_ros_description_and_native_simulation_have_same_link_geometry(tmp_path, monkeypatch, builtin):
    config = configured_probe()
    if not builtin:
        config.robot_urdf = str(imported_iiwa(tmp_path))
        config.initial_positions = list(reversed(model.INITIAL_Q))
    meta = activate(config, tmp_path, monkeypatch)
    description = model.robot_description()
    xml = ET.fromstring(description)
    links = [link.get("name") for link in xml.findall("link")]
    assert len(links) == len(set(links))
    assert links.count("world") == 1
    assert links.count(model.BASE_FRAME) == 1
    file = tmp_path / "ros_description.urdf"
    file.write_text(description)
    client = p.connect(p.DIRECT)
    try:
        ros_body = p.loadURDF(str(file), useFixedBase=True, physicsClientId=client)
        names = joint_indices(ros_body, client)
        positions = meta.default_q
        for name, value in zip(model.JOINT_NAMES, positions):
            p.resetJointState(ros_body, names[name], value, physicsClientId=client)
        ros_frames = link_frames(ros_body, client)
        with model.Kinematics() as kin:
            kin.fk(positions)
            native_frames = link_frames(kin.robot, kin.client)
            for name, pose in native_frames.items():
                canonical = name.replace("lbr_iiwa_link_", "lbr_link_") if builtin else name
                assert_pose(ros_frames[canonical], pose)
            assert_pose(ros_frames[TCP_LINK], kin.fk(positions))
        for canonical, lo, hi in zip(model.JOINT_NAMES, meta.lower_limits, meta.upper_limits):
            control_joint = xml.find(f"ros2_control/joint[@name='{canonical}']")
            assert float(control_joint.find("command_interface/param[@name='min']").text) == lo
            assert float(control_joint.find("command_interface/param[@name='max']").text) == hi
    finally:
        p.disconnect(physicsClientId=client)


def test_ambiguous_existing_native_base_frame_remains_preview_only(tmp_path, monkeypatch):
    config = ProjectConfig(robot_urdf=str(imported_iiwa(tmp_path, add_existing_base_frame=True)))
    meta = activate(config, tmp_path, monkeypatch)
    assert not meta.control_compatible
    assert "ambiguous reserved base" in meta.control_reason
    # The native simulation still opens, without changing imported frame names.
    imported = ET.parse(model.model_path()).getroot()
    assert imported.find("link[@name='lbr_link_0']") is not None
    with pytest.raises(ValueError, match="reserved.*base|ambiguous.*base"):
        model.robot_description()

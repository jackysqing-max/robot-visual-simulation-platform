"""Model imports preserve robot geometry and attach real rigid tools safely."""

from dataclasses import asdict
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pybullet as p
import pytest

from lbr_pybullet_tutorial.project_config import (
    ProjectConfig, TCP_LINK, active_project, build_model, inspect_robot,
    load_project, save_project, validate_project,
)


def robot_file(tmp_path, axes=7, fixed_adapter=True, joint_type="revolute"):
    root = ET.Element("robot", {"name": "ImportedRobot"})
    ET.SubElement(root, "link", {"name": "base"})
    parent = "base"
    for index in range(axes):
        child = f"native_link_{index}"
        ET.SubElement(root, "link", {"name": child})
        joint = ET.SubElement(root, "joint", {"name": f"native_axis_{index}", "type": joint_type})
        ET.SubElement(joint, "parent", {"link": parent})
        ET.SubElement(joint, "child", {"link": child})
        ET.SubElement(joint, "origin", {"xyz": "0 0 0.1", "rpy": "0 0 0"})
        ET.SubElement(joint, "axis", {"xyz": "0 0 1"})
        ET.SubElement(joint, "limit", {"lower": "-1.2", "upper": "1.4", "effort": "12", "velocity": "0.8"})
        parent = child
    if fixed_adapter:
        ET.SubElement(root, "link", {"name": "native_adapter"})
        joint = ET.SubElement(root, "joint", {"name": "native_adapter_mount", "type": "fixed"})
        ET.SubElement(joint, "parent", {"link": parent})
        ET.SubElement(joint, "child", {"link": "native_adapter"})
        ET.SubElement(joint, "origin", {"xyz": "0 0 0.03", "rpy": "0 0 0"})
    path = tmp_path / "robot.urdf"
    path.write_text(ET.tostring(root, encoding="unicode"))
    return path


def test_builtin_model_preserves_names_assets_and_initial_positions(tmp_path):
    meta = inspect_robot(ProjectConfig())
    assert meta.control_compatible
    assert meta.joint_names == [f"lbr_iiwa_joint_{i}" for i in range(1, 8)]
    assert meta.default_q == [0.0, 0.35, 0.0, -0.9, 0.0, 0.65, 0.0]
    model = Path(build_model(ProjectConfig(), tmp_path))
    root = ET.parse(model).getroot()
    assert root.find("link[@name='lbr_iiwa_link_7']") is not None
    assert root.find(f"link[@name='{TCP_LINK}']") is not None
    assert all(Path(mesh.get("filename")).is_file() for mesh in root.findall(".//mesh"))
    assert "Redistribution" in model.read_text()


def test_imported_geometry_and_limits_are_not_rewritten(tmp_path):
    path = robot_file(tmp_path)
    before = path.read_bytes()
    config = ProjectConfig(robot_urdf=str(path))
    meta = inspect_robot(config)
    assert meta.control_compatible
    assert meta.flange_link == "native_adapter"
    assert meta.default_q == [0.0] * 7
    assert meta.lower_limits == [-1.2] * 7
    assert meta.effort_limits == [12.0] * 7
    assert meta.velocity_limits == [0.8] * 7
    assembled = ET.parse(build_model(config, tmp_path / "cache")).getroot()
    original = ET.parse(path).getroot()
    for joint in original.findall("joint"):
        assert ET.tostring(assembled.find(f"joint[@name='{joint.get('name')}']")) == ET.tostring(joint)
    assert path.read_bytes() == before


@pytest.mark.parametrize("axes", (0, 1, 6, 8))
def test_other_axis_counts_remain_available_for_preview(tmp_path, axes):
    meta = inspect_robot(ProjectConfig(robot_urdf=str(robot_file(tmp_path, axes))))
    assert len(meta.joint_names) == axes
    assert not meta.control_compatible


def test_flange_before_final_axis_is_preview_only(tmp_path):
    config = ProjectConfig(robot_urdf=str(robot_file(tmp_path)), flange_link="native_link_3")
    meta = inspect_robot(config)
    assert not meta.control_compatible
    assert "flange" in meta.control_reason
    config.flange_link = "missing"
    with pytest.raises(ValueError, match="Flange link does not exist"):
        inspect_robot(config)


def test_continuous_joints_cannot_use_bounded_lbr_controller(tmp_path):
    meta = inspect_robot(ProjectConfig(robot_urdf=str(robot_file(tmp_path, joint_type="continuous"))))
    assert not meta.control_compatible
    assert "bounded revolute" in meta.control_reason


def test_serial_model_flange_is_independent_of_xml_joint_order(tmp_path):
    source = robot_file(tmp_path)
    root = ET.parse(source).getroot()
    joints = root.findall("joint")
    for joint in joints:
        root.remove(joint)
    root.extend(reversed(joints))
    source.write_text(ET.tostring(root, encoding="unicode"))
    meta = inspect_robot(ProjectConfig(robot_urdf=str(source)))
    assert meta.control_compatible
    assert meta.flange_link == "native_adapter"


@pytest.mark.parametrize("reserved", ("world", "lbr_link_0"))
def test_reserved_descendant_base_frame_requires_preview_mode(tmp_path, reserved):
    source = robot_file(tmp_path)
    root = ET.parse(source).getroot()
    for element in root.iter():
        for attribute in ("name", "link"):
            if element.get(attribute) == "native_adapter":
                element.set(attribute, reserved)
    source.write_text(ET.tostring(root, encoding="unicode"))
    config = ProjectConfig(robot_urdf=str(source))
    meta = inspect_robot(config)
    assert not meta.control_compatible
    assert "ambiguous reserved base" in meta.control_reason
    # Preview can retain every original frame, including reserved descendants.
    assembled = ET.parse(build_model(config, tmp_path / "cache")).getroot()
    assert assembled.find(f"link[@name='{reserved}']") is not None


def test_mount_and_tcp_transform_match_actual_bullet_link_frame(tmp_path):
    config = ProjectConfig(
        robot_urdf=str(robot_file(tmp_path)), tool="probe",
        mount_xyz=[0.01, 0.02, 0.03], mount_rpy=[0.2, -0.1, 0.4],
        tcp_xyz=[0, 0, 0.179], tcp_rpy=[0.3, 0.0, -0.1],
    )
    model = build_model(config, tmp_path / "cache")
    client = p.connect(p.DIRECT)
    try:
        body = p.loadURDF(model, useFixedBase=True, physicsClientId=client)
        names = {p.getJointInfo(body, i, physicsClientId=client)[12].decode(): i
                 for i in range(p.getNumJoints(body, physicsClientId=client))}
        flange = p.getLinkState(body, names["native_adapter"], computeForwardKinematics=True, physicsClientId=client)
        tcp = p.getLinkState(body, names[TCP_LINK], computeForwardKinematics=True, physicsClientId=client)
        mount = p.multiplyTransforms(flange[4], flange[5], config.mount_xyz, p.getQuaternionFromEuler(config.mount_rpy))
        expected = p.multiplyTransforms(*mount, config.tcp_xyz, p.getQuaternionFromEuler(config.tcp_rpy))
        assert np.allclose(tcp[4], expected[0], atol=1e-6)
        assert abs(float(np.dot(tcp[5], expected[1]))) == pytest.approx(1.0, abs=1e-6)
        tool = names["workbench_tool_body"]
        assert p.getDynamicsInfo(body, tool, physicsClientId=client)[0] > 0
        assert len(p.getCollisionShapeData(body, tool, physicsClientId=client)) == 3
    finally:
        p.disconnect(physicsClientId=client)


def test_rigid_gripper_has_collision_geometry_and_no_movable_tool_axes(tmp_path):
    model = ET.parse(build_model(ProjectConfig(tool="gripper"), tmp_path)).getroot()
    body = model.find("link[@name='workbench_tool_body']")
    assert len(body.findall("collision")) == 3
    assert len(body.findall("visual")) == 3
    assert all(joint.get("type") == "fixed" for joint in model.findall("joint") if joint.get("name").startswith("workbench_"))


def test_mesh_instrument_scale_visual_and_collision_match(tmp_path):
    mesh = tmp_path / "tool.obj"
    mesh.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
    model = ET.parse(build_model(ProjectConfig(tool=str(mesh), mesh_scale=[0.001] * 3), tmp_path / "cache")).getroot()
    tool_meshes = model.findall("link[@name='workbench_tool_body']/.//mesh")
    assert len(tool_meshes) == 2
    assert {item.get("filename") for item in tool_meshes} == {str(mesh)}
    assert {item.get("scale") for item in tool_meshes} == {"0.001 0.001 0.001"}


def test_tool_urdf_names_are_prefixed_and_actuated_tools_are_rejected(tmp_path):
    tool = robot_file(tmp_path, axes=1)
    config = ProjectConfig(tool=str(tool))
    with pytest.raises(ValueError, match="every instrument joint must be fixed"):
        build_model(config, tmp_path / "cache")
    root = ET.parse(tool).getroot()
    for joint in root.findall("joint"):
        joint.set("type", "fixed")
    tool.write_text(ET.tostring(root, encoding="unicode"))
    assembled = ET.parse(build_model(config, tmp_path / "cache")).getroot()
    assert assembled.find("link[@name='workbench_tool_native_adapter']") is not None
    assert assembled.find("joint[@name='workbench_tool_native_axis_0']/parent").get("link") == "workbench_tool_base"


def test_relative_profile_paths_and_active_profile_are_resolved(tmp_path, monkeypatch):
    robot_file(tmp_path)
    profile = tmp_path / "project.json"
    profile.write_text(json.dumps({"robot_urdf": "robot.urdf", "tool": "probe"}))
    config = load_project(profile)
    assert config.robot_urdf == str(tmp_path / "robot.urdf")
    monkeypatch.setenv("LBR_PYBULLET_PROJECT", str(profile))
    assert active_project() == config
    assert load_project(save_project(config, tmp_path / "nested" / "saved.json")) == config
    assert "workbench_robot.urdf" not in (tmp_path / "nested" / "saved.json").read_text()


@pytest.mark.parametrize("field,value", (("mesh_scale", [0, 1, 1]), ("mount_xyz", [0, 0]), ("tcp_rpy", [0, float("nan"), 0])))
def test_invalid_offsets_and_mesh_scale_are_rejected(field, value):
    config = ProjectConfig()
    setattr(config, field, value)
    with pytest.raises(ValueError):
        validate_project(config)


def test_missing_mesh_and_invalid_limits_fail_before_loading_bullet(tmp_path):
    robot = robot_file(tmp_path)
    root = ET.parse(robot).getroot()
    link = root.find("link")
    geometry = ET.SubElement(ET.SubElement(link, "visual"), "geometry")
    ET.SubElement(geometry, "mesh", {"filename": "missing.stl"})
    robot.write_text(ET.tostring(root, encoding="unicode"))
    config = ProjectConfig(robot_urdf=str(robot))
    with pytest.raises(ValueError, match="Asset does not exist"):
        inspect_robot(config)
    link.remove(link.find("visual"))
    root.find("joint/limit").set("lower", "nan")
    robot.write_text(ET.tostring(root, encoding="unicode"))
    with pytest.raises(ValueError, match="finite"):
        inspect_robot(config)


def test_initial_positions_validate_count_limits_and_copy(tmp_path):
    config = ProjectConfig(robot_urdf=str(robot_file(tmp_path)), initial_positions=[0.0] * 7)
    before = asdict(config)
    inspect_robot(config)
    assert asdict(config) == before
    config.initial_positions = [0.0] * 6
    with pytest.raises(ValueError, match="7 finite numbers"):
        inspect_robot(config)
    config.initial_positions = [2.0] * 7
    with pytest.raises(ValueError, match="must be within"):
        inspect_robot(config)


def test_package_assets_resolve_from_ament_prefix(tmp_path, monkeypatch):
    prefix = tmp_path / "install"
    mesh = prefix / "share" / "surgical_tools" / "meshes" / "probe.obj"
    mesh.parent.mkdir(parents=True)
    mesh.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
    robot = robot_file(tmp_path)
    root = ET.parse(robot).getroot()
    geometry = ET.SubElement(ET.SubElement(root.find("link"), "visual"), "geometry")
    ET.SubElement(geometry, "mesh", {"filename": "package://surgical_tools/meshes/probe.obj"})
    robot.write_text(ET.tostring(root, encoding="unicode"))
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(prefix))
    assembled = ET.parse(build_model(ProjectConfig(robot_urdf=str(robot)), tmp_path / "cache")).getroot()
    assert assembled.find(".//mesh").get("filename") == str(mesh)

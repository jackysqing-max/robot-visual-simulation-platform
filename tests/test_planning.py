"""Motion invariants against real Bullet FK, without ROS or a running robot."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pybullet as p
import pytest

from lbr_pybullet_tutorial.model import (
    INITIAL_Q, JOINT_NAMES, LOWER_LIMITS, UPPER_LIMITS, Kinematics,
    model_path, orientation_error, robot_description, validate_joints,
)
from lbr_pybullet_tutorial.planning import cartesian_plan, joint_plan


@pytest.fixture
def kin():
    with Kinematics() as instance:
        yield instance


@pytest.mark.parametrize("joint", range(7))
@pytest.mark.parametrize("side", ("lower", "upper"))
def test_joint_limit_boundaries_and_rejection(joint, side):
    limits = LOWER_LIMITS if side == "lower" else UPPER_LIMITS
    command = INITIAL_Q.copy()
    command[joint] = limits[joint]
    assert validate_joints(command)[joint] == limits[joint]
    command[joint] += -1e-6 if side == "lower" else 1e-6
    with pytest.raises(ValueError, match=JOINT_NAMES[joint]):
        joint_plan(INITIAL_Q, command)


@pytest.mark.parametrize("invalid", (float("nan"), float("inf"), -float("inf")))
def test_nonfinite_joint_commands_never_form_a_trajectory(invalid):
    command = INITIAL_Q.copy()
    command[3] = invalid
    with pytest.raises(ValueError, match="finite"):
        joint_plan(INITIAL_Q, command)


@pytest.mark.parametrize("command", ([], [0.0] * 6, [0.0] * 8))
def test_joint_commands_require_exactly_seven_axes(command):
    with pytest.raises(ValueError):
        joint_plan(INITIAL_Q, command)


@pytest.mark.parametrize("requested_duration", (0.01, 1.0, 20.0))
def test_endpoint_zero_velocity_cubic_respects_joint_speed(requested_duration):
    goal = [0.8, -0.2, -0.4, -0.6, 0.3, 0.1, -0.5]
    speed = 0.20
    plan = joint_plan(INITIAL_Q, goal, duration=requested_duration, joint_speed=speed)
    assert plan.zero_endpoint_velocity
    assert plan.times[0] == 0
    assert plan.times[-1] >= requested_duration
    assert plan.positions[0] == INITIAL_Q
    assert plan.positions[-1] == goal
    # Evaluate the controller's endpoint-zero-velocity Hermite cubic, then
    # differentiate numerically. A secant-only test misses its faster midpoint.
    u = np.linspace(0, 1, 2001)
    shape = 3 * u**2 - 2 * u**3
    curve = np.asarray(INITIAL_Q) + shape[:, None] * (np.asarray(goal) - INITIAL_Q)
    velocity = np.gradient(curve, plan.times[-1] / (len(u) - 1), axis=0)
    assert np.max(np.abs(velocity)) <= speed + 1e-6
    assert np.max(np.abs(velocity[[0, -1]])) < 0.001


@pytest.mark.parametrize("parameter", ("duration", "joint_speed"))
@pytest.mark.parametrize("invalid", (float("nan"), float("inf"), 0.0, -1.0))
def test_invalid_joint_timing_inputs_are_rejected(parameter, invalid):
    with pytest.raises(ValueError):
        joint_plan(INITIAL_Q, INITIAL_Q, **{parameter: invalid})


@pytest.mark.parametrize("parameter", ("linear_speed", "angular_speed", "joint_speed"))
@pytest.mark.parametrize("invalid", (float("nan"), float("inf"), 0.0, -1.0))
def test_invalid_cartesian_speed_inputs_are_rejected(kin, parameter, invalid):
    pose = kin.fk(INITIAL_Q)
    with pytest.raises(ValueError, match="positive and finite"):
        cartesian_plan(kin, INITIAL_Q, [pose], **{parameter: invalid})


def test_joint_retiming_cannot_create_an_excessively_long_trajectory():
    goal = INITIAL_Q.copy()
    goal[0] += 0.5
    with pytest.raises(ValueError, match="120 seconds"):
        joint_plan(INITIAL_Q, goal, joint_speed=0.001)


def distance_to_segment(point, begin, end):
    point, begin, end = map(np.asarray, (point, begin, end))
    vector = end - begin
    fraction = np.clip(np.dot(point - begin, vector) / np.dot(vector, vector), 0, 1)
    return np.linalg.norm(point - begin - fraction * vector)


def test_cartesian_path_has_correct_geometry_continuity_and_bounded_speed(kin):
    home, quaternion = kin.fk(INITIAL_Q)
    corner = [home[0] - 0.03, home[1] + 0.02, home[2]]
    finish = [corner[0], home[1] - 0.025, home[2]]
    plan = cartesian_plan(kin, INITIAL_Q, [(corner, quaternion), (finish, quaternion)])
    assert len(plan.positions) > 10
    assert len(plan.positions) == len(plan.times)
    assert plan.times[0] == 0 and 0 < plan.times[-1] <= 120
    dt = np.diff(plan.times)
    assert np.all(dt > 0)
    joints = np.asarray(plan.positions)
    assert np.all(joints >= LOWER_LIMITS) and np.all(joints <= UPPER_LIMITS)
    assert np.max(np.abs(np.diff(joints, axis=0))) < 0.25
    assert np.max(np.abs(np.diff(joints, axis=0)) / dt[:, None]) <= 0.35
    measured = []
    for q in plan.positions:
        position, orientation = kin.fk(q)
        measured.append(position)
        assert min(distance_to_segment(position, home, corner),
                   distance_to_segment(position, corner, finish)) < 0.001
        assert np.linalg.norm(orientation_error(quaternion, orientation)) < 0.005
    assert min(math.dist(point, corner) for point in measured) < 0.001
    assert math.dist(measured[-1], finish) < 0.001
    assert np.max(np.linalg.norm(np.diff(measured, axis=0), axis=1) / dt) < 0.044


def test_unreachable_cartesian_target_is_rejected(kin):
    _, quaternion = kin.fk(INITIAL_Q)
    with pytest.raises(ValueError, match="IK did not converge"):
        kin.ik([5.0, 0.0, 5.0], quaternion, INITIAL_Q)


@pytest.mark.parametrize("quaternion", ([0, 0, 0, 0], [0, 0, 0, 2],
                                        [0, float("nan"), 0, 1], [0, 0, 1]))
def test_invalid_pose_quaternion_is_rejected_before_planning(kin, quaternion):
    position, _ = kin.fk(INITIAL_Q)
    with pytest.raises(ValueError):
        cartesian_plan(kin, INITIAL_Q, [(position, quaternion)])


def test_zero_quaternion_is_rejected_by_direct_ik(kin):
    position, _ = kin.fk(INITIAL_Q)
    with pytest.raises(ValueError, match="nonzero"):
        kin.ik(position, [0, 0, 0, 0], INITIAL_Q)


def test_large_finite_quaternion_is_normalized_without_overflow(kin):
    position, orientation = kin.fk(INITIAL_Q)
    result = kin.ik(position, [value * 1e308 for value in orientation], INITIAL_Q)
    actual_position, actual_orientation = kin.fk(result)
    assert math.dist(position, actual_position) < 0.001
    assert np.linalg.norm(orientation_error(orientation, actual_orientation)) < 0.005


def test_urdf_control_mapping_and_installed_meshes():
    description = robot_description({"state_topic": "/test/measured"})
    root = ET.fromstring(description)
    assert "Copyright (c) 2015" in description
    assert {link.get("name") for link in root.findall("link")} == {
        "world", *(f"lbr_link_{index}" for index in range(8))}
    joints = [joint for joint in root.findall("joint") if joint.get("type") != "fixed"]
    assert [joint.get("name") for joint in joints] == JOINT_NAMES
    assert [joint.find("parent").get("link") for joint in joints] == [
        f"lbr_link_{index}" for index in range(7)]
    assert [joint.find("child").get("link") for joint in joints] == [
        f"lbr_link_{index}" for index in range(1, 8)]
    base_joint = next(joint for joint in root.findall("joint")
                      if joint.find("child").get("link") == "lbr_link_0")
    assert base_joint.get("type") == "fixed"
    assert base_joint.find("parent").get("link") == "world"
    origin = base_joint.find("origin")
    for attribute in ("xyz", "rpy"):
        values = "0 0 0" if origin is None else origin.get(attribute, "0 0 0")
        assert [float(value) for value in values.split()] == [0.0] * 3
    source = ET.parse(model_path()).getroot()
    for actual, original in zip(joints, source.findall("joint")):
        for tag in ("origin", "axis", "limit"):
            assert actual.find(tag).attrib == original.find(tag).attrib
    for mesh in root.findall(".//mesh"):
        path = Path(mesh.get("filename"))
        assert path.is_absolute() and path.is_file()
    hardware = root.find("ros2_control/hardware")
    assert hardware.find("plugin").text == "lbr_pybullet_control/PyBulletSystem"
    assert hardware.find("param[@name='state_topic']").text == "/test/measured"
    controlled = root.findall("ros2_control/joint")
    assert [joint.get("name") for joint in controlled] == JOINT_NAMES
    for index, joint in enumerate(controlled):
        assert [interface.get("name") for interface in joint.findall("command_interface")] == ["position"]
        assert {interface.get("name") for interface in joint.findall("state_interface")} == {"position", "velocity"}
        assert float(joint.find("command_interface/param[@name='min']").text) == LOWER_LIMITS[index]
        assert float(joint.find("command_interface/param[@name='max']").text) == UPPER_LIMITS[index]
        assert float(joint.find("state_interface[@name='position']/param[@name='initial_value']").text) == INITIAL_Q[index]


def test_generated_ros_urdf_has_same_fk_as_physics_model(kin, tmp_path):
    urdf = tmp_path / "ros_robot.urdf"
    urdf.write_text(robot_description())
    client = p.connect(p.DIRECT)
    try:
        robot = p.loadURDF(str(urdf), useFixedBase=True, physicsClientId=client)
        infos = [p.getJointInfo(robot, index, physicsClientId=client)
                 for index in range(p.getNumJoints(robot, physicsClientId=client))]
        assert sum(info[2] != p.JOINT_FIXED for info in infos) == 7
        by_name = {info[1].decode(): info[0] for info in infos}
        actuated_indices = [by_name[name] for name in JOINT_NAMES]
        ee_index = next(info[0] for info in infos if info[12].decode() == "lbr_link_7")
        base_index = next(info[0] for info in infos if info[12].decode() == "lbr_link_0")
        base_state = p.getLinkState(robot, base_index, computeForwardKinematics=True,
                                    physicsClientId=client)
        np.testing.assert_allclose(base_state[4], [0, 0, 0], atol=1e-7)
        np.testing.assert_allclose(base_state[5], [0, 0, 0, 1], atol=1e-7)
        for q in (INITIAL_Q, [0.2, -0.4, 0.3, -1.1, 0.15, 0.9, -0.2],
                  [-0.5, 0.7, -0.8, -0.3, 0.25, 0.4, 0.6]):
            for index, value in zip(actuated_indices, q):
                p.resetJointState(robot, index, value, physicsClientId=client)
            state = p.getLinkState(robot, ee_index, computeForwardKinematics=True, physicsClientId=client)
            position, quaternion = kin.fk(q)
            np.testing.assert_allclose(state[4], position, atol=1e-7)
            assert np.linalg.norm(orientation_error(quaternion, state[5])) < 1e-7
    finally:
        p.disconnect(physicsClientId=client)

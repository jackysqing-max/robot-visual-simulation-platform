# SPDX-License-Identifier: Apache-2.0
"""The exact iiwa model used by both tutorial physics and ROS kinematics.

The bundled PyBullet URDF retains its original BSD notice.  Renaming joints in
the generated ROS description changes identifiers, never geometry or axes.
Each :class:`Kinematics` owns a private DIRECT client; resetting that client for
FK or IK cannot teleport the independently running physics robot.
"""

from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np
import pybullet as p
import pybullet_data


JOINT_NAMES = [f"lbr_A{index}" for index in range(1, 8)]
BULLET_JOINT_NAMES = [f"iiwa_joint_{index}" for index in range(1, 8)]
INITIAL_Q = [0.0, 0.35, 0.0, -0.9, 0.0, 0.65, 0.0]
LOWER_LIMITS = [-2.96705972839, -2.09439510239, -2.96705972839,
                -2.09439510239, -2.96705972839, -2.09439510239,
                -3.05432619099]
UPPER_LIMITS = [-value for value in LOWER_LIMITS]
EE_LINK = 6
BASE_FRAME = "lbr_link_0"


def model_path():
    """Return the installed PyBullet iiwa URDF, with its packaged meshes."""
    return Path(pybullet_data.getDataPath()) / "kuka_iiwa" / "model.urdf"


def finite_vector(values, length, label):
    """Reject malformed/non-finite numeric commands before touching Bullet."""
    try:
        result = [float(value) for value in values]
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{label} must contain {length} finite numbers") from error
    if len(result) != length or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} must contain {length} finite numbers")
    return result


def validate_joints(q):
    """Validate seven joint positions in radians against the model's limits."""
    result = finite_vector(q, 7, "joint positions")
    for index, (value, lower, upper) in enumerate(zip(result, LOWER_LIMITS, UPPER_LIMITS)):
        if not lower <= value <= upper:
            raise ValueError(f"{JOINT_NAMES[index]}={value:.6f} outside [{lower:.6f}, {upper:.6f}] rad")
    return result


def normalized_quaternion(quaternion):
    result = finite_vector(quaternion, 4, "quaternion (x y z w)")
    norm = math.hypot(*result)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("quaternion must have nonzero norm")
    return [value / norm for value in result]


def orientation_error(target, actual):
    """Shortest world-frame rotation vector taking actual to target."""
    # Quaternion target * inverse(actual), in Bullet's x y z w order.
    x, y, z, w = target
    ax, ay, az, aw = actual
    vector = np.array([
        -w * ax + x * aw - y * az + z * ay,
        -w * ay + x * az + y * aw - z * ax,
        -w * az - x * ay + y * ax + z * aw,
    ])
    scalar = w * aw + x * ax + y * ay + z * az
    if scalar < 0:
        vector, scalar = -vector, -scalar
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        return 2.0 * vector
    return vector * (2.0 * math.atan2(norm, scalar) / norm)


def robot_description(hardware_parameters=None, initial_q=None):
    """Generate exact Bullet geometry with LBR names and a ROS control system.

    ``hardware_parameters`` are emitted verbatim under ``<hardware>``.  Paths to
    the installed meshes are made absolute so robot_state_publisher/RViz can
    resolve them without depending on the caller's current directory.
    """
    initial_q = validate_joints(INITIAL_Q if initial_q is None else initial_q)
    source = model_path().read_text()
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    root = ET.fromstring(source, parser=parser)
    root.set("name", "lbr")
    ET.SubElement(root, "link", {"name": "world"})
    fixed = ET.SubElement(root, "joint", {"name": "world_to_lbr", "type": "fixed"})
    ET.SubElement(fixed, "parent", {"link": "world"})
    ET.SubElement(fixed, "child", {"link": "lbr_link_0"})
    for index in range(8):
        old, new = f"lbr_iiwa_link_{index}", f"lbr_link_{index}"
        for element in root.iter():
            for attribute in ("name", "link"):
                if element.get(attribute) == old:
                    element.set(attribute, new)
    for index, joint_name in enumerate(JOINT_NAMES, start=1):
        root.find(f"joint[@name='lbr_iiwa_joint_{index}']").set("name", joint_name)
    for mesh in root.findall(".//mesh"):
        mesh.set("filename", str(model_path().parent / mesh.get("filename")))
    system = ET.SubElement(root, "ros2_control", {"name": "PyBulletSystem", "type": "system"})
    hardware = ET.SubElement(system, "hardware")
    ET.SubElement(hardware, "plugin").text = "lbr_pybullet_control/PyBulletSystem"
    parameters = {"command_topic": "/iiwa7/joint_desired", "state_topic": "/iiwa7/joint_states",
                  "state_timeout": "0.5"}
    parameters.update(hardware_parameters or {})
    for name, value in parameters.items():
        ET.SubElement(hardware, "param", {"name": str(name)}).text = str(value)
    for name, initial, lower, upper in zip(JOINT_NAMES, initial_q, LOWER_LIMITS, UPPER_LIMITS):
        joint = ET.SubElement(system, "joint", {"name": name})
        command = ET.SubElement(joint, "command_interface", {"name": "position"})
        ET.SubElement(command, "param", {"name": "min"}).text = str(lower)
        ET.SubElement(command, "param", {"name": "max"}).text = str(upper)
        for interface in ("position", "velocity"):
            state = ET.SubElement(joint, "state_interface", {"name": interface})
            ET.SubElement(state, "param", {"name": "initial_value"}).text = str(initial if interface == "position" else 0.0)
    # ElementTree does not retain comments before the root; preserve that
    # preamble explicitly because it contains the original BSD license.
    return source.split("<robot", 1)[0] + ET.tostring(root, encoding="unicode")


build_robot_description = robot_description


class Kinematics:
    """Isolated FK/IK using the exact simulated joint axes and link frame."""

    def __init__(self):
        self.client = p.connect(p.DIRECT)
        self.robot = p.loadURDF(str(model_path()), useFixedBase=True, physicsClientId=self.client)
        self.lower = list(LOWER_LIMITS)
        self.upper = list(UPPER_LIMITS)

    def close(self):
        if self.client is not None:
            p.disconnect(physicsClientId=self.client)
            self.client = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def _set_q(self, q):
        if self.client is None:
            raise RuntimeError("kinematics client is closed")
        for index, value in enumerate(q):
            p.resetJointState(self.robot, index, value, physicsClientId=self.client)

    def fk(self, q):
        self._set_q(validate_joints(q))
        state = p.getLinkState(self.robot, EE_LINK, computeForwardKinematics=True,
                               physicsClientId=self.client)
        # 4/5 are URDF link-frame coordinates; 0/1 are the inertial COM frame.
        return list(state[4]), list(state[5])

    def ik(self, position, orientation, seed, position_tolerance=0.001, orientation_tolerance=0.005):
        """Return bounded joints only after an independent FK residual check.

        A Bullet solve provides an initial guess, followed by damped Jacobian
        refinement.  Infeasible targets raise ``ValueError`` instead of silently
        publishing the unconverged approximation returned by a raw Bullet IK.
        """
        target = np.asarray(finite_vector(position, 3, "position (m)"))
        quaternion = normalized_quaternion(orientation)
        seed = validate_joints(seed)
        if not (math.isfinite(position_tolerance) and position_tolerance > 0 and
                math.isfinite(orientation_tolerance) and orientation_tolerance > 0):
            raise ValueError("IK tolerances must be positive finite numbers")
        self._set_q(seed)
        result = p.calculateInverseKinematics(
            self.robot, EE_LINK, target.tolist(), quaternion,
            lowerLimits=self.lower, upperLimits=self.upper,
            jointRanges=[upper - lower for lower, upper in zip(self.lower, self.upper)],
            restPoses=seed, maxNumIterations=200, residualThreshold=1e-7,
            physicsClientId=self.client,
        )
        q = np.clip(np.asarray(result[:7]), self.lower, self.upper)
        position_residual, rotation_residual = float("inf"), float("inf")
        for _ in range(300):
            actual_position, actual_quaternion = self.fk(q)
            position_error = target - actual_position
            rotation_error = orientation_error(quaternion, actual_quaternion)
            position_residual = float(np.linalg.norm(position_error))
            rotation_residual = float(np.linalg.norm(rotation_error))
            if position_residual <= position_tolerance and rotation_residual <= orientation_tolerance:
                return validate_joints(q)
            linear, angular = p.calculateJacobian(
                self.robot, EE_LINK, [0.0, 0.0, 0.0], q.tolist(), [0.0] * 7,
                [0.0] * 7, physicsClientId=self.client,
            )
            jacobian = np.vstack((linear, angular))
            error = np.concatenate((position_error, rotation_error))
            damping = 0.01
            delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping**2 * np.eye(6), error)
            largest = float(np.max(np.abs(delta)))
            if largest > 0.15:
                delta *= 0.15 / largest
            q = np.clip(q + delta, self.lower, self.upper)
        raise ValueError(f"IK did not converge within joint limits: position error={position_residual:.6f} m, orientation error={rotation_residual:.6f} rad")


PyBulletKinematics = Kinematics

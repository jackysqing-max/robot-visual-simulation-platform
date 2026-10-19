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
from functools import lru_cache
import atexit
import os
import shutil
import tempfile

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


@lru_cache(maxsize=8)
def _project_spec(path, modified):
    from .project_config import load_project, inspect_robot, build_model
    project = load_project(path)
    metadata = inspect_robot(project)
    directory = Path(tempfile.mkdtemp(prefix='lbr_workbench_model_'))
    atexit.register(shutil.rmtree, directory, ignore_errors=True)
    return project, metadata, Path(build_model(project, directory))


def project_spec():
    path = os.environ.get('LBR_PYBULLET_PROJECT', '')
    if not path:
        return None
    path = str(Path(path).expanduser().resolve())
    return _project_spec(path, Path(path).stat().st_mtime_ns)


def initial_positions():
    spec = project_spec()
    return list(spec[1].default_q) if spec else list(INITIAL_Q)


def model_limits():
    spec = project_spec()
    return (list(spec[1].lower_limits), list(spec[1].upper_limits)) if spec else (LOWER_LIMITS, UPPER_LIMITS)


def model_path():
    """Return the installed PyBullet iiwa URDF, with its packaged meshes."""
    spec = project_spec()
    return spec[2] if spec else Path(pybullet_data.getDataPath()) / "kuka_iiwa" / "model.urdf"


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
    lower_limits, upper_limits = model_limits()
    result = finite_vector(q, len(lower_limits), "joint positions")
    for index, (value, lower, upper) in enumerate(zip(result, lower_limits, upper_limits)):
        if not lower <= value <= upper:
            name = JOINT_NAMES[index] if index < len(JOINT_NAMES) else f'Joint {index+1}'
            raise ValueError(f"{name}={value:.6f} outside [{lower:.6f}, {upper:.6f}] rad")
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
    spec = project_spec()
    if spec and not spec[1].control_compatible:
        raise ValueError('This robot is available in preview mode: '+spec[1].control_reason)
    initial_q = validate_joints(initial_positions() if initial_q is None else initial_q)
    source = model_path().read_text()
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    root = ET.fromstring(source, parser=parser)
    root.set("name", "lbr")
    builtin = not spec or not spec[0].robot_urdf
    for index in range(8) if builtin else []:
        old, new = f"lbr_iiwa_link_{index}", f"lbr_link_{index}"
        for element in root.iter():
            for attribute in ("name", "link"):
                if element.get(attribute) == old:
                    element.set(attribute, new)
    native_joints = spec[1].joint_names if spec else [f'lbr_iiwa_joint_{i}' for i in range(1, 8)]
    for native, joint_name in zip(native_joints, JOINT_NAMES):
        root.find(f"joint[@name='{native}']").set("name", joint_name)
    root_link = 'lbr_link_0' if builtin else spec[1].root_link
    existing_links = {link.get('name') for link in root.findall('link')}
    for reserved in ('world', 'lbr_link_0'):
        if reserved in existing_links and root_link != reserved:
            raise ValueError(f'Imported model has an ambiguous reserved base frame {reserved}; '
                             'rename that non-root link before enabling LBR control')
    if root_link != 'world':
        ET.SubElement(root, 'link', {'name': 'world'})
        fixed = ET.SubElement(root, 'joint', {'name': 'world_to_robot', 'type': 'fixed'})
        ET.SubElement(fixed, 'parent', {'link': 'world'})
        ET.SubElement(fixed, 'child', {'link': root_link})
    if root_link != 'lbr_link_0':
        ET.SubElement(root, 'link', {'name': 'lbr_link_0'})
        fixed = ET.SubElement(root, 'joint', {'name': 'base_frame_alias', 'type': 'fixed'})
        ET.SubElement(fixed, 'parent', {'link': root_link})
        ET.SubElement(fixed, 'child', {'link': 'lbr_link_0'})
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
    for name, initial, lower, upper in zip(JOINT_NAMES, initial_q, *model_limits()):
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
        self.lower, self.upper = map(list, model_limits())
        bullet_indices = [i for i in range(p.getNumJoints(self.robot, physicsClientId=self.client))
                              if p.getJointInfo(self.robot, i, physicsClientId=self.client)[2]
                              in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)]
        spec = project_spec()
        native = {p.getJointInfo(self.robot, i, physicsClientId=self.client)[1].decode(): i
                  for i in bullet_indices}
        self.joint_indices = [native[name] for name in spec[1].joint_names] if spec else bullet_indices
        self.dof_order = [bullet_indices.index(i) for i in self.joint_indices]
        links = {p.getJointInfo(self.robot, i, physicsClientId=self.client)[12].decode(): i
                 for i in range(p.getNumJoints(self.robot, physicsClientId=self.client))}
        self.ee_link = links.get('workbench_tcp', EE_LINK)

    def _bullet_order(self, values):
        result = [0.0]*len(values)
        for value, index in zip(values, self.dof_order):
            result[index] = value
        return result

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
        for index, value in zip(self.joint_indices, q):
            p.resetJointState(self.robot, index, value, physicsClientId=self.client)

    def fk(self, q):
        self._set_q(validate_joints(q))
        state = p.getLinkState(self.robot, self.ee_link, computeForwardKinematics=True,
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
            self.robot, self.ee_link, target.tolist(), quaternion,
            lowerLimits=self._bullet_order(self.lower), upperLimits=self._bullet_order(self.upper),
            jointRanges=self._bullet_order([upper-lower for lower, upper in zip(self.lower, self.upper)]),
            restPoses=self._bullet_order(seed), maxNumIterations=200, residualThreshold=1e-7,
            physicsClientId=self.client,
        )
        n = len(self.joint_indices)
        q = np.clip(np.asarray([result[i] for i in self.dof_order]), self.lower, self.upper)
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
                self.robot, self.ee_link, [0.0, 0.0, 0.0], self._bullet_order(q), [0.0] * n,
                [0.0] * n, physicsClientId=self.client,
            )
            jacobian = np.vstack((linear, angular))[:, self.dof_order]
            error = np.concatenate((position_error, rotation_error))
            damping = 0.01
            delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping**2 * np.eye(6), error)
            largest = float(np.max(np.abs(delta)))
            if largest > 0.15:
                delta *= 0.15 / largest
            q = np.clip(q + delta, self.lower, self.upper)
        raise ValueError(f"IK did not converge within joint limits: position error={position_residual:.6f} m, orientation error={rotation_residual:.6f} rad")


PyBulletKinematics = Kinematics

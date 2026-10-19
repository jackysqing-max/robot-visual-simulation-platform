# SPDX-License-Identifier: Apache-2.0
"""Small physics backend compatible with the existing iiwa7 ROS topic API."""

import math
import time
import json
from collections import deque

import pybullet as p
import pybullet_data
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import Image

from .model import (BASE_FRAME, EE_LINK, initial_positions, model_path,
                    project_spec, validate_joints)


class PyBulletSim(Node):
    """Run one physics step per wall timer tick, then publish measured feedback."""

    def __init__(self):
        super().__init__("lbr_pybullet_sim")
        defaults = {
            "gui": False,
            "show_ee_trace": True,
            "sim_hz": 240.0,
            "command_timeout": 0.5,
            "initial_q": initial_positions(),
            "render_camera": False,
            "camera_hz": 8.0,
            "position_command_topic": "/iiwa7/joint_desired",
            "joint_state_topic": "/iiwa7/joint_states",
            "ee_pose_topic": "/lbr/ee_pose",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        sim_hz = float(self.get_parameter("sim_hz").value)
        timeout = float(self.get_parameter("command_timeout").value)
        if not math.isfinite(sim_hz) or not 100.0 <= sim_hz <= 1000.0:
            raise ValueError("sim_hz must be finite and between 100 and 1000 Hz")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("command_timeout must be positive and finite")
        self.dt, self.command_timeout = 1.0 / sim_hz, timeout
        self.q_command = validate_joints(self.get_parameter("initial_q").value)
        self.gui = bool(self.get_parameter("gui").value)
        self.trace_enabled = self.gui and bool(self.get_parameter("show_ee_trace").value)
        self.trace_previous = None
        self.trace_segments = deque()
        self.last_command = None
        self.watchdog_holding = True
        self.client = p.connect(p.GUI if self.gui else p.DIRECT)
        if self.client < 0:
            raise RuntimeError("could not create the PyBullet client")
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)
        p.setRealTimeSimulation(0, physicsClientId=self.client)
        p.setTimeStep(self.dt, physicsClientId=self.client)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client)
        p.setPhysicsEngineParameter(numSolverIterations=100, physicsClientId=self.client)
        p.loadURDF("plane.urdf", physicsClientId=self.client)
        self.robot = p.loadURDF(str(model_path()), useFixedBase=True, physicsClientId=self.client)
        self.joint_indices = [i for i in range(p.getNumJoints(self.robot, physicsClientId=self.client))
                              if p.getJointInfo(self.robot, i, physicsClientId=self.client)[2]
                              in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)]
        self.n = len(self.joint_indices)
        self.joint_names = [f'iiwa_joint_{i+1}' for i in range(self.n)]
        links = {p.getJointInfo(self.robot, i, physicsClientId=self.client)[12].decode(): i
                 for i in range(p.getNumJoints(self.robot, physicsClientId=self.client))}
        spec = project_spec()
        if spec:
            native = {p.getJointInfo(self.robot, i, physicsClientId=self.client)[1].decode(): i
                      for i in self.joint_indices}
            self.joint_indices = [native[name] for name in spec[1].joint_names]
        self.forces = [min(300.0, limit) for limit in spec[1].effort_limits] if spec else [300.0]*self.n
        self.flange_link = links.get(spec[1].flange_link, -1) if spec else EE_LINK
        self.tcp_link = links.get('workbench_tcp', self.flange_link)
        for index, value in zip(self.joint_indices, self.q_command):
            p.resetJointState(self.robot, index, value, physicsClientId=self.client)
        if self.gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client)
            p.resetDebugVisualizerCamera(1.5, 45, -30, [0.25, 0, 0.5], physicsClientId=self.client)
        self.state_pub = self.create_publisher(JointState, self.get_parameter("joint_state_topic").value, 10)
        self.pose_pub = self.create_publisher(PoseStamped, self.get_parameter("ee_pose_topic").value, 10)
        self.tcp_pub = self.create_publisher(PoseStamped, '/lbr/tcp_pose', 10)
        from rclpy.qos import DurabilityPolicy, QoSProfile
        config_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.config_pub = self.create_publisher(String, '/lbr/sim/config', config_qos)
        if spec:
            from dataclasses import asdict
            description = dict(project=asdict(spec[0]), robot_name=spec[1].robot_name,
                               joint_names=spec[1].joint_names, state_names=self.joint_names,
                               control_compatible=spec[1].control_compatible,
                               flange_link=spec[1].flange_link, tcp_link='workbench_tcp')
        else:
            description = dict(robot_name='KUKA LBR iiwa 7', control_compatible=True,
                               tcp_link='lbr_iiwa_link_7', state_names=self.joint_names)
        self.config_pub.publish(String(data=json.dumps(description, allow_nan=False)))
        self.camera_yaw, self.camera_pitch, self.camera_distance = 45.0, -28.0, 1.5
        self.camera_target = [0.25, 0.0, 0.5]
        self.create_subscription(Float64MultiArray, '/lbr/sim/camera', self.on_camera, 1)
        if self.get_parameter('render_camera').value:
            from rclpy.qos import qos_profile_sensor_data
            self.image_pub = self.create_publisher(Image, '/lbr/sim/camera/image', qos_profile_sensor_data)
            self.create_timer(1.0/float(self.get_parameter('camera_hz').value), self.render_frame)
        self.command_sub = self.create_subscription(
            Float64MultiArray, self.get_parameter("position_command_topic").value, self.on_command, 10)
        self.timer = self.create_timer(self.dt, self.step)
        self.get_logger().info(f"PyBullet iiwa physics ready at {sim_hz:g} Hz; end effector is link 7 in {BASE_FRAME}")

    def on_camera(self, message):
        values = list(message.data)
        if len(values) not in (3, 6) or not all(map(math.isfinite, values)):
            return
        yaw, pitch, distance = values[:3]
        if not -89 <= pitch <= 89 or not 0.15 <= distance <= 10:
            return
        self.camera_yaw, self.camera_pitch, self.camera_distance = yaw, pitch, distance
        if len(values) == 6:
            self.camera_target = values[3:]

    def link_pose(self, index):
        if index >= 0:
            state = p.getLinkState(self.robot, index, computeForwardKinematics=True,
                                   physicsClientId=self.client)
            return state[4], state[5]
        position, orientation = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.client)
        inertial = p.getDynamicsInfo(self.robot, -1, physicsClientId=self.client)
        inverse = p.invertTransform(inertial[3], inertial[4])
        return p.multiplyTransforms(position, orientation, *inverse)

    def render_frame(self):
        import numpy as np
        width, height = 640, 400
        view = p.computeViewMatrixFromYawPitchRoll(self.camera_target, self.camera_distance,
                                                  self.camera_yaw, self.camera_pitch, 0, 2)
        projection = p.computeProjectionMatrixFOV(55, width/height, 0.02, 20.0)
        frame = p.getCameraImage(width, height, view, projection, renderer=p.ER_TINY_RENDERER,
                                 physicsClientId=self.client)
        rgb = np.asarray(frame[2], dtype=np.uint8).reshape(height, width, 4)[:, :, :3].copy()
        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = BASE_FRAME
        message.width, message.height = width, height
        message.encoding = 'rgb8'
        message.step = width*3
        message.data = rgb.tobytes()
        self.image_pub.publish(message)

    def on_command(self, message):
        try:
            command = validate_joints(message.data)
        except ValueError as error:
            self.get_logger().warning(f"Rejected joint command: {error}")
            return
        self.q_command = command
        self.last_command = time.monotonic()
        self.watchdog_holding = False

    def step(self):
        if self.last_command is not None and time.monotonic() - self.last_command > self.command_timeout and not self.watchdog_holding:
            self.q_command = [state[0] for state in p.getJointStates(
                self.robot, self.joint_indices, physicsClientId=self.client)]
            self.watchdog_holding = True
            self.get_logger().warning("Position command watchdog expired; holding current measured joints")
        if self.n:
            p.setJointMotorControlArray(
                self.robot, self.joint_indices, p.POSITION_CONTROL,
                targetPositions=self.q_command, targetVelocities=[0.0] * self.n,
                forces=self.forces, positionGains=[0.3] * self.n, velocityGains=[1.0] * self.n,
                physicsClientId=self.client,
            )
        p.stepSimulation(physicsClientId=self.client)
        states = p.getJointStates(self.robot, self.joint_indices, physicsClientId=self.client)
        stamp = self.get_clock().now().to_msg()
        feedback = JointState()
        feedback.header.stamp = stamp
        feedback.header.frame_id = BASE_FRAME
        feedback.name = self.joint_names
        feedback.position = [state[0] for state in states]
        feedback.velocity = [state[1] for state in states]
        feedback.effort = [state[3] for state in states]
        self.state_pub.publish(feedback)
        flange_xyz, flange_quat = self.link_pose(self.flange_link)
        if self.trace_enabled:
            xyz = self.link_pose(self.tcp_link)[0]
            if self.trace_previous is None:
                self.trace_previous = xyz
            elif math.dist(self.trace_previous, xyz) >= 0.0005:
                self.trace_segments.append(p.addUserDebugLine(
                    self.trace_previous, xyz, [0.0, 0.85, 0.3], lineWidth=2.0,
                    physicsClientId=self.client))
                self.trace_previous = xyz
                if len(self.trace_segments) > 600:
                    p.removeUserDebugItem(self.trace_segments.popleft(), physicsClientId=self.client)
        pose = PoseStamped()
        pose.header = feedback.header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = flange_xyz
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = flange_quat
        self.pose_pub.publish(pose)
        tcp_xyz, tcp_quat = self.link_pose(self.tcp_link)
        tcp_pose = PoseStamped()
        tcp_pose.header = feedback.header
        tcp_pose.pose.position.x, tcp_pose.pose.position.y, tcp_pose.pose.position.z = tcp_xyz
        (tcp_pose.pose.orientation.x, tcp_pose.pose.orientation.y,
         tcp_pose.pose.orientation.z, tcp_pose.pose.orientation.w) = tcp_quat
        self.tcp_pub.publish(tcp_pose)

    def destroy_node(self):
        if getattr(self, "client", None) is not None:
            p.disconnect(physicsClientId=self.client)
            self.client = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PyBulletSim()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

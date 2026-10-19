# SPDX-License-Identifier: Apache-2.0
"""Small physics backend compatible with the existing iiwa7 ROS topic API."""

import math
import time
from collections import deque

import pybullet as p
import pybullet_data
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .model import BASE_FRAME, BULLET_JOINT_NAMES, EE_LINK, INITIAL_Q, model_path, validate_joints


class PyBulletSim(Node):
    """Run one physics step per wall timer tick, then publish measured feedback."""

    def __init__(self):
        super().__init__("lbr_pybullet_sim")
        defaults = {
            "gui": False,
            "show_ee_trace": True,
            "sim_hz": 240.0,
            "command_timeout": 0.5,
            "initial_q": INITIAL_Q,
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
        for index, value in enumerate(self.q_command):
            p.resetJointState(self.robot, index, value, physicsClientId=self.client)
        if self.gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client)
            p.resetDebugVisualizerCamera(1.5, 45, -30, [0.25, 0, 0.5], physicsClientId=self.client)
        self.state_pub = self.create_publisher(JointState, self.get_parameter("joint_state_topic").value, 10)
        self.pose_pub = self.create_publisher(PoseStamped, self.get_parameter("ee_pose_topic").value, 10)
        self.command_sub = self.create_subscription(
            Float64MultiArray, self.get_parameter("position_command_topic").value, self.on_command, 10)
        self.timer = self.create_timer(self.dt, self.step)
        self.get_logger().info(f"PyBullet iiwa physics ready at {sim_hz:g} Hz; end effector is link 7 in {BASE_FRAME}")

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
                self.robot, list(range(7)), physicsClientId=self.client)]
            self.watchdog_holding = True
            self.get_logger().warning("Position command watchdog expired; holding current measured joints")
        p.setJointMotorControlArray(
            self.robot, list(range(7)), p.POSITION_CONTROL,
            targetPositions=self.q_command, targetVelocities=[0.0] * 7,
            forces=[300.0] * 7, positionGains=[0.3] * 7, velocityGains=[1.0] * 7,
            physicsClientId=self.client,
        )
        p.stepSimulation(physicsClientId=self.client)
        states = p.getJointStates(self.robot, list(range(7)), physicsClientId=self.client)
        stamp = self.get_clock().now().to_msg()
        feedback = JointState()
        feedback.header.stamp = stamp
        feedback.header.frame_id = BASE_FRAME
        feedback.name = list(BULLET_JOINT_NAMES)
        feedback.position = [state[0] for state in states]
        feedback.velocity = [state[1] for state in states]
        feedback.effort = [state[3] for state in states]
        self.state_pub.publish(feedback)
        state = p.getLinkState(self.robot, EE_LINK, computeForwardKinematics=True, physicsClientId=self.client)
        if self.trace_enabled:
            xyz = state[4]
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
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = state[4]
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = state[5]
        self.pose_pub.publish(pose)

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

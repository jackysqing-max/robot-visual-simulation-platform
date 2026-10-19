"""Qt-thread ROS adapter. The workbench only controls its local simulation graph."""
import json
import math
import time

from PyQt5 import QtCore, QtGui
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String, Float64MultiArray
from std_srvs.srv import Trigger
from control_msgs.msg import JointTrajectoryControllerState


class WorkbenchBridge(QtCore.QObject):
    image = QtCore.pyqtSignal(QtGui.QImage)
    snapshot = QtCore.pyqtSignal(dict)
    event = QtCore.pyqtSignal(dict)
    connection = QtCore.pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init(args=[])
        self.node = rclpy.create_node('robot_workbench_ui')
        self.data = dict(names=[], positions=[], velocities=[], efforts=[], desired=[],
                         ee_xyz=[], ee_xyzw=[], tcp_xyz=[], tcp_xyzw=[])
        self._raw_at = self._controller_at = self._emitted_at = 0.0
        self._raw_sequence = 0
        self._connected = None
        sensor = qos_profile_sensor_data
        self.node.create_subscription(JointState, '/iiwa7/joint_states', self._joint_state, sensor)
        self.node.create_subscription(JointTrajectoryControllerState,
                                      '/lbr/joint_trajectory_controller/state', self._controller, sensor)
        self.node.create_subscription(PoseStamped, '/lbr/ee_pose', lambda m: self._pose(m, 'ee'), sensor)
        self.node.create_subscription(PoseStamped, '/lbr/tcp_pose', lambda m: self._pose(m, 'tcp'), sensor)
        self.node.create_subscription(Image, '/lbr/sim/camera/image', self._image, sensor)
        status_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, '/lbr_tutorial/status', self._event, status_qos)
        self.joint_pub = self.node.create_publisher(JointState, '/lbr_tutorial/joint_goal', 10)
        self.pose_pub = self.node.create_publisher(PoseStamped, '/lbr_tutorial/pose_goal', 10)
        self.path_pub = self.node.create_publisher(Path, '/lbr_tutorial/cartesian_path', 10)
        self.camera_pub = self.node.create_publisher(Float64MultiArray, '/lbr/sim/camera', 1)
        self.cancel_client = self.node.create_client(Trigger, '/lbr_tutorial/cancel')

    def _joint_state(self, message):
        count = len(message.name)
        if len(message.position) != count or len(set(message.name)) != count:
            return
        expected = [f'iiwa_joint_{i+1}' for i in range(count)]
        indices = [message.name.index(name) for name in expected] if set(expected) == set(message.name) else list(range(count))
        def ordered(values):
            return [values[i] for i in indices] if len(values) == count else []
        # Canonical controller names for seven-axis operation, numbered labels in preview.
        self.data.update(names=[f'lbr_A{i+1}' for i in range(count)],
                         positions=ordered(message.position), velocities=ordered(message.velocity),
                         efforts=ordered(message.effort))
        self._raw_at = time.monotonic()
        self._raw_sequence += 1

    def _controller(self, message):
        values = dict(zip(message.joint_names, message.desired.positions))
        self.data['desired'] = [values.get(n, float('nan')) for n in self.data['names']]
        self._controller_at = time.monotonic()

    def _pose(self, message, prefix):
        p, q = message.pose.position, message.pose.orientation
        self.data[prefix+'_xyz'] = [p.x, p.y, p.z]
        self.data[prefix+'_xyzw'] = [q.x, q.y, q.z, q.w]

    def _image(self, message):
        if message.encoding != 'rgb8' or message.step < message.width*3:
            return
        content = bytes(message.data)
        if len(content) < message.step*message.height:
            return
        frame = QtGui.QImage(content, message.width, message.height, message.step,
                             QtGui.QImage.Format_RGB888).copy()
        self.image.emit(frame)

    def _event(self, message):
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                self.event.emit(value)
        except ValueError:
            self.event.emit(dict(event='error', reason='Invalid motion status JSON received'))

    def spin(self):
        if self.node is None or not rclpy.ok():
            return
        begin = time.monotonic()
        for _ in range(24):
            rclpy.spin_once(self.node, timeout_sec=0)
            if time.monotonic()-begin > 0.004:
                break
        now = time.monotonic()
        if now-self._emitted_at >= 0.1:
            self._emitted_at = now
            connected = now-self._raw_at < 0.75
            self.snapshot.emit(dict(self.data, raw_age=now-self._raw_at,
                                    controller_age=now-self._controller_at,
                                    raw_sequence=self._raw_sequence))
            if connected != self._connected:
                self._connected = connected
                self.connection.emit(connected)

    def send_joint(self, positions):
        values = [float(v) for v in positions]
        if len(values) != 7 or not all(map(math.isfinite, values)):
            raise ValueError('Seven finite joint targets are required')
        self.joint_pub.publish(JointState(name=[f'lbr_A{i+1}' for i in range(7)], position=values))

    @staticmethod
    def _pose_message(xyz, xyzw):
        if len(xyz) != 3 or len(xyzw) != 4 or not all(map(math.isfinite, [*xyz, *xyzw])):
            raise ValueError('TCP goal must contain finite XYZ and XYZW values')
        message = PoseStamped()
        message.header.frame_id = 'lbr_link_0'
        message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, xyz)
        (message.pose.orientation.x, message.pose.orientation.y,
         message.pose.orientation.z, message.pose.orientation.w) = map(float, xyzw)
        return message

    def send_pose(self, xyz, xyzw):
        self.pose_pub.publish(self._pose_message(xyz, xyzw))

    def send_path(self, poses):
        message = Path()
        message.header.frame_id = 'lbr_link_0'
        message.poses = [self._pose_message(xyz, xyzw) for xyz, xyzw in poses]
        self.path_pub.publish(message)

    def cancel(self):
        if not self.cancel_client.service_is_ready():
            self.event.emit(dict(event='error', reason='Motion cancel service is unavailable'))
            return
        self.cancel_client.call_async(Trigger.Request())

    def set_camera(self, yaw, pitch, distance):
        self.camera_pub.publish(Float64MultiArray(data=[float(yaw), float(pitch), float(distance)]))

    def close(self):
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self._owns_context and rclpy.ok():
            rclpy.shutdown()

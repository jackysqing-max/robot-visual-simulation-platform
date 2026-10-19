"""Optional seven-joint adapter for the original C++ Robot Monitor process.

This script is executed in the original workspace's ROS environment. Keeping its
custom message import here avoids making robot_control_msgs a workbench dependency.
It publishes monitoring data only and never publishes motion commands.
"""

import math
import time

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_control_msgs.msg import RobotState
from sensor_msgs.msg import JointState


RAW_NAMES = tuple(f'iiwa_joint_{index}' for index in range(1, 8))
CONTROL_NAMES = tuple(f'lbr_A{index}' for index in range(1, 8))


def ordered_values(names, values, expected):
    if len(names) != len(values) or len(set(names)) != len(names):
        return None
    by_name = dict(zip(names, values))
    if not all(name in by_name and math.isfinite(by_name[name]) for name in expected):
        return None
    return [float(by_name[name]) for name in expected]


class LegacyMonitorBridge(Node):
    def __init__(self):
        super().__init__('workbench_legacy_monitor_bridge')
        self.raw = None
        self.raw_at = None
        self.desired = None
        self.desired_at = 0.0
        self.last_stamp = None
        self.waiting_warning = False
        self.publisher = self.create_publisher(RobotState, '/workbench/robot_states', 10)
        self.create_subscription(JointState, '/iiwa7/joint_states', self.on_state,
                                 qos_profile_sensor_data)
        self.create_subscription(JointTrajectoryControllerState,
                                 '/lbr/joint_trajectory_controller/state', self.on_controller,
                                 qos_profile_sensor_data)
        self.create_timer(1.0 / 30.0, self.publish_snapshot)
        self.get_logger().info('Read-only legacy monitor bridge ready; seven-joint LBR feedback only')

    def on_state(self, message):
        arrays = [ordered_values(message.name, values, RAW_NAMES)
                  for values in (message.position, message.velocity, message.effort)]
        if any(values is None for values in arrays):
            return
        self.raw = (message.header, arrays)
        self.raw_at = time.monotonic()

    def on_controller(self, message):
        desired = ordered_values(message.joint_names, message.desired.positions, CONTROL_NAMES)
        if desired is not None:
            self.desired = desired
            self.desired_at = time.monotonic()

    def publish_snapshot(self):
        if self.raw is None or time.monotonic() - self.raw_at > 0.5:
            return
        if self.desired is None or time.monotonic()-self.desired_at > 0.5:
            if not self.waiting_warning:
                self.get_logger().info('Waiting for controller desired positions before publishing RobotState')
                self.waiting_warning = True
            return
        header, arrays = self.raw
        stamp = (header.stamp.sec, header.stamp.nanosec)
        if stamp == self.last_stamp:
            return
        self.last_stamp = stamp
        message = RobotState()
        message.header = header
        message.robot_state = arrays[0] + arrays[1] + arrays[2] + self.desired
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = LegacyMonitorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

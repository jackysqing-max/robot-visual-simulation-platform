"""Run a topic-driven motion demo and save measurements from controller state."""

import argparse
import json
import math
from pathlib import Path as FilePath
import time

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .model import JOINT_NAMES, Kinematics
from .motion_node import ACTION
from .planning import quaternion_distance


def stamped_pose(xyz, quat):
    msg = PoseStamped()
    msg.header.frame_id = 'lbr_link_0'
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz
    (msg.pose.orientation.x, msg.pose.orientation.y,
     msg.pose.orientation.z, msg.pose.orientation.w) = quat
    return msg


def polyline_distance(point, vertices):
    distances = []
    for a, b in zip(vertices, vertices[1:]):
        segment = [y-x for x, y in zip(a, b)]
        norm2 = sum(v*v for v in segment)
        u = (sum((x-y)*v for x, y, v in zip(point, a, segment))/norm2) if norm2 else 0
        u = max(0.0, min(1.0, u))
        distances.append(math.dist(point, [x+u*v for x, v in zip(a, segment)]))
    return min(distances)


class Demo(Node):
    def __init__(self):
        super().__init__('lbr_tutorial_demo')
        self.kin = Kinematics()
        self.q = None
        self.state_at = 0.0
        self.events = []
        self.ready_seen = False
        self.samples = []
        self.record = False
        self.cancel_client = self.create_client(Trigger, '/lbr_tutorial/cancel')
        self.joint_pub = self.create_publisher(JointState, '/lbr_tutorial/joint_goal', 10)
        self.path_pub = self.create_publisher(Path, '/lbr_tutorial/cartesian_path', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/lbr_tutorial/pose_goal', 10)
        self.create_subscription(JointState, '/lbr/joint_states', self.on_joint, 10)
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/lbr_tutorial/status', self.on_status, qos)
        self.create_subscription(JointTrajectoryControllerState,
                                 '/lbr/joint_trajectory_controller/state',
                                 self.on_controller, 10)

    def on_joint(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(n in values for n in JOINT_NAMES):
            self.q = [values[n] for n in JOINT_NAMES]
            self.state_at = time.monotonic()

    def on_status(self, msg):
        event = json.loads(msg.data)
        self.ready_seen = self.ready_seen or event['event'] == 'ready'
        self.events.append(event)

    def on_controller(self, msg):
        if not self.record or len(msg.actual.positions) != 7 or len(msg.desired.positions) != 7:
            return
        actual_xyz, actual_quat = self.kin.fk(msg.actual.positions)
        desired_xyz, desired_quat = self.kin.fk(msg.desired.positions)
        self.samples.append(dict(
            time=time.monotonic(), actual_joints=list(msg.actual.positions),
            desired_joints=list(msg.desired.positions), actual_xyz=actual_xyz,
            desired_xyz=desired_xyz,
            joint_error_rad=max(abs(a-b) for a, b in zip(msg.actual.positions, msg.desired.positions)),
            position_error_m=math.dist(actual_xyz, desired_xyz),
            orientation_error_rad=quaternion_distance(actual_quat, desired_quat),
        ))

    def wait(self, predicate, timeout):
        end = time.monotonic()+timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError('timed out waiting for ROS feedback/result')

    def run(self, mode, timeout):
        self.wait(lambda: self.q is not None and time.monotonic()-self.state_at < 0.5 and
                  self.joint_pub.get_subscription_count() > 0 and
                  self.path_pub.get_subscription_count() > 0 and
                  self.ready_seen, timeout)
        start_q = self.q[:]
        start_xyz, start_quat = self.kin.fk(start_q)
        if mode == 'joint':
            path_vertices = None
            target = start_q[:]
            target[0] += 0.18
            target[3] -= 0.12
            target[5] += 0.08
            msg = JointState(name=list(JOINT_NAMES), position=target)
            publisher = self.joint_pub
            expected_xyz, expected_quat = self.kin.fk(target)
        elif mode == 'cartesian':
            msg = Path()
            msg.header.frame_id = 'lbr_link_0'
            # Closed 3-cm square, chosen relative to measured FK, fixed orientation.
            offsets = [(0, 0.03, 0), (-0.03, 0.03, 0), (-0.03, 0, 0), (0, 0, 0)]
            path_vertices = [start_xyz]
            for offset in offsets:
                xyz = [a+b for a, b in zip(start_xyz, offset)]
                msg.poses.append(stamped_pose(xyz, start_quat))
                path_vertices.append(xyz)
            publisher = self.path_pub
            expected_xyz, expected_quat = start_xyz, start_quat
        else:
            raise ValueError('mode must be joint or cartesian')
        self.events.clear()
        self.samples.clear()
        self.record = True
        publisher.publish(msg)
        self.wait(lambda: any(e['event'] in ('accepted', 'rejected') for e in self.events), timeout)
        first = next(e for e in self.events if e['event'] in ('accepted', 'rejected'))
        request_id = first['request_id']
        self.wait(lambda: any(e['request_id'] == request_id and e['event'] in
                             ('succeeded', 'failed', 'rejected') for e in self.events), timeout)
        terminal = next(e for e in self.events if e['request_id'] == request_id and e['event'] in
                        ('succeeded', 'failed', 'rejected'))
        self.record = False
        # Consume a new state after action completion before computing endpoint error.
        last_state = self.state_at
        self.wait(lambda: self.state_at > last_state, 2.0)
        final_xyz, final_quat = self.kin.fk(self.q)
        report = dict(
            mode=mode, action=ACTION, result=terminal, start_joints=start_q,
            final_joints=self.q, target_xyz=expected_xyz, final_xyz=final_xyz,
            final_joint_error_rad=max(abs(a-b) for a, b in zip(self.q, first.get('target', self.q))),
            final_position_error_m=math.dist(final_xyz, expected_xyz),
            final_orientation_error_rad=quaternion_distance(final_quat, expected_quat),
            sample_count=len(self.samples),
            max_joint_tracking_error_rad=max((s['joint_error_rad'] for s in self.samples), default=None),
            max_position_tracking_error_m=max((s['position_error_m'] for s in self.samples), default=None),
            max_orientation_tracking_error_rad=max((s['orientation_error_rad'] for s in self.samples), default=None),
            rms_position_tracking_error_m=math.sqrt(sum(s['position_error_m']**2 for s in self.samples)
                                                   /len(self.samples)) if self.samples else None,
            events=self.events, samples=self.samples,
        )
        if path_vertices is not None:
            report['path_vertices'] = path_vertices
            report['max_actual_path_deviation_m'] = max(
                (polyline_distance(s['actual_xyz'], path_vertices) for s in self.samples), default=None)
            report['max_desired_path_deviation_m'] = max(
                (polyline_distance(s['desired_xyz'], path_vertices) for s in self.samples), default=None)
        report['passed'] = (terminal['event'] == 'succeeded' and len(self.samples) >= 5 and
                            report['final_position_error_m'] < 0.003 and
                            report['final_joint_error_rad'] < 0.02 and
                            report['final_orientation_error_rad'] < 0.01 and
                            report['max_position_tracking_error_m'] < 0.015)
        if path_vertices is not None:
            report['passed'] = report['passed'] and report['max_actual_path_deviation_m'] < 0.003
        return report

    def destroy_node(self):
        self.kin.close()
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['joint', 'cartesian'], default='joint')
    parser.add_argument('--timeout', type=float, default=60)
    parser.add_argument('--report', type=FilePath)
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = Demo()
    code = 1
    try:
        report = node.run(options.mode, options.timeout)
        if options.report:
            options.report.parent.mkdir(parents=True, exist_ok=True)
            options.report.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(json.dumps({k: v for k, v in report.items() if k not in ('samples', 'events')}, indent=2))
        code = 0 if report['passed'] else 1
    except (RuntimeError, KeyboardInterrupt) as exc:
        print(str(exc))
        if options.report:
            options.report.parent.mkdir(parents=True, exist_ok=True)
            options.report.write_text(json.dumps(dict(mode=options.mode, passed=False,
                                                       error=str(exc), events=node.events), indent=2)+'\n')
        if node.cancel_client.service_is_ready():
            future = node.cancel_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)

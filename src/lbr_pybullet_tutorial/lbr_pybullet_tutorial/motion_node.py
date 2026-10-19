"""Receive ROS positions, solve waypoints and submit to the real JTC action."""

import json
import math
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from .model import JOINT_NAMES, Kinematics
from .planning import cartesian_plan, joint_plan, pose_values


ACTION = '/lbr/joint_trajectory_controller/follow_joint_trajectory'


def duration(seconds):
    from builtin_interfaces.msg import Duration
    total = round(seconds*1e9)
    return Duration(sec=total//10**9, nanosec=total % 10**9)


class MotionNode(Node):
    def __init__(self):
        super().__init__('lbr_tutorial_motion')
        self.declare_parameter('joint_duration', 4.0)
        self.kin = Kinematics()
        self.q = None
        self.state_at = 0.0
        self.controller_at = 0.0
        self.raw_state_at = 0.0
        self.raw_stamp = -1
        self.busy = False
        self.active = None
        self.current_id = None
        self.cancel_requested = False
        self.deadline = 0.0
        self.sent_at = 0.0
        self.fault = None
        self.ready_sent = False
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/lbr_tutorial/status', qos)
        self.create_subscription(JointState, '/lbr/joint_states', self.on_state, 10)
        self.create_subscription(JointState, '/iiwa7/joint_states', self.on_raw_state, 10)
        self.create_subscription(JointTrajectoryControllerState,
                                 '/lbr/joint_trajectory_controller/state',
                                 self.on_controller, 10)
        self.create_subscription(JointState, '/lbr_tutorial/joint_goal', self.on_joint, 10)
        self.create_subscription(PoseStamped, '/lbr_tutorial/pose_goal', self.on_pose, 10)
        self.create_subscription(Path, '/lbr_tutorial/cartesian_path', self.on_path, 10)
        self.create_service(Trigger, '/lbr_tutorial/cancel', self.on_cancel)
        self.create_timer(0.2, self.check_ready)

    def status(self, event, request_id=None, **fields):
        data = dict(event=event, request_id=request_id, **fields)
        self.status_pub.publish(String(data=json.dumps(data, allow_nan=False)))
        self.get_logger().info(json.dumps(data, ensure_ascii=False))

    def on_state(self, msg):
        if len(msg.name) != len(set(msg.name)) or len(msg.position) != len(msg.name):
            return
        values = dict(zip(msg.name, msg.position))
        if all(n in values and math.isfinite(values[n]) for n in JOINT_NAMES):
            self.q = [values[n] for n in JOINT_NAMES]
            self.state_at = time.monotonic()

    def on_controller(self, msg):
        if len(msg.actual.positions) == 7 and len(msg.desired.positions) == 7:
            self.controller_at = time.monotonic()

    def on_raw_state(self, msg):
        # A broadcaster can keep publishing the last hardware sample after a read
        # failure. Observe advancing source timestamps as well as controller state.
        expected = [f'iiwa_joint_{i}' for i in range(1, 8)]
        if (len(msg.name) != len(set(msg.name)) or len(msg.position) != len(msg.name)
                or len(msg.velocity) != len(msg.name)):
            return
        values = dict(zip(msg.name, zip(msg.position, msg.velocity)))
        if not all(n in values and all(map(math.isfinite, values[n])) for n in expected):
            return
        stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        if stamp > self.raw_stamp:
            self.raw_stamp = stamp
            self.raw_state_at = time.monotonic()

    def check_ready(self):
        now = time.monotonic()
        if self.busy and (now > self.deadline or now-self.state_at > 0.75 or
                          now-self.controller_at > 0.75 or
                          now-self.raw_state_at > 0.75 or
                          (self.active is None and now-self.sent_at > 5.0)):
            if self.active is not None:
                self.active.cancel_goal_async()
            self.fault = 'feedback/action timeout; restart tutorial bringup to recover'
            self.status('failed', self.current_id, reason=self.fault)
            self.finish()
        if (not self.ready_sent and self.q is not None and now-self.state_at < 0.5
                and now-self.controller_at < 0.5 and now-self.raw_state_at < 0.5
                and self.client.server_is_ready()):
            self.ready_sent = True
            self.status('ready', action=ACTION)

    @staticmethod
    def check_header(header, allow_empty=False):
        allowed = ('world', 'lbr_link_0', '') if allow_empty else ('world', 'lbr_link_0')
        if header.frame_id not in allowed:
            raise ValueError('frame_id must be world or lbr_link_0 (no implicit TF)')
        if header.stamp.sec or header.stamp.nanosec:
            raise ValueError('scheduled header stamps are unsupported; use zero stamp')

    def execute(self, kind, make_plan):
        request_id = uuid.uuid4().hex
        owns_busy = False
        try:
            if self.fault:
                raise ValueError(self.fault)
            if self.busy:
                raise ValueError('busy: finish or cancel the active trajectory first')
            if self.q is None or time.monotonic()-self.state_at > 0.5:
                raise ValueError('fresh measured joint state unavailable')
            if time.monotonic()-self.raw_state_at > 0.5:
                raise ValueError('fresh raw PyBullet feedback unavailable')
            if not self.client.server_is_ready():
                raise ValueError('trajectory controller action unavailable')
            if time.monotonic()-self.controller_at > 0.5:
                raise ValueError('active trajectory controller feedback unavailable')
            self.busy = True
            owns_busy = True
            plan = make_plan(self.q[:])
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(JOINT_NAMES)
            for q, t in zip(plan.positions, plan.times):
                point = JointTrajectoryPoint(positions=q, time_from_start=duration(t))
                if plan.zero_endpoint_velocity:
                    point.velocities = [0.0]*7
                goal.trajectory.points.append(point)
            goal.goal_time_tolerance = duration(3.0)
            self.current_id = request_id
            self.cancel_requested = False
            self.sent_at = time.monotonic()
            self.deadline = self.sent_at + plan.times[-1] + 8.0
            self.status('accepted', request_id, kind=kind, points=len(plan.times),
                        duration=plan.times[-1], target=plan.positions[-1])
            future = self.client.send_goal_async(goal)
            future.add_done_callback(lambda f: self.on_goal(f, request_id))
        except (ValueError, RuntimeError, OverflowError, TypeError) as exc:
            if owns_busy:
                self.finish()
            self.status('rejected', request_id, kind=kind, reason=str(exc))

    def on_goal(self, future, request_id):
        try:
            if self.current_id != request_id:
                handle = future.result()
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            self.active = future.result()
            if not self.active.accepted:
                raise RuntimeError('controller rejected trajectory')
            if self.cancel_requested:
                self.active.cancel_goal_async()
            self.active.get_result_async().add_done_callback(
                lambda f: self.on_result(f, request_id))
        except Exception as exc:
            self.status('failed', request_id, reason=str(exc))
            self.finish()

    def on_result(self, future, request_id):
        if self.current_id != request_id:
            return
        try:
            result = future.result()
            ok = (result.status == GoalStatus.STATUS_SUCCEEDED and
                  result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL)
            self.status('succeeded' if ok else 'failed', request_id,
                        action_status=result.status, error_code=result.result.error_code,
                        reason=(result.result.error_string or
                                ('trajectory canceled' if result.status == GoalStatus.STATUS_CANCELED else '')))
        except Exception as exc:
            self.status('failed', request_id, reason=str(exc))
        finally:
            self.finish()

    def finish(self):
        self.busy, self.active, self.current_id = False, None, None

    def on_joint(self, msg):
        def make(q):
            if msg.header.stamp.sec or msg.header.stamp.nanosec:
                raise ValueError('joint goal stamp must be zero')
            if len(msg.name) != 7 or set(msg.name) != set(JOINT_NAMES):
                raise ValueError('provide seven unique lbr_A1..lbr_A7 joint names')
            if len(msg.position) != 7 or msg.velocity or msg.effort:
                raise ValueError('provide seven positions only')
            values = dict(zip(msg.name, msg.position))
            return joint_plan(q, [values[n] for n in JOINT_NAMES],
                              float(self.get_parameter('joint_duration').value))
        self.execute('joint', make)

    def on_pose(self, msg):
        def make(q):
            self.check_header(msg.header)
            return cartesian_plan(self.kin, q, [self.values(msg.pose)])
        self.execute('pose', make)

    @staticmethod
    def values(pose):
        return pose_values([pose.position.x, pose.position.y, pose.position.z],
                           [pose.orientation.x, pose.orientation.y,
                            pose.orientation.z, pose.orientation.w])

    def on_path(self, msg):
        def make(q):
            self.check_header(msg.header)
            if not 1 <= len(msg.poses) <= 100:
                raise ValueError('Cartesian path must have 1..100 poses')
            for stamped in msg.poses:
                self.check_header(stamped.header, allow_empty=True)
                if stamped.header.frame_id and stamped.header.frame_id != msg.header.frame_id:
                    raise ValueError('path pose frame must match path frame')
            return cartesian_plan(self.kin, q, [self.values(p.pose) for p in msg.poses])
        self.execute('cartesian_path', make)

    def on_cancel(self, request, response):
        response.success = self.busy
        response.message = 'cancel requested' if self.busy else 'no active trajectory'
        self.cancel_requested = self.busy
        if self.busy:
            self.deadline = min(self.deadline, time.monotonic()+3.0)
        if self.active is not None:
            self.active.cancel_goal_async()
        return response

    def destroy_node(self):
        self.kin.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

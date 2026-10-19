"""Bounded joint and Cartesian waypoint generation; interpolation runs in JTC."""

from dataclasses import dataclass
import math
import time

from .model import Kinematics, validate_joints


@dataclass
class Plan:
    positions: list
    times: list
    zero_endpoint_velocity: bool = False


def pose_values(position, quaternion):
    xyz = [float(x) for x in position]
    quat = [float(x) for x in quaternion]
    if len(xyz) != 3 or len(quat) != 4 or not all(map(math.isfinite, xyz + quat)):
        raise ValueError('pose must contain finite xyz and xyzw')
    if any(abs(x) > 2.0 for x in xyz):
        raise ValueError('Cartesian target is outside the 2 m simulation workspace')
    norm = math.sqrt(sum(x*x for x in quat))
    if abs(norm - 1.0) > 0.001:
        raise ValueError('quaternion must be normalized (xyzw)')
    return xyz, [x/norm for x in quat]


def quaternion_distance(a, b):
    norm = math.sqrt(sum(x*x for x in a)*sum(x*x for x in b))
    return 2.0 * math.acos(min(1.0, abs(sum(x*y for x, y in zip(a, b)))/norm))


def slerp(a, b, u):
    dot = sum(x*y for x, y in zip(a, b))
    if dot < 0:
        b, dot = [-x for x in b], -dot
    if dot > 0.9995:
        q = [(1-u)*x + u*y for x, y in zip(a, b)]
        norm = math.sqrt(sum(x*x for x in q))
        return [x/norm for x in q]
    theta = math.acos(min(1.0, dot))
    return [(math.sin((1-u)*theta)*x + math.sin(u*theta)*y)/math.sin(theta)
            for x, y in zip(a, b)]


def joint_plan(start, goal, duration=4.0, joint_speed=0.35):
    if not math.isfinite(duration) or not 0 < duration <= 120:
        raise ValueError('joint duration must be finite and in (0, 120] seconds')
    if not math.isfinite(joint_speed) or joint_speed <= 0:
        raise ValueError('joint speed must be positive and finite')
    start, goal = validate_joints(start), validate_joints(goal)
    # Cubic endpoint-zero-velocity interpolation has peak speed 1.5*delta/T.
    duration = max(duration, 1.6 * max(abs(a-b) for a, b in zip(start, goal))/joint_speed)
    if duration > 120:
        raise ValueError('retimed trajectory exceeds 120 seconds')
    return Plan([start, goal], [0.0, duration], True)


def cartesian_plan(kin: Kinematics, start, waypoints, linear_speed=0.04,
                   angular_speed=0.20, joint_speed=0.35):
    if not all(math.isfinite(v) and v > 0 for v in (linear_speed, angular_speed, joint_speed)):
        raise ValueError('trajectory speeds must be positive and finite')
    if not 1 <= len(waypoints) <= 100:
        raise ValueError('Cartesian path must have 1..100 poses')
    q = validate_joints(start)
    xyz, quat = kin.fk(q)
    positions, times = [q], [0.0]
    deadline = time.monotonic() + 5.0
    for target_xyz, target_quat in waypoints:
        target_xyz, target_quat = pose_values(target_xyz, target_quat)
        distance = math.dist(xyz, target_xyz)
        angle = quaternion_distance(quat, target_quat)
        count = max(1, math.ceil(distance/0.004), math.ceil(angle/0.03))
        if len(positions) + count > 2000:
            raise ValueError('path exceeds 2000 sampled points')
        begin_xyz, begin_quat = xyz, quat
        for j in range(1, count+1):
            if time.monotonic() > deadline:
                raise ValueError('Cartesian planning exceeded 5 second computation budget')
            u = j/count
            xyz = [(1-u)*a + u*b for a, b in zip(begin_xyz, target_xyz)]
            quat = slerp(begin_quat, target_quat, u)
            next_q = kin.ik(xyz, quat, q)
            delta = max(abs(a-b) for a, b in zip(q, next_q))
            if delta > 0.25:
                raise ValueError('IK branch discontinuity exceeds 0.25 rad per sample')
            dt = max(0.05, distance/count/linear_speed, angle/count/angular_speed,
                     1.1*delta/joint_speed)
            times.append(times[-1] + dt)
            positions.append(next_q)
            q = next_q
    if times[-1] > 120.0:
        raise ValueError('path duration exceeds 120 seconds')
    return Plan(positions, times)

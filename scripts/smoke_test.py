#!/usr/bin/env python3
"""Exercise installed nodes, actual physics, invalid goals, busy and cancellation."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

import rclpy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from lbr_pybullet_tutorial.demo import Demo, stamped_pose
from lbr_pybullet_tutorial.model import JOINT_NAMES


def probe(node, publisher, msg, expected):
    node.events.clear()
    publisher.publish(msg)
    node.wait(lambda: any(e['event'] in ('accepted', 'rejected') for e in node.events), 10)
    first = next(e for e in node.events if e['event'] in ('accepted', 'rejected'))
    if expected == 'succeeded':
        node.wait(lambda: any(e['request_id'] == first['request_id'] and e['event'] in
                             ('succeeded', 'failed', 'rejected') for e in node.events), 15)
        result = next(e for e in node.events if e['request_id'] == first['request_id'] and
                      e['event'] in ('succeeded', 'failed', 'rejected'))
    else:
        result = first
    assert result['event'] == expected, result
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['tutorial', 'external', 'existing'], default='tutorial',
                        help='external requires a compatible simulator already running in --domain')
    parser.add_argument('--domain', type=int, default=89)
    parser.add_argument('--output', type=Path, default=Path('artifacts/smoke'))
    parser.add_argument('--upstream', action='store_true', help='run original 30 s LBR client too')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    process = None
    node = None
    report = {'backend': args.backend, 'domain': args.domain, 'checks': {}}
    with (args.output/'launch.log').open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', 'lbr_pybullet_tutorial', 'tutorial.launch.py',
             f'sim_backend:={args.backend}', 'gui:=false'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            rclpy.init()
            node = Demo()
            for mode in ('joint', 'cartesian'):
                measurement = node.run(mode, 30)
                (args.output/f'{mode}.json').write_text(json.dumps(measurement, indent=2)+'\n')
                assert measurement['passed'], measurement['result']
                report['checks'][mode] = {k: v for k, v in measurement.items()
                                          if k not in ('samples', 'events')}
            report['checks']['duplicate_names'] = probe(
                node, node.joint_pub, JointState(name=['lbr_A1']*7, position=[0.0]*7), 'rejected')
            xyz, quat = node.kin.fk(node.q)
            report['checks']['unreachable_pose'] = probe(
                node, node.pose_pub, stamped_pose([1.8, 1.8, 1.8], quat), 'rejected')
            invalid_frame = stamped_pose(xyz, quat)
            invalid_frame.header.frame_id = 'camera_optical_frame'
            report['checks']['invalid_frame'] = probe(node, node.pose_pub, invalid_frame, 'rejected')
            report['checks']['single_pose_goal'] = probe(
                node, node.pose_pub, stamped_pose([xyz[0]-0.01, xyz[1], xyz[2]], quat), 'succeeded')
            target = node.q[:]
            target[0] += 0.2
            first = probe(node, node.joint_pub,
                          JointState(name=JOINT_NAMES, position=target), 'accepted')
            report['checks']['busy'] = probe(
                node, node.joint_pub, JointState(name=JOINT_NAMES, position=target), 'rejected')
            future = node.cancel_client.call_async(Trigger.Request())
            node.wait(future.done, 3)
            assert future.result().success, future.result()
            node.wait(lambda: any(e['request_id'] == first['request_id'] and
                                 e['event'] == 'failed' for e in node.events), 5)
            cancelled = next(e for e in node.events if e['request_id'] == first['request_id'] and
                             e['event'] == 'failed')
            assert cancelled['action_status'] == 5, cancelled
            report['checks']['cancel'] = cancelled
            if args.upstream:
                upstream = subprocess.run(
                    [sys.executable, str(Path(__file__).with_name('upstream_joint_demo.py')),
                     '--ros-args', '-r', '__ns:=/lbr'],
                    capture_output=True, text=True, timeout=40)
                output = upstream.stdout + upstream.stderr
                (args.output/'upstream_client.log').write_text(output)
                assert upstream.returncode == 0 and output.count('Goal was accepted by server.') == 2, output
                assert 'Failed to execute' not in output and 'rejected' not in output, output
                prior = node.state_at
                node.wait(lambda: node.state_at > prior, 2)
                assert max(abs(q) for q in node.q) < 0.02, node.q
                report['checks']['unmodified_lbr_stack_client'] = {
                    'passed': True, 'accepted_trajectories': 2,
                    'final_joint_max_abs_rad': max(abs(q) for q in node.q)}
            report['passed'] = True
        except Exception as error:
            report['passed'] = False
            report['error'] = repr(error)
            raise
        finally:
            (args.output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()

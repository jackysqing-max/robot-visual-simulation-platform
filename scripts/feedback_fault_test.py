#!/usr/bin/env python3
"""Suspend tutorial physics and verify feedback failure disables motion output.

Run after sourcing the tutorial install. A private ROS domain isolates this test
from other simulations. The simulator is resumed before launch shutdown even if
an assertion fails, so SIGSTOP cannot leave an unkillable launch child behind.
"""

import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from lbr_pybullet_tutorial.demo import Demo
from lbr_pybullet_tutorial.model import JOINT_NAMES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=92)
    parser.add_argument('--output', type=Path, default=Path('artifacts/feedback_fault'))
    args = parser.parse_args()
    if not 0 <= args.domain <= 232:
        parser.error('--domain must be between 0 and 232')
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    log_path = args.output / 'launch.log'
    report = {'domain': args.domain, 'backend': 'tutorial', 'checks': {}, 'passed': False}
    node, process, sim_pid = None, None, None
    commands = []
    stopped = False
    error = None

    with log_path.open('w') as log:
        try:
            process = subprocess.Popen(
                ['ros2', 'launch', 'lbr_pybullet_tutorial', 'tutorial.launch.py',
                 'sim_backend:=tutorial', 'gui:=false'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            node = Demo()
            # Arrival times use a steady clock independently of stopped ROS physics.
            subscription = node.create_subscription(
                Float64MultiArray, '/iiwa7/joint_desired',
                lambda msg: commands.append((time.monotonic(), list(msg.data))), 10)
            node.wait(lambda: node.q is not None and node.ready_seen and
                      time.monotonic() - node.state_at < 0.5 and
                      node.joint_pub.get_subscription_count() > 0 and len(commands) >= 5, 30)
            match = re.search(
                r'\[INFO\]\s+\[sim_node-\d+\]: process started with pid \[(\d+)\]',
                log_path.read_text())
            assert match, 'Could not identify this launch\'s exact simulator PID'
            sim_pid = int(match.group(1))
            command_line = Path(f'/proc/{sim_pid}/cmdline').read_bytes().replace(b'\0', b' ')
            assert b'sim_node' in command_line, command_line.decode(errors='replace')
            report['simulator_pid'] = sim_pid

            start = node.q[:]
            target = start[:]
            target[0] += 0.2
            node.events.clear()
            before_goal = len(commands)
            node.joint_pub.publish(JointState(name=JOINT_NAMES, position=target))
            node.wait(lambda: any(e['event'] in ('accepted', 'rejected')
                                  for e in node.events), 10)
            accepted = next(e for e in node.events if e['event'] in ('accepted', 'rejected'))
            assert accepted['event'] == 'accepted', accepted
            request_id = accepted['request_id']
            # App acceptance precedes the action response. An interpolated command
            # proves the JTC has accepted and started executing the actual goal.
            node.wait(lambda: any(len(q) == 7 and abs(q[0] - start[0]) > 0.002
                                  for _, q in commands[before_goal:]), 5)
            report['checks']['trajectory_executing_before_fault'] = {
                'passed': True, 'request_id': request_id,
                'commands_received': len(commands), 'accepted': accepted}

            os.kill(sim_pid, signal.SIGSTOP)
            stopped = True
            fault_at = time.monotonic()
            count_at_fault = len(commands)
            node.wait(lambda: any(e['request_id'] == request_id and
                                  e['event'] in ('failed', 'succeeded', 'rejected')
                                  for e in node.events), 3.0)
            terminal_at = time.monotonic()
            terminal = next(e for e in node.events if e['request_id'] == request_id and
                            e['event'] in ('failed', 'succeeded', 'rejected'))
            assert terminal['event'] == 'failed' and 'timeout' in terminal.get('reason', ''), terminal
            assert terminal_at - fault_at <= 3.0, 'Feedback failure was not reported within 3 seconds'
            report['checks']['application_feedback_timeout'] = {
                'passed': True, 'failure_latency_s': terminal_at - fault_at, 'terminal': terminal}

            # The hardware allows feedback age <= 0.5 s. Start measuring well
            # beyond that timeout, with time to drain previously queued commands.
            quiet_start, quiet_end = fault_at + 1.0, max(fault_at + 1.8, terminal_at + 0.5)
            while time.monotonic() < quiet_end:
                rclpy.spin_once(node, timeout_sec=0.02)
            late_commands = [(stamp, q) for stamp, q in commands if stamp >= quiet_start]
            assert not late_commands, f'{len(late_commands)} commands continued after feedback timeout'
            report['checks']['commands_stop_after_feedback_loss'] = {
                'passed': True, 'commands_before_fault': count_at_fault,
                'commands_during_timeout_grace': len(commands) - count_at_fault,
                'quiet_window_start_after_fault_s': quiet_start - fault_at,
                'quiet_window_duration_s': quiet_end - quiet_start,
                'commands_in_quiet_window': len(late_commands),
                'last_command_after_fault_s': commands[-1][0] - fault_at}
            log_text = log_path.read_text()
            assert 'PyBullet feedback is stale' in log_text, 'Hardware feedback stale error absent from log'
            report['checks']['hardware_reports_stale_feedback'] = {'passed': True}
            report['passed'] = all(check['passed'] for check in report['checks'].values())
            # Keep the subscription alive through all measurements.
            assert subscription is not None
        except Exception as exc:
            error = exc
            report['error'] = repr(exc)
        finally:
            if stopped and sim_pid is not None:
                try:
                    os.kill(sim_pid, signal.SIGCONT)
                except ProcessLookupError:
                    pass
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            (args.output / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')

    print(json.dumps(report, indent=2))
    if error is not None:
        raise SystemExit(1)


if __name__ == '__main__':
    main()

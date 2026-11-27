#!/usr/bin/env python3
"""Measure native SMB open-loop turns on Edgar, with per-step contacts.

This is a supervised chassis identification experiment, not a planner result.
The original wheel geometry, friction, physics step and controller limits apply.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import statistics
import subprocess
import time

from check_edgar_mapping import stamp_ns, stop_process_tree
from check_fortress_mapping import pose_record, require


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--lateral-slip-compliance', type=float, default=0.)
    parser.add_argument('--shutdown-only', action='store_true',
                        help='Stop under native contact traffic after settling, without a turn experiment')
    args = parser.parse_args()
    if not math.isfinite(args.lateral_slip_compliance) or args.lateral_slip_compliance < 0:
        parser.error('Lateral slip compliance must be finite and nonnegative')
    args.output = args.output.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))
    started = time.monotonic()
    deadline = started + 90.
    result = {'status': 'failed', 'scope': 'Native contact-load shutdown' if args.shutdown_only else
              'Native DART supervised open-loop turn response',
              'autonomous': False, 'turn_duration_sim_s': 0. if args.shutdown_only else 3.,
              'requested_yaw_rate': 0. if args.shutdown_only else .5,
              'contact_sampling': 'Every received native contact message; no decimation',
              'contact_force_semantics': 'Fortress 6.18 may leave normal/depth/wrench arrays empty; empty is unobserved, not zero',
              'lateral_slip_compliance': args.lateral_slip_compliance,
              'phases': {}, 'checks': {}}
    history = {'odometry': [], 'joints': [], 'commands': [], 'limited_commands': []}
    contact_counts = {}
    process = node = log = contacts = None
    log_path = args.output.with_suffix('.log')
    contact_path = args.output.with_name(args.output.stem + '_contacts.jsonl')
    history_path = args.output.with_name(args.output.stem + '_history.json')

    import rclpy
    from controller_manager_msgs.srv import ListControllers
    from geometry_msgs.msg import Twist, TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from ros_gz_interfaces.msg import Contacts
    from rosidl_runtime_py.convert import message_to_ordereddict
    from sensor_msgs.msg import JointState

    class Probe(Node):
        def __init__(self):
            super().__init__('fortress_turn_probe', parameter_overrides=[
                Parameter('use_sim_time', value=True)])
            self.phase = 'startup'
            self.sim_s = 0.
            self.current = None
            self.receipt = 0.
            self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
            self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
            self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
            self.create_subscription(Odometry, 'odometry', self.odom, qos_profile_sensor_data)
            self.create_subscription(JointState, 'joint_states', self.joint, qos_profile_sensor_data)
            self.create_subscription(TwistStamped, 'smb_drive/cmd_vel_out', self.limited, 10)
            for part in ('base', 'lf', 'rf', 'lh', 'rh'):
                self.create_subscription(Contacts, 'smb/contact/' + part,
                    lambda m, p=part: self.contact(m, p), qos_profile_sensor_data)

        def timed(self, data):
            return {'wall_s': time.monotonic()-started, 'sim_s': self.sim_s,
                    'phase': self.phase, **data}

        def clock(self, msg):
            self.sim_s = msg.clock.sec + msg.clock.nanosec*1.e-9

        def odom(self, msg):
            self.current, self.receipt = msg, time.monotonic()
            q, v, w = msg.pose.pose.orientation, msg.twist.twist.linear, msg.twist.twist.angular
            history['odometry'].append(self.timed({'stamp_ns': stamp_ns(msg),
                'pose': pose_record(msg), 'quaternion': [q.x, q.y, q.z, q.w],
                'body_linear': [v.x, v.y, v.z], 'body_angular': [w.x, w.y, w.z]}))

        def joint(self, msg):
            history['joints'].append(self.timed({'stamp_ns': stamp_ns(msg),
                'names': list(msg.name), 'position': list(msg.position),
                'velocity': list(msg.velocity)}))

        def limited(self, msg):
            history['limited_commands'].append(self.timed({'stamp_ns': stamp_ns(msg),
                'linear_x': msg.twist.linear.x, 'angular_z': msg.twist.angular.z}))

        def contact(self, msg, part):
            contact_counts[part] = contact_counts.get(part, 0) + 1
            contacts.write(json.dumps(self.timed({'part': part,
                'native': message_to_ordereddict(msg)}), separators=(',', ':')) + '\n')

        def command(self, v=0., w=0.):
            # A paused/lagging observer never keeps sending nonzero controls.
            if self.current is None or time.monotonic()-self.receipt > .5:
                v = w = 0.
            msg = Twist()
            msg.linear.x, msg.angular.z = v, w
            self.publisher.publish(msg)
            history['commands'].append(self.timed({'linear_x': v, 'angular_z': w}))

        def tick(self):
            require(time.monotonic() < deadline, 'Turn experiment wall deadline')
            require(process.poll() is None, 'Gazebo launch exited during observation')
            rclpy.spin_once(self, timeout_sec=.01)

        def phase_for(self, name, duration, w=0.):
            self.phase = name
            initial_sim = self.sim_s
            initial_pose = pose_record(self.current)
            last_send = 0.
            while self.sim_s-initial_sim < duration:
                self.tick()
                if time.monotonic()-last_send >= .02:
                    self.command(w=w)
                    last_send = time.monotonic()
            self.command()
            samples = [r for r in history['odometry'] if r['phase'] == name
                       and r['sim_s']-initial_sim >= 1.]
            ratios = [r['body_linear'][1]/r['body_angular'][2] for r in samples
                      if abs(r['body_angular'][2]) > .05]
            result['phases'][name] = {'start_sim_s': initial_sim, 'end_sim_s': self.sim_s,
                'start_pose': initial_pose, 'end_pose': pose_record(self.current),
                'translation_xy_m': math.dist(initial_pose[:2], pose_record(self.current)[:2]),
                'steady_body_vx_mps': statistics.median(r['body_linear'][0] for r in samples) if samples else None,
                'steady_body_vy_mps': statistics.median(r['body_linear'][1] for r in samples) if samples else None,
                'steady_body_wz_radps': statistics.median(r['body_angular'][2] for r in samples) if samples else None,
                'steady_vy_over_wz_m': statistics.median(ratios) if ratios else None}

    try:
        log = log_path.open('w')
        contacts = contact_path.open('w')
        process = subprocess.Popen(['ros2', 'launch', 'gbplanner_gazebo', 'edgar_fortress.launch.py',
            'planning:=false', 'mapping:=false', 'contacts:=true',
            f'lateral_slip_compliance:={args.lateral_slip_compliance}'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = Probe()
        while not (node.current and history['joints'] and node.controllers.service_is_ready()):
            node.tick()
            node.command()
        while True:
            future = node.controllers.call_async(ListControllers.Request())
            while not future.done():
                node.tick()
            active = {c.name for c in future.result().controller if c.state == 'active'}
            if {'smb_drive', 'joint_state_broadcaster'} <= active:
                break
            node.tick()
        node.phase_for('settle', 1.5)
        if not args.shutdown_only:
            node.phase_for('positive', 3., .5)
            node.phase_for('stop_between', 1.5)
            node.phase_for('negative', 3., -.5)
            node.phase_for('final_stop', 1.5)
        require(history['limited_commands'], 'Controller limited command topic was not received')
        require(all(contact_counts.get(p, 0) > 0 for p in ('lf', 'rf', 'lh', 'rh')),
                'Missing native contact observations for a wheel')
        require(math.hypot(node.current.twist.twist.linear.x, node.current.twist.twist.linear.y) < .01
                and abs(node.current.twist.twist.angular.z) < .02, 'Chassis did not stop')
        result['checks'].update(limited_commands_received=True, four_wheel_contacts=True, stopped=True)
        result['status'] = 'passed'
    except Exception as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        if node is not None and rclpy.ok():
            node.command()
        if process is not None:
            try:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=7.)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass
        result['cleanup'] = stop_process_tree(process)
        if log:
            log.close()
        if contacts:
            contacts.close()
        failures = re.findall(r'\[([^\]]+)\]: process has died \[pid \d+, exit code (-?\d+)',
                              log_path.read_text() if log_path.exists() else '')
        result['node_exit_failures'] = [{'name': name, 'exit_code': int(code)} for name, code in failures]
        result['checks']['clean_shutdown'] = not failures and not result['cleanup'].get('forced_kill', False)
        if not result['checks']['clean_shutdown']:
            result['status'] = 'failed'
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        result['wall_elapsed_s'] = time.monotonic()-started
        result['contact_messages'] = contact_counts
        result['counts'] = {k: len(v) for k, v in history.items()}
        result['history_file'] = str(history_path)
        result['contacts_file'] = str(contact_path)
        history_path.write_text(json.dumps(history, separators=(',', ':')) + '\n')
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
        print(json.dumps(result, indent=2))
    return 0 if result['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Check project bridge message paths, then drain it while native sensors keep running.

No planning or motion request is issued. Both native launches are exercised
separately: SMB covers the original dual lidar and three cameras; RMF covers
joint states and the controller's zero-speed actuator messages.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import traceback

sys.dont_write_bytecode = True
from check_fortress_smb_sensors import shutdown_launch
from check_fortress_autonomy import shutdown_log_events


def require(value, message):
    if not value:
        raise AssertionError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--platform', choices=('smb', 'rmf'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    log_path = output.with_suffix('.log')
    require(not output.exists() and not log_path.exists(), 'Choose a new output path')
    output.parent.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[1]
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))
    env = dict(os.environ)
    env['IGN_PARTITION'] = 'gbplanner_fortress_' + env['ROS_DOMAIN_ID']
    started = time.monotonic()
    deadline = started + 100.
    summary = {'status': 'failed', 'platform': args.platform, 'scope': __doc__,
               'topics': {}, 'domain': env['ROS_DOMAIN_ID'], 'source_sha256': {}}
    for relative in ('tools/check_native_bridge.py',
                     'src/gbplanner_gazebo/src/managed_bridge_node.cpp'):
        summary['source_sha256'][relative] = hashlib.sha256((root / relative).read_bytes()).hexdigest()
    process = node = log = None
    initialized = False
    try:
        import rclpy
        from actuator_msgs.msg import Actuators
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from ros_gz_interfaces.msg import Contacts
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, PointCloud2

        rclpy.init(args=[])
        initialized = True
        node = Node('native_bridge_probe')
        topics = {'/clock': Clock, '/gazebo/odometry_raw': Odometry}
        if args.platform == 'smb':
            topics.update({'/gazebo/velodyne_points_raw': PointCloud2,
                           '/gazebo/os1_points_raw': PointCloud2, '/smb/os1/imu/raw': Imu})
            for name in ('front', 'left', 'right'):
                topics[f'/rgb_cam_{name}/image_raw'] = Image
                topics[f'/gazebo/rgb_cam_{name}_camera_info_raw'] = CameraInfo
            for part in ('base', 'lf', 'rf', 'lh', 'rh'):
                topics[f'/smb/contact/{part}'] = Contacts
        else:
            topics.update({'/gazebo/points_raw': PointCloud2, '/joint_states': JointState,
                           '/rmf/command/motor_speed': Actuators})
            for name in ('adis', 'ground_truth', 'os0'):
                topics[f'/rmf/imu/{name}/raw'] = Imu
            for part in ('base', 'rotor_0', 'rotor_1', 'rotor_2', 'rotor_3'):
                topics[f'/rmf/contact/{part}'] = Contacts

        def receive(topic, message):
            record = summary['topics'][topic]
            stamp = message.clock if isinstance(message, Clock) else message.header.stamp
            stamp_ns = int(stamp.sec) * 1_000_000_000 + stamp.nanosec
            if stamp_ns == 0:  # Initial simulator time is legitimately zero.
                record['zero_stamp_startup_samples'] = record.get('zero_stamp_startup_samples', 0) + 1
                return
            require(stamp_ns > 0, 'Negative source stamp: ' + topic)
            record['count'] += 1
            record['last_wall_s'] = time.monotonic() - started
            record['last_stamp_ns'] = stamp_ns
            if isinstance(message, (Image, PointCloud2)):
                record['dimensions'] = [message.width, message.height]
                record['bytes'] = len(message.data)
                require(message.width > 0 and message.height > 0 and len(message.data) > 0,
                        'Empty native sensor payload: ' + topic)
            elif isinstance(message, CameraInfo):
                record['dimensions'] = [message.width, message.height]
                require(message.k[0] > 0 and message.k[4] > 0, 'Invalid camera intrinsics')
            elif isinstance(message, Actuators):
                record['velocity'] = list(message.velocity)
                require(len(message.velocity) == 4 and all(x == 0. for x in message.velocity),
                        'Dormant RMF controller should only publish four zero motor speeds')
            elif isinstance(message, JointState):
                record['names'] = list(message.name)
                require(len(message.name) >= 4, 'Missing native rotor joints')
            elif isinstance(message, Imu):
                require(all(math.isfinite(x) for vector in
                            (message.linear_acceleration, message.angular_velocity)
                            for x in (vector.x, vector.y, vector.z)), 'Nonfinite native IMU')

        for topic, message_type in topics.items():
            summary['topics'][topic] = {'count': 0, 'type': message_type.__name__}
            node.create_subscription(message_type, topic, lambda m, t=topic: receive(t, m),
                                     qos_profile_sensor_data)

        def until(predicate, description, limit=None):
            end = min(deadline, limit) if limit is not None else deadline
            while not predicate():
                require(process.poll() is None, 'Native launch exited: ' + description)
                if time.monotonic() > end:
                    raise TimeoutError(description)
                rclpy.spin_once(node, timeout_sec=.01)

        launch = 'edgar_fortress.launch.py' if args.platform == 'smb' else 'rmf_fortress.launch.py'
        command = ['ros2', 'launch', 'gbplanner_gazebo', launch, 'mapping:=false',
                   'planning:=false', 'bridge:=managed', 'contacts:=true']
        if args.platform == 'smb':
            command += ['sensor_profile:=upstream', 'cameras:=true', 'renderer:=project',
                        'renderer_prefix:=' + str(root / 'install_renderer')]
        summary['launch_command'] = command
        # Gazebo Contacts is event-driven: a supported SMB chassis and RMF
        # rotors correctly have no contact message. Require the real supporting
        # wheels/base while retaining every configured channel in the evidence.
        silent_contacts = ({'/smb/contact/base'} if args.platform == 'smb' else
                           {f'/rmf/contact/rotor_{i}' for i in range(4)})
        summary['contact_topics_not_required_to_touch'] = sorted(silent_contacts)
        log = log_path.open('x')
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   env=env, start_new_session=True)
        until(lambda: all(record['count'] >= 3 for topic, record in summary['topics'].items()
                          if topic not in silent_contacts),
              'Every configured native message stream')

        def native_message(topic):
            result = subprocess.run(['ign', 'topic', '-e', '-n', '1', '-t', topic,
                                     '--json-output'], env=env, capture_output=True,
                                    text=True, timeout=6.)
            summary.setdefault('native_echoes', []).append({'topic': topic,
                'returncode': result.returncode, 'stdout': result.stdout, 'stderr': result.stderr})
            require(result.returncode == 0 and result.stdout.strip(), 'Native topic absent: ' + topic)
            # transport11 can dispatch more than one callback before its
            # one-message stop completes. Keep and decode every JSON object.
            remaining = result.stdout.lstrip()
            messages = []
            decoder = json.JSONDecoder()
            while remaining:
                message, end = decoder.raw_decode(remaining)
                messages.append(message)
                remaining = remaining[end:].lstrip()
            return {'topic': topic, 'returncode': result.returncode,
                    'message': messages[0], 'message_count': len(messages)}

        if args.platform == 'rmf':
            native = native_message('/rmf/command/motor_speed')
            require([float(v) for v in native['message'].get('velocity', [])] == [0., 0., 0., 0.],
                    'ROS actuator command did not reach Gazebo transport unchanged')
            summary['native_actuator'] = native
        text = log_path.read_text(errors='replace')
        match = re.search(r'\[(managed_bridge_node-\d+)\]: process started with pid \[(\d+)\]', text)
        require(match is not None, 'Bridge process identity missing from launch log')
        bridge_name, bridge_pid = match[1], int(match[2])
        summary['bridge_pid'] = bridge_pid
        summary['signal'] = 'SIGINT to bridge only; native Gazebo remains running'
        stop_started = time.monotonic()
        os.kill(bridge_pid, signal.SIGINT)
        until(lambda: f'[{bridge_name}]: process has finished cleanly [pid {bridge_pid}]' in
              log_path.read_text(errors='replace'), 'Bridge drained and exited', stop_started + 5.)
        summary['bridge_exit_wall_s'] = time.monotonic() - stop_started
        text = log_path.read_text(errors='replace')
        summary['drained_channels'] = [
            {'direction': direction, 'topic': topic, 'received': int(received),
             'transferred': int(transferred), 'publish_failures': int(failed)}
            for direction, topic, received, transferred, failed in re.findall(
                r'Bridge drained: (GZ->ROS|ROS->GZ) (\S+) received=(\d+) transferred=(\d+) publish_failures=(\d+)', text)]
        require(len(summary['drained_channels']) == len(topics), 'Not all configured bridges reported draining')
        require(all((c['received'] > 0 or c['topic'] in silent_contacts)
                    and c['received'] == c['transferred'] and c['publish_failures'] == 0
                    for c in summary['drained_channels']), 'Incomplete message transfer before drain')
        require('Managed bridge drained; ROS context still valid=true' in text,
                'ROS context closed before the bridge drain barrier')
        raw = '/smb/os1/imu/raw' if args.platform == 'smb' else '/rmf/imu/os0/raw'
        summary['native_sensor_after_bridge_exit'] = native_message(raw)
        summary['functional_status'] = 'passed'
    except Exception as error:
        summary['error'] = str(error)
        summary['traceback'] = traceback.format_exc()
    finally:
        summary['cleanup'] = shutdown_launch(process)
        if log is not None:
            log.close()
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()
        log_text = log_path.read_text(errors='replace') if log_path.exists() else ''
        summary['node_exit_failures'], summary['signal_escalations'] = shutdown_log_events(log_text)
        summary['bridge_shutdown_log'] = [line for line in log_text.splitlines()
                                          if 'managed_bridge_node' in line]
        summary['context_errors'] = [line for line in log_text.splitlines() if any(term in line for term in
                                    ('context is invalid', 'pthread_mutex_lock', 'terminate called', 'Assertion'))]
        cleanup = summary['cleanup']
        if (summary.get('functional_status') == 'passed' and cleanup.get('launch_returncode') == 0
                and not any(cleanup.get(k) for k in ('forced_kill', 'emergency_sigterm', 'process_group_remaining'))
                and not summary['node_exit_failures'] and not summary['signal_escalations']
                and not summary['context_errors']):
            summary['status'] = 'passed'
        summary['wall_elapsed_s'] = time.monotonic() - started
        output.write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps({k: summary.get(k) for k in ('status', 'platform', 'error',
              'bridge_exit_wall_s', 'wall_elapsed_s', 'node_exit_failures', 'signal_escalations')}, indent=2))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

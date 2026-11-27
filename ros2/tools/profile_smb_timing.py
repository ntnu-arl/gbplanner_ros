#!/usr/bin/env python3
"""Observe one stationary SMB/Bullet node for 20 seconds; no planner or mapper.

This collects timing evidence only. Failure to reproduce a delay here does not
establish that the full exploration chain is fixed.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def distribution(values):
    values = sorted(values)
    if not values:
        return {'samples': 0, 'median_s': None, 'p99_s': None, 'max_s': None}
    middle = len(values)//2
    median = values[middle] if len(values) % 2 else .5*(values[middle-1]+values[middle])
    return {'samples': len(values), 'median_s': median,
            'p99_s': values[min(len(values)-1, math.ceil(.99*len(values))-1)],
            'max_s': values[-1]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid() % 80))

    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    started = time.monotonic()
    process, node = None, None
    log_path = args.output.with_suffix('.log')
    odometry_path = args.output.with_suffix('.csv')
    states_path = args.output.with_name(args.output.stem+'_states.json')
    summary = {
        'status': 'failed', 'domain': os.environ['ROS_DOMAIN_ID'],
        'scope': 'one stationary native SMB node with real Bullet physics and lidar; '
                 'no commands, mapper, planner, tracker or robot-state publisher',
        'requested_observation_s': 20.,
        'limitations': [
            'An isolated run does not establish whether full-chain contention is fixed.',
            'Odometry source gaps are between received messages; best-effort queue loss can enlarge them.',
            'Wall minus thread CPU includes descheduling and waits; it does not identify their cause.',
            'Scan counters cover completed calls through the final snapshot; an in-progress stall is not yet counted.',
        ],
    }

    class Probe(Node):
        def __init__(self):
            super().__init__('smb_timing_probe')
            self.phase = 'startup'
            self.rows, self.states = [], []
            self.last_receive, self.last_stamp = None, None
            self.latest = None
            self.create_subscription(Odometry, 'odometry', self.odometry, qos_profile_sensor_data)
            self.create_subscription(String, 'smb_status', self.status, 10)
            self.state = self.create_client(Trigger, 'smb_state')

        def odometry(self, message):
            received = time.monotonic()
            stamp = message.header.stamp.sec+message.header.stamp.nanosec*1.e-9
            ros_now = self.get_clock().now().nanoseconds*1.e-9
            self.rows.append({
                'time_s': received-started, 'phase': self.phase,
                'source_stamp_s': stamp, 'source_age_s': ros_now-stamp,
                'receive_gap_s': received-self.last_receive if self.last_receive is not None else 0.,
                'received_source_stamp_gap_s': stamp-self.last_stamp if self.last_stamp is not None else 0.,
                'ros_wall_minus_monotonic_s': ros_now-received,
            })
            self.last_receive, self.last_stamp = received, stamp

        def status(self, message):
            self.latest = json.loads(message.data)
            self.states.append({'time_s': time.monotonic()-started,
                                'phase': self.phase, 'state': self.latest})

    def spin_until(predicate, seconds):
        end = time.monotonic()+seconds
        while not predicate():
            if process.poll() is not None:
                raise RuntimeError(f'SMB exited {process.returncode}; see {log_path}')
            if time.monotonic() >= end:
                raise TimeoutError('Bounded SMB observation wait expired')
            rclpy.spin_once(node, timeout_sec=.02)

    try:
        rclpy.init(args=[])
        node = Probe()
        config = Path(get_package_share_directory('gbplanner_smb'))/'config/smb.yaml'
        command = ['ros2', 'run', 'gbplanner_smb', 'smb_node', '--ros-args',
                   '--params-file', str(config), '-p', 'use_sim_time:=false']
        summary['command'] = command
        with log_path.open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True, env=dict(os.environ))
            spin_until(lambda: node.latest is not None and bool(node.rows), 6.)
            expected = {'cpu_at_max_scan_s', 'total_scan_wall_s', 'total_scan_cpu_s',
                        'scan_timing_samples', 'scan_cpu_samples'}
            if not expected.issubset(node.latest.get('timing', {})):
                raise RuntimeError('SMB binary does not contain the scan thread-CPU instrumentation')
            summary['initial_state'] = node.latest
            node.phase = 'observation'
            observed_from = time.monotonic()
            observed_until = observed_from+20.
            previous_spin = observed_from
            max_spin_gap = 0.
            while time.monotonic() < observed_until:
                if process.poll() is not None:
                    raise RuntimeError(f'SMB exited {process.returncode}; see {log_path}')
                now = time.monotonic()
                max_spin_gap = max(max_spin_gap, now-previous_spin)
                previous_spin = now
                rclpy.spin_once(node, timeout_sec=.02)
            observed_end = time.monotonic()
            summary['observation_s'] = observed_end-observed_from
            summary['max_probe_spin_gap_s'] = max(max_spin_gap, observed_end-previous_spin)
            summary['end_odometry_receive_age_s'] = observed_end-node.last_receive
            summary['end_status_receive_age_s'] = observed_end-started-node.states[-1]['time_s']
            node.phase = 'final_snapshot'
            final = node.latest
            summary['final_state_source'] = 'last_status'
            if node.state.service_is_ready():
                future = node.state.call_async(Trigger.Request())
                try:
                    spin_until(future.done, 1.5)
                    response = future.result()
                    if not response.success:
                        raise RuntimeError(response.message)
                    final = json.loads(response.message)
                    summary['final_state_source'] = 'smb_state_service'
                except (TimeoutError, RuntimeError) as error:
                    summary['final_state_error'] = str(error)
            else:
                summary['final_state_error'] = 'smb_state_service_unavailable'
            summary['final_state'] = final
            timing = final['timing']
            wall, paired_cpu = timing['max_scan_s'], timing['cpu_at_max_scan_s']
            total_wall, total_cpu = timing['total_scan_wall_s'], timing['total_scan_cpu_s']
            complete_cpu = timing['scan_timing_samples'] == timing['scan_cpu_samples'] > 0
            summary['scan_timing'] = {
                **timing,
                'counter_scope': 'node lifetime through final snapshot',
                'cpu_fraction_at_max_wall_scan': paired_cpu/wall if paired_cpu is not None and wall > 0 else None,
                'wall_minus_cpu_at_max_scan_s': wall-paired_cpu if paired_cpu is not None else None,
                'total_cpu_fraction': total_cpu/total_wall if complete_cpu and total_wall > 0 else None,
                'cpu_samples_complete': complete_cpu,
                'scan_wall_over_0p5s_observed': wall > .5,
            }
            rows = [row for row in node.rows if row['phase'] == 'observation']
            status_times = [item['time_s'] for item in node.states if item['phase'] == 'observation']
            summary['odometry'] = {
                'samples': len(rows),
                'receive_gaps': distribution([row['receive_gap_s'] for row in rows]),
                'received_source_stamp_gaps': distribution([row['received_source_stamp_gap_s'] for row in rows]),
                'source_age_at_receive': distribution([row['source_age_s'] for row in rows]),
                'max_receive_silence_including_tail_s': max(
                    [summary['end_odometry_receive_age_s'], *[row['receive_gap_s'] for row in rows]]),
                'ros_wall_monotonic_offset_span_s': (
                    max(row['ros_wall_minus_monotonic_s'] for row in rows)-
                    min(row['ros_wall_minus_monotonic_s'] for row in rows)) if rows else None,
            }
            summary['status_receive_gaps'] = distribution([b-a for a, b in zip(status_times, status_times[1:])])
            summary['assessment'] = ('Long scan wall time observed; compare its paired thread CPU and probe gaps.'
                                     if wall > .5 else
                                     'The 20-second isolated run did not reproduce a scan wall delay above 0.5 s; '
                                     'the full-chain delay remains unproven as fixed.')
            summary['status'] = 'completed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
    finally:
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=1.)
        if node is not None:
            fields = ['time_s', 'phase', 'source_stamp_s', 'source_age_s', 'receive_gap_s',
                      'received_source_stamp_gap_s', 'ros_wall_minus_monotonic_s']
            with odometry_path.open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerows(node.rows)
            states_path.write_text(json.dumps(node.states, indent=2)+'\n')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        summary['elapsed_s'] = round(time.monotonic()-started, 3)
        summary['processes_stopped'] = process is None or process.poll() is not None
        summary['process_exit_code'] = process.returncode if process is not None else None
        summary['artifacts'] = {'odometry': str(odometry_path), 'states': str(states_path), 'log': str(log_path)}
        args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if summary['status'] != 'completed':
        raise SystemExit(1)


if __name__ == '__main__':
    main()

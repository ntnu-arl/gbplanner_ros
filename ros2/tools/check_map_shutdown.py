#!/usr/bin/env python3
"""Interrupt this tool's planner while replaying a saved native map and graph.

The two runs exercise snapshot parsing and retained-edge revalidation. They do
not run a simulator, publish odometry, or claim a navigation mission passed.
"""
import argparse
from copy import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger


def read_trace(path):
    if not path.exists():
        return []
    rows = []
    for line in path.read_text().splitlines():
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            # The producer flushes whole lines, but a read may overlap write().
            continue
    return rows


def cloud_from(directory, metadata):
    cloud = metadata['cloud']
    raw = (directory / 'map.bin').read_bytes()
    if len(raw) != cloud['row_step'] * cloud['height']:
        raise ValueError('Captured map bytes and metadata disagree')
    message = PointCloud2()
    message.header.frame_id = cloud['frame']
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(cloud['stamp_ns'], 1_000_000_000)
    for name in ('height', 'width', 'point_step', 'row_step', 'is_bigendian', 'is_dense'):
        setattr(message, name, cloud[name])
    message.fields = [PointField(**field) for field in cloud['fields']]
    message.data = raw
    return message, hashlib.sha256(raw).hexdigest()


def wait_until(node, process, predicate, timeout_s, description):
    deadline = time.monotonic() + timeout_s
    while True:
        if process.poll() is not None:
            raise RuntimeError(f'Planner exited while waiting for {description}: {process.returncode}')
        result = predicate()
        if result:
            return result
        if time.monotonic() >= deadline:
            raise TimeoutError(f'Timed out waiting for {description}')
        rclpy.spin_once(node, timeout_sec=.005)


def run_case(args, message, stage):
    marker, completed_marker = {
        'parse': ('parse_begin', 'parsed'),
        'global': ('global_active_edges_begin', 'global_active_edges_end'),
    }[stage]
    trace_path = args.output / f'{stage}.trace.jsonl'
    log_path = args.output / f'{stage}.log'
    command = [str(args.binary), '--ros-args']
    for path in args.params_file:
        command += ['--params-file', str(path)]
    command += ['-p', 'use_sim_time:=false', '-p', 'global.background_enable:=false',
                '-p', 'global.startup_graph:=' + str(args.snapshot_dir / 'graph.bin'),
                '-p', 'diagnostics.map_trace_file:=' + str(trace_path)]
    result = {'stage': stage, 'status': 'failed', 'command': command,
              'expected_phase': marker, 'cleanup_signals': []}
    source_stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
    target_stamp_ns = source_stamp_ns
    started = time.monotonic()
    node = None
    process = None
    with log_path.open('w') as log:
        try:
            node = Node('map_shutdown_' + stage)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            publisher = node.create_publisher(PointCloud2, 'voxel_map', qos)
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 20.
            while publisher.get_subscription_count() == 0:
                if process.poll() is not None:
                    raise RuntimeError(f'Planner exited during startup: {process.returncode}')
                if time.monotonic() >= deadline:
                    raise TimeoutError('Planner subscription did not become ready')
                rclpy.spin_once(node, timeout_sec=.02)
            target_message = message
            if stage == 'global':
                # Loading a graph without a map initially suspends its edges.
                # Let the first real snapshot restore admissible links before
                # testing cancellation of an actual active-edge revalidation.
                publisher.publish(message)
                warmup_completed = wait_until(node, process, lambda: next((row for row in
                    read_trace(trace_path) if row['stamp_ns'] == source_stamp_ns and
                    row['phase'] == 'completed'), None), 60., 'warmup map completion')
                status_client = node.create_client(Trigger, 'global_status')
                wait_until(node, process, status_client.service_is_ready, 10., 'global_status service')
                status_future = status_client.call_async(Trigger.Request())
                wait_until(node, process, status_future.done, 10., 'global_status response')
                reply = status_future.result()
                if reply is None or not reply.success:
                    raise RuntimeError('Warmup global_status was not available')
                global_status = json.loads(reply.message)
                result['warmup'] = {'stamp_ns': source_stamp_ns, 'completed_trace': warmup_completed,
                                    'global_status': global_status}
                if global_status.get('edges', 0) < 1000:
                    raise RuntimeError('Warmup did not restore at least 1000 active graph edges')
                # Share immutable data bytes, but give the test update its own
                # header/stamp so only its trace contributes to the verdict.
                target_message = copy(message)
                target_message.header = copy(message.header)
                target_message.header.stamp = copy(message.header.stamp)
                target_stamp_ns = source_stamp_ns + 1
                target_message.header.stamp.sec, target_message.header.stamp.nanosec = divmod(
                    target_stamp_ns, 1_000_000_000)
            result['target_stamp_ns'] = target_stamp_ns
            publisher.publish(target_message)
            deadline = time.monotonic() + 60.
            while True:
                if process.poll() is not None:
                    raise RuntimeError(f'Planner exited before {marker}: {process.returncode}')
                rows = [row for row in read_trace(trace_path) if row['stamp_ns'] == target_stamp_ns]
                matched = [row for row in rows if row['phase'] == marker]
                if matched:
                    if any(row['phase'] == completed_marker for row in rows):
                        raise RuntimeError(f'Missed active {stage} window; no SIGINT result claimed')
                    result['observed_phase'] = matched[-1]
                    result['signal_steady_s'] = time.monotonic()
                    process.send_signal(signal.SIGINT)
                    break
                if time.monotonic() >= deadline:
                    raise TimeoutError(f'Planner never reached {marker}')
                rclpy.spin_once(node, timeout_sec=.005)
            process.wait(timeout=5.)
            result['exit_after_signal_s'] = time.monotonic() - result['signal_steady_s']
            result['exit_code'] = process.returncode
            rows = [row for row in read_trace(trace_path) if row['stamp_ns'] == target_stamp_ns]
            signal_time = result['signal_steady_s']
            phase_was_active = not any(row['phase'] == completed_marker and
                                       row['steady_s'] <= signal_time for row in rows)
            interrupted = [row for row in rows if row['phase'] == 'shutdown_interrupted']
            log.flush()
            log_text = log_path.read_text()
            result['checks'] = {
                'signal_during_expected_phase': phase_was_active,
                'cooperative_shutdown_recorded': bool(interrupted),
                'no_map_ack_or_acceptance': not any(row['phase'] in
                    ('ack_published', 'accepted_published', 'completed') for row in rows),
                'not_counted_as_rejected_input': not any(row['phase'] == 'rejected' for row in rows)
                    and 'Map rejected:' not in log_text,
                'no_invalid_context_log': 'context is invalid' not in log_text,
                'exit_zero_before_launch_escalation': process.returncode == 0 and
                    result['exit_after_signal_s'] < 5.,
            }
            result['interrupt_trace'] = interrupted
            result['status'] = 'passed' if all(result['checks'].values()) else 'failed'
        except Exception as error:
            result['error'] = f'{type(error).__name__}: {error}'
        finally:
            # Only this run's child is cleaned up; no external process lookup.
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGTERM)
                result['cleanup_signals'].append('SIGTERM')
                try:
                    process.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    result['cleanup_signals'].append('SIGKILL')
                    process.wait(timeout=3.)
            if process is not None:
                result['exit_code'] = process.returncode
            if node is not None:
                node.destroy_node()
    result['elapsed_s'] = time.monotonic() - started
    (args.output / f'{stage}.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot-dir', type=Path, required=True,
                        help='Existing capture containing snapshot.json, map.bin and graph.bin')
    parser.add_argument('--output', type=Path, required=True,
                        help='New directory for the selected bounded shutdown checks')
    parser.add_argument('--stage', choices=('parse', 'global', 'all'), default='all')
    parser.add_argument('--binary', type=Path,
                        default=Path('install_fortress/gbplanner_local/lib/gbplanner_local/local_planner_node'))
    parser.add_argument('--params-file', type=Path, action='append',
                        help='Ordered planner parameter files; default: SMB production profiles')
    args = parser.parse_args()
    args.snapshot_dir = args.snapshot_dir.resolve()
    args.binary = args.binary.resolve()
    args.output = args.output.resolve()
    args.params_file = [path.resolve() for path in (args.params_file or [
        Path('src/gbplanner_smb/config/edgar_exploration.yaml'),
        Path('src/gbplanner_smb/config/upstream_planning.yaml'),
        Path('src/gbplanner_smb/config/upstream_sensors.yaml')])]
    if args.output.exists():
        parser.error('Refusing to overwrite an existing check directory')
    for path in [args.binary, args.snapshot_dir / 'snapshot.json',
                 args.snapshot_dir / 'graph.bin', args.snapshot_dir / 'map.bin', *args.params_file]:
        if not path.is_file():
            parser.error(f'Missing input: {path}')
    metadata = json.loads((args.snapshot_dir / 'snapshot.json').read_text())
    message, map_sha = cloud_from(args.snapshot_dir, metadata)
    if message.width * message.height < 1_000_000 or metadata['graph_active_edges'] < 1000:
        parser.error('This check needs a saved large native map and a nontrivial retained graph')
    os.environ.setdefault('ROS_DOMAIN_ID', '218')
    args.output.mkdir(parents=True)
    stages = ('parse', 'global') if args.stage == 'all' else (args.stage,)
    report = {'status': 'failed', 'scope': 'Selected isolated native ROS planner shutdowns, no simulation',
              'requested_stages': list(stages),
              'source_snapshot': str(args.snapshot_dir), 'map_sha256': map_sha,
              'snapshot_metadata_sha256': hashlib.sha256((args.snapshot_dir / 'snapshot.json').read_bytes()).hexdigest(),
              'graph_sha256': hashlib.sha256((args.snapshot_dir / 'graph.bin').read_bytes()).hexdigest(),
              'binary_sha256': hashlib.sha256(args.binary.read_bytes()).hexdigest(),
              'domain': os.environ['ROS_DOMAIN_ID'], 'cases': []}
    rclpy.init()
    try:
        for stage in stages:
            result = run_case(args, message, stage)
            report['cases'].append(result)
            print(json.dumps({'stage': stage, 'status': result['status'],
                              'exit_after_signal_s': result.get('exit_after_signal_s'),
                              'error': result.get('error')}), flush=True)
        report['status'] = 'passed' if all(case['status'] == 'passed' for case in report['cases']) else 'failed'
    finally:
        rclpy.shutdown()
        (args.output / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

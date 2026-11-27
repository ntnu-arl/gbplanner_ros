#!/usr/bin/env python3
"""One bounded local request on a saved native map; no simulation or retries."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry
from planner_msgs.msg import Graph
from planner_msgs.srv import PlannerSrv
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--root', type=float, nargs=4, required=True, metavar=('X', 'Y', 'Z', 'YAW'),
                        help='Measured base pose, before the planner ground-height conversion')
    parser.add_argument('--live-global', action='store_true',
                        help='Use measured odometry and a zero-root request, including absorption into a fresh global graph')
    parser.add_argument('--seed', type=int, default=43,
                        help='Fixed seed for this single local build (default: 43)')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--library-dir', type=Path,
                        help='Optional retained libgbplanner_local.so directory for a same-node baseline')
    parser.add_argument('--binary', type=Path,
                        help='Optional retained planner executable matching the baseline library ABI')
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in args.root):
        parser.error('Root must be finite')
    if not 0 <= args.seed <= 2**32-1:
        parser.error('Seed must be a uint32 value')
    args.output = args.output.resolve()
    if args.output.exists():
        parser.error('Use a new output file')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    original = json.loads(args.run.read_text())['configuration']['effective_planner_parameters']
    snapshot_metadata = json.loads(args.metadata.read_text())
    metadata = snapshot_metadata['accepted_map']
    data = args.snapshot.read_bytes()
    sha = hashlib.sha256(data).hexdigest()
    if sha != metadata['sha256'] or len(data) != metadata['row_step']*metadata['height']:
        parser.error('Captured raw map does not match metadata SHA256/size')
    overrides = {'use_sim_time': False, 'global.enable': args.live_global, 'global.background_enable': False,
                 'sampling.seed': args.seed, 'sampling.seed_mode': 'fixed',
                 'diagnostics.map_trace_file': str(args.output.with_suffix('.map_trace.jsonl'))}
    if args.live_global:
        overrides['global.auto_home'] = True
    parameters = {**original, **overrides}
    config = args.output.with_suffix('.yaml')
    config.write_text(yaml.safe_dump({'gbplanner_local': {'ros__parameters': parameters}}, sort_keys=False))
    root = Path(__file__).resolve().parents[1]
    binary = root/'install_fortress/gbplanner_local/lib/gbplanner_local/local_planner_node'
    if args.binary:
        binary = args.binary.resolve()
        if not binary.is_file():
            parser.error('The requested planner executable is missing')
    child_environment = os.environ.copy()
    library = root/'install_fortress/gbplanner_local/lib/libgbplanner_local.so'
    if args.library_dir:
        library = args.library_dir.resolve()/'libgbplanner_local.so'
        if not library.is_file():
            parser.error('The requested baseline library is missing')
        child_environment['LD_LIBRARY_PATH'] = str(library.parent)+os.pathsep+os.environ.get('LD_LIBRARY_PATH', '')
    os.environ.setdefault('ROS_DOMAIN_ID', '148')
    scope = ('One frozen-map local request with measured base odometry, a zero-root request, and fresh global memory. '
             'Includes local graph absorption; does not restore the original global graph, visit history, or exploration-direction history. '
             'Not an exact replay of the original request or its map/odometry scheduling.' if args.live_global else
             'One frozen-map local-core diagnostic with a hypothetical base root. '
             'Not an exact replay of the original request scheduling, background work, or mission history.')
    report = {'status': 'failed', 'scope': scope, 'live_global': args.live_global,
              'input_run': str(args.run), 'snapshot': str(args.snapshot), 'snapshot_sha256': sha,
              'source_snapshot_metadata': metadata, 'parameter_overrides': overrides,
              'source_odometry_metadata': snapshot_metadata.get('odometry'),
              'recorded_original_parameters': original, 'root_base_pose_xyzyaw': args.root,
              'root_convention': 'Base pose; the production planner performs its own ground-height conversion and projection.',
              'bound_mode': 3, 'seed': args.seed, 'planning_call_limit_s': 90., 'domain': os.environ['ROS_DOMAIN_ID'],
              'seed_derivation': f'This isolated process deliberately uses fixed seed {args.seed}, request_index 0. '
                                  'The original sequence advances only on actual local graph builds; scheduling and previous global state are not replayed.',
              'sampling_status': [], 'planning_metrics': [], 'local_graphs': [], 'global_graphs': [], 'process_samples': [],
              'library': str(library), 'library_sha256': hashlib.sha256(library.read_bytes()).hexdigest(),
              'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest()}
    process = node = None
    started = time.monotonic()
    log_path = args.output.with_suffix('.log')
    log = log_path.open('w')
    try:
        command = [str(binary), '--ros-args', '--params-file', str(config)]
        report['command'] = command
        child_environment['ROS_DOMAIN_ID'] = os.environ['ROS_DOMAIN_ID']
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                                   env=child_environment)
        report['planner_pid'] = process.pid
        rclpy.init()
        node = Node('frozen_local_probe')
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = node.create_publisher(PointCloud2, 'voxel_map', durable)
        odometry_publisher = node.create_publisher(Odometry, 'odometry', 10) if args.live_global else None
        for topic in ('sampling_status', 'planning_metrics'):
            node.create_subscription(String, topic,
                lambda msg, topic=topic: report[topic].append({'elapsed_s': time.monotonic()-started,
                                                              **json.loads(msg.data)}), durable)
        def capture_graph(message, topic):
            index = len(report[topic+'s'])
            graph_path = args.output.with_suffix(f'.{topic}{index}.json')
            graph_text = json.dumps(message_to_ordereddict(message), indent=2, allow_nan=False)+'\n'
            graph_path.write_text(graph_text)
            report[topic+'s'].append({'topic': topic,
                'source': f'Actual planner {topic} topic publication',
                'elapsed_s': time.monotonic()-started, 'vertices': len(message.vertices),
                'edges': len(message.edges), 'artifact': str(graph_path),
                'sha256': hashlib.sha256(graph_text.encode()).hexdigest()})
        node.create_subscription(Graph, 'local_graph', lambda message: capture_graph(message, 'local_graph'), durable)
        if args.live_global:
            node.create_subscription(Graph, 'global_graph', lambda message: capture_graph(message, 'global_graph'), durable)
        status = node.create_client(Trigger, 'map_status')
        global_status = node.create_client(Trigger, 'global_status') if args.live_global else None
        local = node.create_client(PlannerSrv, 'gbplanner')
        parameter_client = node.create_client(GetParameters, 'gbplanner_local/get_parameters')
        def until(predicate, timeout=15.):
            deadline = time.monotonic()+timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
                value = predicate()
                if value:
                    return value
                if process.poll() is not None:
                    raise RuntimeError(f'Planner exited {process.returncode}')
            raise TimeoutError('ROS readiness/input timeout')
        def call(client, request, timeout=15.):
            future = client.call_async(request);until(future.done, timeout);return future.result()
        until(lambda: status.service_is_ready() and local.service_is_ready() and
              parameter_client.service_is_ready() and publisher.get_subscription_count()>0 and
              (not args.live_global or (global_status.service_is_ready() and
                                       odometry_publisher.get_subscription_count()>0)))
        request = GetParameters.Request();request.names = list(parameters)
        readback = call(parameter_client, request)
        effective = {name: parameter_value_to_python(value)
                     for name, value in zip(request.names, readback.values)}
        report['effective_parameters'] = effective
        if effective != parameters:
            raise AssertionError('Planner parameter readback differs from the prepared frozen request')
        message = PointCloud2()
        message.header.frame_id = metadata['frame_id']
        message.header.stamp.sec = metadata['stamp']['sec']
        message.header.stamp.nanosec = metadata['stamp']['nanosec']
        for name in ('width','height','point_step','row_step','is_bigendian','is_dense'):
            setattr(message, name, metadata[name])
        message.fields = [PointField(**field) for field in metadata['fields']]
        message.data = data
        publisher.publish(message)
        def loaded():
            result = json.loads(call(status, Trigger.Request(), 20.).message)
            return result if result['map_revision'] == 1 else None
        report['map_status'] = until(loaded, 30.)
        if report['map_status']['known_cells'] != metadata['width']:
            raise AssertionError('Captured native map did not load in full')
        if args.live_global:
            odometry = Odometry()
            odometry.header.frame_id = metadata['frame_id']
            odometry.header.stamp = node.get_clock().now().to_msg()
            odometry.child_frame_id = 'base_link'
            pose = odometry.pose.pose
            pose.position.x,pose.position.y,pose.position.z = args.root[:3]
            pose.orientation.z = math.sin(args.root[3]/2)
            pose.orientation.w = math.cos(args.root[3]/2)
            report['published_odometry'] = message_to_ordereddict(odometry)
            odometry_publisher.publish(odometry)
            def home_ready():
                response = call(global_status, Trigger.Request())
                result = json.loads(response.message)
                return result if response.success and result['vertices'] > 0 else None
            report['initial_global_status'] = until(home_ready)
        request = PlannerSrv.Request()
        request.header.frame_id = metadata['frame_id'];request.header.stamp = node.get_clock().now().to_msg()
        request.bound_mode = 3
        if args.live_global:
            # geometry_msgs/Quaternion defaults w to 1 in this ROS install.
            # The production service recognizes odometry only for all seven
            # pose scalars exactly zero, including quaternion w.
            pose = request.root_pose
            pose.position.x = pose.position.y = pose.position.z = 0.
            pose.orientation.x = pose.orientation.y = pose.orientation.z = pose.orientation.w = 0.
            if any((pose.position.x,pose.position.y,pose.position.z,pose.orientation.x,
                    pose.orientation.y,pose.orientation.z,pose.orientation.w)):
                raise AssertionError('Live-global request must use an all-zero root pose')
            report['zero_root_protocol_verified'] = True
        else:
            request.root_pose.position.x,request.root_pose.position.y,request.root_pose.position.z = args.root[:3]
            request.root_pose.orientation.z = math.sin(args.root[3]/2)
            request.root_pose.orientation.w = math.cos(args.root[3]/2)
        report['request'] = message_to_ordereddict(request)
        call_started = time.monotonic()
        future = local.call_async(request)
        report['local_call_started_elapsed_s'] = call_started-started
        next_sample = call_started
        while not future.done() and time.monotonic()-call_started < 90.:
            rclpy.spin_once(node, timeout_sec=.05)
            if time.monotonic() >= next_sample:
                sample = {'call_elapsed_s': time.monotonic()-call_started}
                for name in ('stat','status','wchan'):
                    try:
                        sample[name] = Path(f'/proc/{process.pid}/{name}').read_text()
                    except OSError as error:
                        sample[name] = str(error)
                report['process_samples'].append(sample)
                next_sample = time.monotonic()+15.
            if process.poll() is not None:
                raise RuntimeError(f'Planner exited {process.returncode} during local call')
        report['local_call_elapsed_s'] = time.monotonic()-call_started
        if future.done():
            response = future.result()
            report['response'] = {'status': response.status, 'bound_mode': response.planning_bound_mode,
                'path': [[p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,
                          p.orientation.z,p.orientation.w] for p in response.path]}
            report['status'] = 'returned'
            end = time.monotonic()+.5
            while time.monotonic()<end:
                rclpy.spin_once(node, timeout_sec=.02)
        else:
            report['status'] = 'planning_call_timeout'
            report['known_phase'] = ('Local build seed was published; no completed planning_metrics. Consult the saved phase trace for the last completed boundary.'
                                    if report['sampling_status'] else
                                    'Request was sent, but sampling_status was not received; local-core entry is unproven.')
            debugger = shutil.which('gdb')
            if debugger:
                try:
                    stack = subprocess.run([debugger, '--batch', '-nx', '-p', str(process.pid),
                        '-ex', 'set pagination off', '-ex', 'thread apply all bt', '-ex', 'detach'],
                        capture_output=True, text=True, timeout=10.)
                    stack_path = args.output.with_suffix('.backtrace.txt')
                    stack_path.write_text(stack.stdout+'\n'+stack.stderr)
                    report['backtrace'] = {'artifact': str(stack_path), 'exit_code': stack.returncode}
                except subprocess.TimeoutExpired:
                    report['backtrace'] = {'error': 'gdb exceeded its 10 s bound'}
            else:
                report['backtrace'] = {'available': False, 'reason': 'gdb is not installed; no alternate privileged tracing was attempted'}
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        forced = False
        if process is not None:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    forced = True;os.killpg(process.pid, signal.SIGKILL);process.wait(timeout=2.)
            report['planner_exit_code'] = process.returncode
            report['forced_kill'] = forced
        log.close()
        report['log'] = str(log_path)
        report['elapsed_s'] = time.monotonic()-started
        report['stage_timing_log_lines'] = [line for line in log_path.read_text().splitlines()
                                            if 'planner_request' in line or 'planner_phases' in line]
        trace_path = args.output.with_suffix('.map_trace.jsonl')
        if trace_path.exists():
            lines = trace_path.read_text().splitlines()
            report['phase_trace'] = {'artifact': str(trace_path), 'last_lines': lines[-20:]}
            if args.live_global:
                report['live_global_absorption_entered'] = any(
                    '"phase":"local_absorb_begin"' in line for line in lines)
        if args.live_global and report['status'] == 'returned':
            metrics = report['planning_metrics']
            if (not report.get('live_global_absorption_entered') or not metrics or
                    metrics[-1].get('auto_global', {}).get('reason') == 'hypothetical_root'):
                report['status'] = 'failed'
                report['error'] = 'Live-global call returned without verified odometry-root graph absorption'
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({key:report.get(key) for key in ('status','error','local_call_elapsed_s','elapsed_s','planner_exit_code','forced_kill','known_phase')}))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Check periodic ESDF service/product semantics with real ROS and synthetic scans.

One isolated mapper, a controlled ROS clock, and identity map/sensor coordinates;
no simulator or physical controller is launched.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
import traceback


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=227)
    parser.add_argument('--wall-budget', type=float, default=25.)
    parser.add_argument('--check-mesh-cadence', action='store_true',
                        help='Also check that wall-clock meshing does not republish periodic ESDF products')
    args = parser.parse_args()
    if not 0 <= args.domain <= 232 or not 5. <= args.wall_budget <= 60.:
        parser.error('domain must be 0..232 and wall budget 5..60 seconds')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    args.output = args.output.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    artifacts = (args.output, args.output.with_suffix('.vxblx'),
                 args.output.with_name(args.output.stem + '_mapper.log'))
    trace_path = args.output.with_name(args.output.stem + '_mapper.jsonl')
    if args.check_mesh_cadence:
        artifacts += (trace_path,)
    if any(path.exists() for path in artifacts):
        parser.error('output artifacts already exist; choose a new name to preserve previous evidence')
    root = Path(__file__).resolve().parents[1]
    binary = root / 'install_fortress/gbplanner_voxblox/lib/gbplanner_voxblox/voxblox_mapper_node'
    prefixes = os.environ.get('AMENT_PREFIX_PATH', '').split(':')
    if any(str(root / 'install') == p or p.startswith(str(root / 'install') + '/') for p in prefixes):
        raise RuntimeError('Old install overlay is present; source only ROS and install_fortress/local_setup.bash')

    import rclpy
    from gbplanner_voxblox.srv import FilePath
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import PointCloud2, PointField
    from std_srvs.srv import Trigger

    started = time.monotonic()
    deadline = started + args.wall_budget
    report = {'status': 'failed', 'scope': __doc__, 'domain': args.domain,
              'wall_budget_s': args.wall_budget, 'binary': str(binary),
              'ament_prefix_path': prefixes, 'checks': {}, 'stages': {}, 'events': []}
    for label, path in {'binary': binary, 'test': Path(__file__),
                        'mapper': root / 'src/gbplanner_voxblox/src/voxblox_mapper_node.cpp'}.items():
        report.setdefault('sha256', {})[label] = hashlib.sha256(path.read_bytes()).hexdigest()
    products = ('esdf_voxels', 'esdf_pointcloud', 'esdf_slice', 'traversable')
    received = {topic: [] for topic in products}
    late_received = {topic: [] for topic in products}
    process = node = log = None
    sim_ns = 0
    phase = 'startup'
    namespace = '/periodic_esdf/'

    def check(name, condition):
        report['checks'][name] = bool(condition)
        if not condition:
            raise AssertionError(name)

    try:
        rclpy.init(args=[])
        node = rclpy.create_node('periodic_esdf_probe')
        durable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cloud_pub = node.create_publisher(PointCloud2, namespace + 'points', 10)
        clock_pub = node.create_publisher(Clock, '/clock', 10)
        service_names = ['mapper_status', 'reset_map']
        if args.check_mesh_cadence:
            service_names.append('publish_pointclouds')
            report['mesh_cadence'] = {'trace_path': str(trace_path), 'windows': {}}
        services = {name: node.create_client(Trigger, namespace + name) for name in service_names}
        files = {name: node.create_client(FilePath, namespace + name)
                 for name in ('save_map', 'load_map')}

        def consume(topic, message, late=False):
            payload = bytes(message.data)
            if message.point_step != 16 or message.row_step != 16 * message.width:
                raise AssertionError(topic + ' malformed point layout')
            if len(payload) != message.height * message.row_step:
                raise AssertionError(topic + ' malformed payload size')
            points = sorted(struct.iter_unpack('<ffff', payload))
            if not all(math.isfinite(value) for point in points for value in point):
                raise AssertionError(topic + ' nonfinite product')
            # Hash sorted numeric records so unordered block traversal cannot
            # make a correct save/load round trip look different.
            canonical = b''.join(struct.pack('<ffff', *point) for point in points)
            item = {'stamp_ns': message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
                    'frame': message.header.frame_id, 'points': len(points),
                    'sha256_sorted_points': hashlib.sha256(canonical).hexdigest()}
            (late_received if late else received)[topic].append(item)
            event = {'phase': phase, 'topic': topic, **item}
            if late:
                event['reader'] = 'late'
            report['events'].append(event)

        subscriptions = [node.create_subscription(PointCloud2, namespace + topic,
                         lambda message, topic=topic: consume(topic, message), durable)
                         for topic in products]

        def pump():
            if time.monotonic() >= deadline:
                raise TimeoutError('periodic ESDF protocol wall budget exhausted in ' + phase)
            if process is not None and process.poll() is not None:
                raise RuntimeError('mapper exited unexpectedly: ' + str(process.returncode))
            clock = Clock()
            clock.clock.sec, clock.clock.nanosec = divmod(sim_ns, 10**9)
            clock_pub.publish(clock)
            rclpy.spin_once(node, timeout_sec=.01)

        def wait(predicate, label):
            stop = min(deadline, time.monotonic() + 5.)
            while True:
                pump()
                value = predicate()
                if value:
                    return value
                if time.monotonic() >= stop:
                    raise TimeoutError(label)

        def call(client, request):
            future = client.call_async(request)
            wait(future.done, 'response from ' + client.srv_name)
            response = future.result()
            if response is None:
                raise RuntimeError('missing response from ' + client.srv_name)
            return response

        def status():
            return json.loads(call(services['mapper_status'], Trigger.Request()).message)

        def stage(name):
            result = status()
            report['stages'][name] = {'status': result, 'ros_clock_ns': sim_ns,
                                     'products': {key: values[-1] if values else None
                                                  for key, values in received.items()}}
            return result

        def product_counts():
            return {key: len(values) for key, values in received.items()}

        def traces():
            if not trace_path.exists():
                return []
            # Only complete flushed lines: the mapper may be writing the next
            # record while the probe reads this diagnostic file.
            return [json.loads(line) for line in trace_path.read_text().split('\n')[:-1] if line]

        def pump_for(seconds):
            stop = time.monotonic() + seconds
            while time.monotonic() < stop:
                pump()

        def mesh_window(name, expected_scans, before_counts, before_products, trace_start):
            wait(lambda: any(item['phase'] == 'mesh_generate_end' and
                             item['integrated_scans'] == expected_scans
                             for item in traces()[trace_start:]), name + ' wall-clock mesh completion')
            # Allow queued DDS messages and another mesh timer tick to arrive
            # while the controlled ROS clock remains below its ESDF deadline.
            pump_for(.12)
            events = traces()[trace_start:]
            current = stage(name)
            report['mesh_cadence']['windows'][name] = {
                'product_counts_before': before_counts, 'product_counts_after': product_counts(),
                'trace_start': trace_start, 'trace_end': trace_start + len(events),
                'mesh_completions': sum(item['phase'] == 'mesh_generate_end' for item in events)}
            check(name + '_mesh_ran_without_esdf_generation',
                  current['mesh_updates'] > 0 and current['esdf_pending'] and sim_ns < 10**9 and
                  not any(item['phase'] in ('esdf_products_begin', 'esdf_voxels_generate_begin',
                                            'esdf_update_begin') for item in events))
            check(name + '_esdf_messages_and_values_unchanged', product_counts() == before_counts and
                  all((received[key][-1] if received[key] else None) == before_products[key]
                      for key in products))

        def updated_products(before, stamp, empty=False):
            wait(lambda: all(len(received[key]) > before[key] and
                            received[key][-1]['stamp_ns'] == stamp for key in products),
                 phase + ' publishes every ESDF product with expected stamp')
            check(phase + '_all_products_' + ('empty' if empty else 'valid'),
                  all(received[key][-1]['frame'] == 'map' and
                      (received[key][-1]['points'] == 0 if empty else received[key][-1]['points'] > 0)
                      for key in products))

        def scan(stamp, endpoint, expected_scans):
            nonlocal sim_ns
            sim_ns = stamp
            pump()
            cloud = PointCloud2()
            cloud.header.frame_id = 'map'
            cloud.header.stamp.sec, cloud.header.stamp.nanosec = divmod(stamp, 10**9)
            cloud.height, cloud.width, cloud.point_step, cloud.row_step = 1, 1, 12, 12
            cloud.is_dense = True
            cloud.fields = [PointField(name=axis, offset=4 * index,
                                       datatype=PointField.FLOAT32, count=1)
                            for index, axis in enumerate(('x', 'y', 'z'))]
            cloud.data = struct.pack('<fff', *endpoint)
            cloud_pub.publish(cloud)
            wait(lambda: status()['integrated_scans'] == expected_scans, 'scan integration')

        with tempfile.TemporaryDirectory(prefix='periodic-esdf-') as temporary:
            params = {'use_sim_time': True, 'frame_id': 'map', 'sensor_frame': 'map',
                      'input_sensors': ['main'], 'input.main.topic': 'points', 'input.main.frame': 'map',
                      'pointcloud_reliability': 'reliable', 'use_tf_transforms': True,
                      'map.storage': 'sparse', 'map.resolution': .25,
                      'tsdf.voxels_per_side': 4, 'tsdf.method': 'simple', 'tsdf.threads': 1,
                      'tsdf.truncation_distance': .5, 'esdf.enabled': True,
                      'update_esdf_every_n_sec': 1., 'publish_map_every_n_sec': 0.,
                      'update_mesh_every_n_sec': 0., 'publish_pointclouds': True,
                      'publish_pointclouds_on_update': False, 'publish_slices': True,
                      'slice_level': .125, 'publish_traversable': True, 'traversability_radius': .2}
            if args.check_mesh_cadence:
                params['update_mesh_every_n_sec'] = .1
                params['diagnostics.snapshot_trace_file'] = str(trace_path)
            param_path = Path(temporary) / 'mapper.yaml'
            param_path.write_text(json.dumps({'/periodic_esdf/voxblox_mapper': {'ros__parameters': params}}))
            report['parameters'] = params
            log = args.output.with_name(args.output.stem + '_mapper.log').open('w')
            process = subprocess.Popen([str(binary), '--ros-args', '-r', '__ns:=/periodic_esdf',
                                        '--params-file', str(param_path)], stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            report['mapper_pid'] = process.pid
            wait(lambda: all(client.service_is_ready() for client in (*services.values(), *files.values())),
                 'service discovery')
            wait(lambda: cloud_pub.get_subscription_count() == 1 and clock_pub.get_subscription_count() > 0 and
                 all(node.count_publishers(sub.topic_name) == 1 for sub in subscriptions), 'topic discovery')

            phase = 'pending_two_scans'
            if args.check_mesh_cadence:
                cadence_counts = product_counts()
                cadence_products = {key: values[-1] if values else None for key, values in received.items()}
                cadence_trace_start = len(traces())
            scan(100_000_000, (1.5, .15, .15), 1)
            scan(200_000_000, (.15, 1.5, .15), 2)
            pending = stage(phase)
            check('two_scans_pending_before_ros_timer', pending['esdf_update_period'] == 1. and
                  pending['esdf_pending'] and pending['esdf_input_sequence'] == 2 and
                  pending['esdf_completed_sequence'] == 0 and pending['esdf_updates'] == 0)
            if args.check_mesh_cadence:
                mesh_window('before_first_esdf', 2, cadence_counts, cadence_products, cadence_trace_start)

            phase = 'save_flush'
            before = product_counts()
            map_path = args.output.with_suffix('.vxblx')
            response = call(files['save_map'], FilePath.Request(file_path=str(map_path)))
            check('save_succeeds', response.success and map_path.is_file() and map_path.stat().st_size > 0)
            updated_products(before, 200_000_000)
            saved = stage(phase)
            check('save_catches_up_pending_sequences', not saved['esdf_pending'] and
                  saved['esdf_input_sequence'] == saved['esdf_completed_sequence'] == 2 and
                  saved['esdf_updates'] == 1 and saved['esdf_observation_stamp_ns'] == 200_000_000)
            saved_products = {key: dict(received[key][-1]) for key in products}
            report['saved_map'] = {'path': str(map_path), 'bytes': map_path.stat().st_size,
                                   'sha256': hashlib.sha256(map_path.read_bytes()).hexdigest()}

            phase = 'pending_third_scan'
            if args.check_mesh_cadence:
                cadence_counts = product_counts()
                cadence_trace_start = len(traces())
            scan(300_000_000, (-1.5, .15, .15), 3)
            pending = stage(phase)
            check('third_scan_pending_preserves_previous_product', pending['esdf_pending'] and
                  pending['esdf_input_sequence'] == 3 and pending['esdf_completed_sequence'] == 2 and
                  all(received[key][-1] == saved_products[key] for key in products))
            if args.check_mesh_cadence:
                mesh_window('after_save_new_scan', 3, cadence_counts, saved_products, cadence_trace_start)

            phase = 'failed_load_flush'
            before = product_counts()
            missing = str(Path(temporary) / 'does_not_exist.vxblx')
            response = call(files['load_map'], FilePath.Request(file_path=missing))
            report['expected_load_failure'] = {'success': response.success, 'message': response.message}
            check('missing_load_reports_failure', not response.success)
            updated_products(before, 300_000_000)
            failed_load = stage(phase)
            check('failed_load_still_flushes_and_publishes', not failed_load['esdf_pending'] and
                  failed_load['esdf_input_sequence'] == failed_load['esdf_completed_sequence'] == 3 and
                  failed_load['esdf_updates'] == 2 and failed_load['esdf_observation_stamp_ns'] == 300_000_000 and
                  failed_load['loaded_maps'] == 0 and failed_load['map_epoch'] == saved['map_epoch'])

            phase = 'reset'
            before = product_counts()
            response = call(services['reset_map'], Trigger.Request())
            check('reset_succeeds', response.success)
            updated_products(before, 0, empty=True)
            reset = stage(phase)
            check('reset_clears_map_and_observation_provenance', reset['allocated_blocks'] == 0 and
                  reset['integrated_scans'] == 0 and reset['esdf_input_sequence'] == 0 and
                  reset['esdf_completed_sequence'] == 0 and not reset['esdf_pending'] and
                  reset['esdf_observation_stamp_ns'] == 0 and reset['map_epoch'] > saved['map_epoch'])

            phase = 'load_saved'
            before = product_counts()
            response = call(files['load_map'], FilePath.Request(file_path=str(map_path)))
            check('load_saved_succeeds', response.success)
            updated_products(before, 0)
            loaded = stage(phase)
            check('load_refreshes_exact_saved_products_with_unknown_time', loaded['loaded_maps'] == 1 and
                  loaded['integrated_scans'] == 0 and loaded['esdf_input_sequence'] == 0 and
                  loaded['esdf_completed_sequence'] == 0 and not loaded['esdf_pending'] and
                  loaded['esdf_observation_stamp_ns'] == 0 and loaded['map_epoch'] > reset['map_epoch'] and
                  all(received[key][-1]['sha256_sorted_points'] == saved_products[key]['sha256_sorted_points']
                      for key in products))

            phase = 'ros_timer'
            scan(400_000_000, (.15, .15, 1.5), 1)
            pending = status()
            check('restored_map_accepts_pending_scan', pending['esdf_pending'] and
                  pending['esdf_input_sequence'] == 1 and pending['esdf_completed_sequence'] == 0)
            before = product_counts()
            sim_ns = 1_100_000_000
            updated_products(before, 400_000_000)
            timed = stage(phase)
            check('ros_timer_catches_up_with_input_stamp', not timed['esdf_pending'] and
                  timed['esdf_input_sequence'] == timed['esdf_completed_sequence'] == 1 and
                  timed['esdf_observation_stamp_ns'] == 400_000_000 and
                  timed['esdf_updates'] == pending['esdf_updates'] + 1 and timed['rejected_scans'] == 0)
            if args.check_mesh_cadence:
                # Finish the last dirty mesh callback before attaching readers
                # or inspecting trace pairing; no subsequent scan makes it
                # dirty again during these final service checks.
                wait(lambda: any(item['phase'] == 'mesh_generate_end' and
                                 item['map_epoch'] == timed['map_epoch'] and
                                 item['integrated_scans'] == 1 for item in traces()),
                     'restored map wall-clock mesh completion')
                phase = 'late_subscriber'
                before = product_counts()
                latest_products = {key: dict(received[key][-1]) for key in products}
                subscriptions.extend(node.create_subscription(PointCloud2, namespace + topic,
                    lambda message, topic=topic: consume(topic, message, late=True), durable)
                    for topic in products)
                wait(lambda: all(late_received[key] for key in products), 'transient-local late ESDF products')
                check('late_subscriber_receives_latest_without_regeneration',
                      all(late_received[key][-1] == latest_products[key] for key in products) and
                      product_counts() == before)
                stage(phase)

                phase = 'explicit_unchanged_stamp'
                before = product_counts()
                response = call(services['publish_pointclouds'], Trigger.Request())
                check('explicit_pointcloud_request_succeeds', response.success)
                updated_products(before, 400_000_000)
                explicit = stage(phase)
                check('explicit_request_forces_each_product_without_new_observation',
                      all(len(received[key]) == before[key] + 1 and
                          received[key][-1] == latest_products[key] for key in products) and
                      explicit['esdf_updates'] == timed['esdf_updates'] and
                      explicit['esdf_input_sequence'] == timed['esdf_input_sequence'] and
                      explicit['esdf_completed_sequence'] == timed['esdf_completed_sequence'])

                # Validate paired generation stages and their nesting. A mesh
                # refresh may build TSDF products, but no ESDF stage may occur
                # inside that wall-clock callback in periodic mode.
                pairs = {'mesh_generate_begin': 'mesh_generate_end',
                         'mesh_update_begin': 'mesh_update_end',
                         'esdf_update_begin': 'esdf_update_end',
                         'esdf_voxels_generate_begin': 'esdf_voxels_generate_end',
                         'esdf_products_begin': 'esdf_products_end',
                         'tsdf_products_begin': 'tsdf_products_end'}
                events = traces()
                stack, completed, mesh_records = [], {name: 0 for name in pairs}, []
                mesh_start = None
                for index, item in enumerate(events):
                    event = item['phase']
                    if event in pairs:
                        if event.startswith('esdf_') and mesh_start is not None:
                            raise AssertionError('ESDF generated inside wall-clock mesh callback')
                        if event == 'mesh_generate_begin':
                            mesh_start = index
                        stack.append(event)
                    elif event in pairs.values():
                        if not stack or pairs[stack[-1]] != event:
                            raise AssertionError('incomplete or misnested generation trace: ' + event)
                        begin = stack.pop()
                        completed[begin] += 1
                        if event == 'mesh_generate_end':
                            phases = [record['phase'] for record in events[mesh_start:index + 1]]
                            mesh_records.append({'begin': mesh_start, 'end': index,
                                                 'stamp_ns': item['stamp_ns'], 'phases': phases})
                            if any(required not in phases for required in
                                   ('mesh_update_begin', 'mesh_update_end',
                                    'tsdf_products_begin', 'tsdf_products_end')):
                                raise AssertionError('mesh callback omitted update or TSDF product stages')
                            mesh_start = None
                report['mesh_cadence']['trace_pairs'] = completed
                report['mesh_cadence']['mesh_callbacks'] = mesh_records
                check('generation_trace_stages_complete', not stack and all(completed.values()))
            report['status'] = 'passed'
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
        report['traceback'] = traceback.format_exc()
    finally:
        if process is not None:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
            try:
                process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                report['checks']['mapper_exited_cleanly'] = False
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=1.)
            report['mapper_exit_code'] = process.returncode
            report['checks'].setdefault('mapper_exited_cleanly', process.returncode == 0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if log is not None:
            log.close()
        report['elapsed_wall_s'] = round(time.monotonic() - started, 3)
        if not all(report['checks'].values()):
            report['status'] = 'failed'
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({key: report.get(key) for key in
                          ('status', 'checks', 'error', 'elapsed_wall_s', 'mapper_exit_code')}, indent=2))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

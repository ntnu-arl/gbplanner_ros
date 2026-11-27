#!/usr/bin/env python3
"""Check native TSDF aliases against ray clearing, analytic weights and a zero time budget."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(170 + os.getpid() % 20))

    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from std_srvs.srv import Trigger

    names = ('legacy', 'override', 'zero')
    resolution, truncation, clearing_weight = .25, .5, .1
    summary = {'status': 'failed', 'scope': 'three native ROS2 TSDF mapper configurations',
               'checks': {}, 'observations': {}}
    started, probe = time.monotonic(), None
    deadline = started + 25.
    processes, logs, latest_status = {}, {}, {}
    snapshots = {name: {} for name in names}
    expected_configs = {
        'legacy': {'use_const_weight': True, 'use_symmetric_weight_dropoff': False,
                   'weight_ray_by_range': False, 'allow_clear': True,
                   'integration_order_mode': 'sorted', 'start_voxel_subsampling_factor': 3.,
                   'max_consecutive_ray_collisions': 2, 'clear_checks_every_n_frames': 2,
                   'max_integration_time_s': 1., 'anti_grazing': True},
        'override': {'use_const_weight': True, 'use_symmetric_weight_dropoff': True,
                     'weight_ray_by_range': True, 'allow_clear': False,
                     'integration_order_mode': 'mixed', 'start_voxel_subsampling_factor': 4.,
                     'max_consecutive_ray_collisions': 4, 'clear_checks_every_n_frames': 3,
                     'max_integration_time_s': 2., 'anti_grazing': False},
    }
    legacy_conflicts = {'use_const_weight': False, 'use_symmetric_weight_dropoff': False,
                        'weight_ray_by_range': False, 'allow_clear': True,
                        'integration_order_mode': 'sorted', 'start_voxel_subsampling_factor': 3.,
                        'max_consecutive_ray_collisions': 2, 'clear_checks_every_n_frames': 2,
                        'max_integration_time_s': 1., 'anti_grazing': True}

    def stop_processes():
        for process in processes.values():
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        for name, process in processes.items():
            try:
                process.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                summary['checks'][name + '_exited_cleanly'] = False
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=1.)
            summary['checks'].setdefault(name + '_exited_cleanly', process.returncode == 0)

    def describe(records):
        return {'observed_voxels': len(records), 'cells': [
            {'index': list(index), 'distance': values[0], 'weight': values[1]}
            for index, values in sorted(records.items())]}

    try:
        with tempfile.TemporaryDirectory(prefix='gbplanner-tsdf-configuration-') as temporary:
            rclpy.init(args=[])
            probe = Node('voxblox_tsdf_configuration_probe')
            reliable = QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE)
            durable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            ordinary, freespace, transforms, clients, subscriptions = {}, {}, {}, {}, []

            def consume(name, message):
                fields = {field.name: field for field in message.fields}
                if (message.header.frame_id != 'map' or message.height != 1 or
                        set(fields) != {'x', 'y', 'z', 'distance', 'weight'} or
                        len(fields) != len(message.fields) or
                        message.row_step != message.width * message.point_step or
                        len(message.data) != message.row_step):
                    raise AssertionError('malformed tsdf_voxels snapshot')
                if any(field.datatype != PointField.FLOAT32 or field.count != 1 or
                       field.offset + 4 > message.point_step for field in fields.values()):
                    raise AssertionError('TSDF fields must be scalar float32')
                endian, records = ('>' if message.is_bigendian else '<'), {}
                for i in range(message.width):
                    values = [struct.unpack_from(endian + 'f', message.data,
                        i * message.point_step + fields[field].offset)[0]
                        for field in ('x', 'y', 'z', 'distance', 'weight')]
                    if not all(math.isfinite(value) for value in values) or values[4] <= 0.:
                        raise AssertionError('invalid observed TSDF voxel')
                    index = tuple(math.floor(value / resolution) for value in values[:3])
                    if index in records:
                        raise AssertionError('duplicate TSDF voxel')
                    records[index] = tuple(values[3:])
                stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                snapshots[name][stamp] = records

            for name in names:
                prefix = '/' + name + '/'
                ordinary[name] = probe.create_publisher(PointCloud2, prefix + 'points', reliable)
                freespace[name] = probe.create_publisher(PointCloud2, prefix + 'freespace_pointcloud', reliable)
                transforms[name] = probe.create_publisher(TransformStamped, prefix + 'transform', reliable)
                clients[name] = probe.create_client(Trigger, prefix + 'mapper_status')
                subscriptions.append(probe.create_subscription(PointCloud2, prefix + 'tsdf_voxels',
                    lambda message, name=name: consume(name, message), durable))
                parameters = {
                    'frame_id': 'map', 'sensor_frame': 'lidar', 'use_tf_transforms': False,
                    'input_sensors': ['main'], 'input.main.topic': 'points', 'input.main.frame': 'lidar',
                    'use_freespace_pointcloud': True, 'freespace.sensor_frames': ['lidar'],
                    'map.storage': 'sparse', 'map.resolution': resolution,
                    'tsdf.voxels_per_side': 4, 'tsdf.method': 'simple',
                    'tsdf.truncation_distance': truncation, 'tsdf.use_weight_dropoff': True,
                    'tsdf.clearing_ray_weight_factor': clearing_weight,
                    'max_ray_length_m': 2., 'min_ray_length_m': .1,
                    'color_mode': 'gray', 'esdf.enabled': False,
                    'update_mesh_every_n_sec': 0., 'publish_map_every_n_sec': 0.,
                }
                if name == 'legacy':
                    parameters.update(expected_configs[name])
                elif name == 'override':
                    parameters.update(legacy_conflicts)
                    parameters.update({'tsdf.' + key: value for key, value in expected_configs[name].items()})
                    # An explicit namespaced range wins over a different old name.
                    parameters.update({'sensor.max_range': 2., 'max_ray_length_m': 1.})
                else:
                    parameters.update({'tsdf.method': 'fast', 'tsdf.max_integration_time_s': 0.})
                params_path = Path(temporary) / (name + '.yaml')
                params_path.write_text(json.dumps({prefix + 'voxblox_mapper': {'ros__parameters': parameters}}))
                log_path = args.output.with_name(args.output.stem + '_' + name + '.log')
                logs[name] = log_path.open('w')
                processes[name] = subprocess.Popen(
                    ['ros2', 'run', 'gbplanner_voxblox', 'voxblox_mapper_node', '--ros-args',
                     '-r', '__ns:=/' + name, '--params-file', str(params_path)],
                    stdout=logs[name], stderr=subprocess.STDOUT, start_new_session=True, env=dict(os.environ))

            def spin_until(predicate, label):
                while True:
                    for name, process in processes.items():
                        if process.poll() is not None:
                            raise RuntimeError(f'{name} mapper exited ({process.returncode})')
                    if time.monotonic() >= deadline:
                        raise TimeoutError(label)
                    value = predicate()
                    if value:
                        return value
                    rclpy.spin_once(probe, timeout_sec=.01)

            def status(name):
                future = clients[name].call_async(Trigger.Request())
                spin_until(future.done, name + ' status response')
                response = future.result()
                if response is None:
                    raise RuntimeError(name + ' status returned no response')
                # The service can return success=false for an initially empty map.
                latest_status[name] = json.loads(response.message)
                return latest_status[name]

            def check(name, condition):
                summary['checks'][name] = bool(condition)
                if not condition:
                    raise AssertionError(name)

            def send_stage(label, stamp, origin_x, endpoint, free=False):
                expected_transforms = {name: status(name)['received_transforms'] + 1 for name in names}
                for name in names:
                    pose = TransformStamped()
                    pose.header.frame_id, pose.child_frame_id = 'map', 'lidar'
                    pose.header.stamp.sec = stamp
                    pose.transform.translation.x = origin_x
                    pose.transform.rotation.w = 1.
                    transforms[name].publish(pose)
                for name in names:
                    spin_until(lambda name=name: status(name)['received_transforms'] == expected_transforms[name],
                               name + ' transform reception')
                cloud = PointCloud2()
                cloud.header.frame_id, cloud.header.stamp.sec = 'lidar', stamp
                cloud.height, cloud.width = 1, 1
                cloud.point_step, cloud.row_step = 12, 12
                cloud.is_dense = True
                cloud.fields = [PointField(name=field, offset=4 * i,
                    datatype=PointField.FLOAT32, count=1) for i, field in enumerate(('x', 'y', 'z'))]
                cloud.data = struct.pack('<fff', *endpoint)
                for name in names:
                    (freespace if free else ordinary)[name].publish(cloud)
                stamp_ns = stamp * 1_000_000_000
                for name in names:
                    spin_until(lambda name=name: stamp_ns in snapshots[name], name + ' ' + label + ' snapshot')
                result = {name: snapshots[name][stamp_ns] for name in names}
                summary['observations'][label] = {name: describe(records) for name, records in result.items()}
                return result

            spin_until(lambda: all(client.service_is_ready() for client in clients.values()), 'service discovery')
            spin_until(lambda: all(pub.get_subscription_count() for group in (ordinary, freespace, transforms)
                                  for pub in group.values()), 'input discovery')
            spin_until(lambda: all(probe.count_publishers(sub.topic_name) for sub in subscriptions), 'output discovery')
            for name, expected in expected_configs.items():
                resolved = status(name)['tsdf_configuration']
                matches = all(resolved.get(key) == value if isinstance(value, (bool, str, int)) else
                              math.isclose(resolved.get(key, math.inf), value, rel_tol=1e-6, abs_tol=1e-7)
                              for key, value in expected.items())
                check(name + '_parameter_resolution', matches and resolved['max_range'] == 2.)

            # The same out-of-range return clears to max_range only when
            # allow_clear is enabled; Fast's zero budget must not touch a voxel.
            far = send_stage('out_of_range_return', 10, 0., (0., 0., 4.))
            check('legacy_allow_clear_fuses_positive_free_space', len(far['legacy']) >= 8 and
                  all(abs(distance - truncation) < 1e-6 and abs(weight - clearing_weight) < 1e-6
                      for distance, weight in far['legacy'].values()))
            check('namespaced_allow_clear_false_skips_out_of_range_return', not far['override'])

            # A new world location avoids previous observations. At center
            # (4.125,.125,1.125), projective distance is 1.5-1.125=.375.
            # Const wins over dropoff; symmetric wins over both and the base
            # weight is 1/z^2: (.1+.9*(.5-.375)/.5)/(1.5^2)=.144444...
            near = send_stage('in_range_weighted_return', 20, 4., (0., 0., 1.5))
            index = (16, 0, 4)
            expected_weight = (clearing_weight + (1. - clearing_weight) * .25) / (1.5 ** 2)
            legacy_value, override_value = near['legacy'].get(index), near['override'].get(index)
            summary['observations']['analytic_query'] = {'index': list(index), 'distance': .375,
                'legacy_weight': 1., 'override_weight': expected_weight}
            check('constant_weight_path_matches_analytic_voxel', legacy_value is not None and
                  abs(legacy_value[0] - .375) < 1e-6 and abs(legacy_value[1] - 1.) < 1e-6)
            check('symmetric_priority_and_range_weight_match_analytic_voxel', override_value is not None and
                  abs(override_value[0] - .375) < 1e-6 and abs(override_value[1] - expected_weight) < 1e-6)

            # Explicit free-space clouds remain valid with allow_clear=false.
            # At z=4 the base weight is 1/16, so clear voxels receive .1/16.
            free = send_stage('explicit_freespace', 30, 8., (0., 0., 4.), free=True)
            fresh = {index: value for index, value in free['override'].items() if index[0] == 32}
            check('explicit_freespace_bypasses_allow_clear_false', len(fresh) >= 8 and
                  all(abs(distance - truncation) < 1e-6 and abs(weight - .00625) < 1e-7
                      for distance, weight in fresh.values()))
            zero_status = status('zero')
            check('zero_fast_budget_keeps_all_three_snapshots_empty',
                  not far['zero'] and not near['zero'] and not free['zero'] and
                  zero_status['allocated_blocks'] == 0 and
                  zero_status['tsdf_configuration']['max_integration_time_s'] == 0.)
            for name in names:
                final = status(name)
                check(name + '_two_regular_and_one_free_scan_complete', final['integrated_scans'] == 2 and
                      final['rejected_scans'] == 0 and final['freespace']['integrated_clouds'] == 1 and
                      final['freespace']['rejected_clouds'] == 0)
            summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
    finally:
        stop_processes()
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for log in logs.values():
            log.close()
        summary['mapper_exit_codes'] = {name: process.returncode for name, process in processes.items()}
        summary['last_mapper_status'] = latest_status
        summary['elapsed_seconds'] = round(time.monotonic() - started, 3)
        if not all(summary['checks'].values()):
            summary['status'] = 'failed'
        args.output.write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

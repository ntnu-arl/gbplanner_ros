#!/usr/bin/env python3
"""Check ESDF parameter aliases against native full/quasi distance and crust output."""
import argparse
from collections import deque
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
    os.environ.setdefault('ROS_DOMAIN_ID', str(150 + os.getpid() % 20))

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_srvs.srv import Trigger
    from voxblox_msgs.msg import Block, Layer

    names = ('full', 'quasi')
    summary = {'status': 'failed', 'scope': 'two native ROS2 ESDF mapper configurations',
               'checks': {}, 'observations': {}, 'events': []}
    processes, logs, latest_status = {}, {}, {}
    received = {name: deque(maxlen=8) for name in names}
    probe = None
    started = time.monotonic()
    deadline = started + 25.
    resolution, side, seed_distance = .25, 4, .05
    seed_index, query_index, crust_index = (0, 0, 0), (2, 1, 0), (3, 3, 3)
    expected_configs = {
        'full': {'full_euclidean_distance': True, 'min_distance': .1, 'max_distance': 2.,
                 'default_distance': 3., 'min_diff': .0001, 'add_occupied_crust': True},
        'quasi': {'full_euclidean_distance': False, 'min_distance': .15, 'max_distance': 2.,
                  'default_distance': 2., 'min_diff': .0002, 'add_occupied_crust': False},
    }
    specific_params = {
        'full': {'esdf_euclidean_distance': True, 'esdf_min_distance_m': .1,
                 'esdf_default_distance_m': 3., 'esdf_min_diff_m': .0001,
                 'esdf_add_occupied_crust': True},
        'quasi': {'esdf.full_euclidean_distance': False, 'esdf_euclidean_distance': True,
                  'esdf.min_distance': .15, 'esdf_min_distance_m': .35,
                  'esdf.default_distance': 1., 'esdf_default_distance_m': 4.,
                  'esdf.min_diff': .0002, 'esdf_min_diff_m': .02,
                  'esdf.add_occupied_crust': False, 'esdf_add_occupied_crust': True},
    }

    def stop_processes():
        # Signal both nodes before waiting for either one.
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

    def word(value):
        return struct.unpack('<I', struct.pack('<f', value))[0]

    def layer(with_seed, unknown_corner=False):
        message = Layer(voxel_size=resolution, voxels_per_side=side,
                        layer_type='tsdf', action=Layer.ACTION_RESET)
        block = Block(x_index=0, y_index=0, z_index=0)
        data = []
        for z in range(side):
            for y in range(side):
                for x in range(side):
                    index = (x, y, z)
                    distance = seed_distance if with_seed and index == seed_index else 2.
                    weight = 0. if unknown_corner and index == crust_index else 1.
                    data.extend((word(distance), word(weight), 0x14283CFF))
        block.data = data
        message.blocks = [block]
        return message

    def decode_layer(message):
        if (message.layer_type != 'esdf' or message.voxels_per_side != side or
                not math.isclose(message.voxel_size, resolution, abs_tol=1e-9) or
                message.action != Layer.ACTION_RESET):
            raise AssertionError('expected a complete ESDF RESET with fixture geometry')
        if len(message.blocks) != 1:
            raise AssertionError('the single-block TSDF fixture must yield one ESDF block')
        block = message.blocks[0]
        if (int(block.x_index), int(block.y_index), int(block.z_index)) != (0, 0, 0):
            raise AssertionError('unexpected ESDF block index')
        if len(block.data) != 2 * side ** 3:
            raise AssertionError('ESDF payload must have two uint32 words per voxel')
        records = {}
        for i in range(side ** 3):
            distance = struct.unpack('<f', struct.pack('<I', int(block.data[2 * i])))[0]
            packed = int(block.data[2 * i + 1])
            if not math.isfinite(distance):
                raise AssertionError('nonfinite ESDF distance')
            index = (i % side, (i // side) % side, i // (side * side))
            records[index] = {'distance': distance, 'flags': packed & 255,
                              'observed': bool(packed & 1), 'hallucinated': bool(packed & 2),
                              'in_queue': bool(packed & 4), 'fixed': bool(packed & 8)}
        return records

    def describe(records):
        ordered = [records[(x, y, z)] for z in range(side) for y in range(side) for x in range(side)]
        return {'seed': records[seed_index], 'query': records[query_index], 'corner': records[crust_index],
                'observed_voxels': sum(record['observed'] for record in ordered),
                'hallucinated_voxels': sum(record['hallucinated'] for record in ordered),
                'distances_in_xyz_linear_order': [record['distance'] for record in ordered],
                'flags_in_xyz_linear_order': [record['flags'] for record in ordered]}

    try:
        with tempfile.TemporaryDirectory(prefix='gbplanner-esdf-configuration-') as temporary:
            rclpy.init(args=[])
            probe = Node('voxblox_esdf_configuration_probe')
            qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            publishers, clients, subscriptions = {}, {}, []

            def consume(name, message):
                records = decode_layer(message)
                received[name].append(records)
                summary['events'].append({'node': name, 'topic': 'esdf_map_out',
                    'action': int(message.action), 'query': records[query_index],
                    'corner': records[crust_index],
                    'observed_voxels': sum(record['observed'] for record in records.values())})

            for name in names:
                publishers[name] = probe.create_publisher(Layer, '/' + name + '/tsdf_map_in', qos)
                clients[name] = probe.create_client(Trigger, '/' + name + '/mapper_status')
                subscriptions.append(probe.create_subscription(Layer, '/' + name + '/esdf_map_out',
                    lambda message, name=name: consume(name, message), qos))
                parameters = {
                    'frame_id': 'map', 'sensor_frame': 'lidar', 'map.storage': 'sparse',
                    'map.resolution': resolution, 'tsdf.voxels_per_side': side,
                    'tsdf.method': 'simple', 'tsdf.truncation_distance': .5,
                    'esdf.enabled': True, 'esdf.max_distance': 2.,
                    'publish_esdf_map': True, 'publish_map_every_n_sec': 0.,
                    'update_mesh_every_n_sec': 0., **specific_params[name],
                }
                params_path = Path(temporary) / (name + '.yaml')
                params_path.write_text(json.dumps({'/' + name + '/voxblox_mapper': {'ros__parameters': parameters}}))
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
                spin_until(future.done, name + ' mapper_status response')
                response = future.result()
                # An empty map returns success=false but still carries status.
                if response is None:
                    raise RuntimeError(name + ' mapper_status returned no response')
                latest_status[name] = json.loads(response.message)
                return latest_status[name]

            def check(name, condition):
                summary['checks'][name] = bool(condition)
                if not condition:
                    raise AssertionError(name)

            def send_both(stage, message):
                previous = {name: status(name)['received_layers'] for name in names}
                for name in names:
                    received[name].clear()
                    publishers[name].publish(message)
                result = {}
                for name in names:
                    spin_until(lambda name=name: status(name)['received_layers'] == previous[name] + 1,
                               name + ' ' + stage + ' Layer reception')
                    result[name] = spin_until(lambda name=name: received[name][-1] if received[name] else None,
                                             name + ' ' + stage + ' ESDF output')
                summary['observations'][stage] = {name: describe(records) for name, records in result.items()}
                return result

            spin_until(lambda: all(client.service_is_ready() for client in clients.values()), 'status service discovery')
            spin_until(lambda: all(publisher.get_subscription_count() > 0 for publisher in publishers.values()),
                       'Layer input discovery')
            spin_until(lambda: all(probe.count_publishers(subscription.topic_name) > 0 for subscription in subscriptions),
                       'ESDF output discovery')
            summary['observations']['resolved_configuration'] = {}
            for name in names:
                resolved = status(name)['esdf_configuration']
                summary['observations']['resolved_configuration'][name] = resolved
                matches = all(resolved.get(field) is expected if isinstance(expected, bool) else
                              math.isclose(resolved.get(field, math.inf), expected, rel_tol=1e-6, abs_tol=1e-7)
                              for field, expected in expected_configs[name].items())
                check(name + '_parameter_resolution', matches)

            # A positive fixed seed avoids the original zero-distance sign
            # branch. Keep this fixture fully observed: crust should not create
            # extra signed sources that obscure the distance-mode comparison.
            seeded = send_both('positive_seed', layer(with_seed=True))
            expected_distances = {'full': seed_distance + math.sqrt(5.) * resolution,
                                  'quasi': seed_distance + (1. + math.sqrt(2.)) * resolution}
            summary['observations']['expected_query_distances_m'] = expected_distances
            for name in names:
                records = seeded[name]
                check(name + '_seed_is_fixed_and_observed', records[seed_index]['fixed'] and
                      records[seed_index]['observed'] and not records[seed_index]['hallucinated'] and
                      abs(records[seed_index]['distance'] - seed_distance) < 1e-6)
                check(name + '_query_matches_distance_mode', records[query_index]['observed'] and
                      not records[query_index]['fixed'] and
                      abs(records[query_index]['distance'] - expected_distances[name]) < 2e-5)
                check(name + '_seeded_layer_fully_observed_without_hallucination',
                      all(record['observed'] and not record['hallucinated'] and not record['in_queue']
                          for record in records.values()))
            check('distance_modes_produce_distinct_geometry',
                  seeded['quasi'][query_index]['distance'] - seeded['full'][query_index]['distance'] > .04)

            # With no fixed-band seed, every observed positive voxel takes the
            # resolved default distance. The quasi input of 1 m must be raised
            # to max_distance=2 m; its conflicting legacy value of 4 m must lose.
            unseeded = send_both('positive_without_seed', layer(with_seed=False))
            for name in names:
                default = expected_configs[name]['default_distance']
                check(name + '_all_unseeded_voxels_use_resolved_default',
                      all(record['observed'] and not record['fixed'] and not record['hallucinated'] and
                          not record['in_queue'] and abs(record['distance'] - default) < 1e-6
                          for record in unseeded[name].values()))

            # Isolate occupied crust from propagation: the single unknown corner
            # has no positive fixed seed and thus exposes the batch option itself.
            crust = send_both('unknown_corner_without_seed', layer(with_seed=False, unknown_corner=True))
            full_corner, quasi_corner = crust['full'][crust_index], crust['quasi'][crust_index]
            check('legacy_crust_option_creates_observed_hallucinated_obstacle',
                  full_corner['observed'] and full_corner['hallucinated'] and not full_corner['fixed'] and
                  abs(full_corner['distance'] + expected_configs['full']['default_distance']) < 1e-6)
            check('namespaced_crust_false_overrides_legacy_true',
                  not quasi_corner['observed'] and not quasi_corner['hallucinated'])
            for name in names:
                check(name + '_crust_does_not_change_observed_positive_voxels',
                      all(record['observed'] and not record['hallucinated'] and
                          abs(record['distance'] - expected_configs[name]['default_distance']) < 1e-6
                          for index, record in crust[name].items() if index != crust_index))
                final = status(name)
                check(name + '_three_imports_finish_without_rejections', final['received_layers'] == 3 and
                      final['rejected_layers'] == 0 and final['integrated_scans'] == 0)
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
        summary['last_received_esdf'] = {name: describe(records[-1]) for name, records in received.items() if records}
        summary['elapsed_seconds'] = round(time.monotonic() - started, 3)
        if not all(summary['checks'].values()):
            summary['status'] = 'failed'
        args.output.write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

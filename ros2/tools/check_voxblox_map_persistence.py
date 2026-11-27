#!/usr/bin/env python3
"""Round-trip real TSDF/ESDF state through one file and resume native ROS2 fusion."""
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    map_path = args.output.with_suffix('.vxblx')
    os.environ.setdefault('ROS_DOMAIN_ID', str(130 + os.getpid() % 20))

    import rclpy
    from gbplanner_voxblox.srv import FilePath
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from std_srvs.srv import Trigger
    from voxblox_msgs.msg import Block, Layer

    names, kinds = ('source', 'target'), ('tsdf', 'esdf')
    summary = {'status': 'failed', 'scope': 'sequential native TSDF+ESDF save/load and resumed fusion',
               'checks': {}, 'observations': {}, 'events': []}
    processes, logs, latest_status = {}, {}, {}
    latest = {name: {} for name in names}
    probe = active = None
    started = time.monotonic()
    deadline = started + 25.
    resolution, side, special_index = .25, 4, (10, 0, 0)
    parents = ((3, -2, 5), (-4, 6, -7), (8, -9, -10))

    def stop(name):
        process = processes.get(name)
        if process is None:
            return
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
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

    def float_word(value):
        return struct.unpack('<I', struct.pack('<f', value))[0]

    def as_float(value):
        return struct.unpack('<f', struct.pack('<I', value))[0]

    def esdf_voxel(data, index):
        bits = data[2 * index + 1]
        parent = []
        for shift in (24, 16, 8):
            value = (bits >> shift) & 255
            parent.append(value - 256 if value >= 128 else value)
        return {'distance': as_float(data[2 * index]), 'parent': parent,
                'observed': bool(bits & 1), 'hallucinated': bool(bits & 2),
                'in_queue': bool(bits & 4), 'fixed': bool(bits & 8)}

    def describe(snapshot):
        esdf = [esdf_voxel(data, i) for data in snapshot['esdf'].values() for i in range(side ** 3)]
        return {'tsdf_blocks': len(snapshot['tsdf']), 'esdf_blocks': len(snapshot['esdf']),
                'esdf_only_blocks': len(set(snapshot['esdf']) - set(snapshot['tsdf'])),
                'observed_tsdf_voxels': sum(as_float(data[i]) > 0 for data in snapshot['tsdf'].values()
                                           for i in range(1, len(data), 3)),
                'observed_esdf_voxels': sum(voxel['observed'] for voxel in esdf),
                'hallucinated_free_voxels': sum(voxel['observed'] and voxel['hallucinated'] and voxel['distance'] > 0
                                               for voxel in esdf),
                'hallucinated_occupied_voxels': sum(voxel['observed'] and voxel['hallucinated'] and voxel['distance'] < 0
                                                   for voxel in esdf),
                'negative_parent_voxels': sum(any(value < 0 for value in voxel['parent']) for voxel in esdf),
                'in_queue_voxels': sum(voxel['in_queue'] for voxel in esdf),
                'mixed_parent_samples': [esdf_voxel(snapshot['esdf'][special_index], i) for i in range(3)]
                                        if special_index in snapshot['esdf'] else []}

    def normalized(kind, blocks):
        # Queue membership belongs to the old process, unlike persistent voxel
        # distance, parent, observed, fixed and hallucinated state.
        return {key: [value & ~4 if kind == 'esdf' and i % 2 else value for i, value in enumerate(data)]
                for key, data in blocks.items()}

    try:
        with tempfile.TemporaryDirectory(prefix='gbplanner-map-persistence-') as temporary:
            rclpy.init(args=[])
            probe = Node('voxblox_map_persistence_probe')
            qos = QoSProfile(depth=40, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            publishers, clients, files, subscriptions = {}, {}, {}, []

            def consume(name, kind, message):
                if (message.layer_type != kind or message.voxels_per_side != side or
                        not math.isclose(message.voxel_size, resolution, abs_tol=1e-9) or
                        message.action != Layer.ACTION_RESET):
                    raise AssertionError('expected a complete Layer RESET with fixture geometry')
                blocks, stride = {}, 3 if kind == 'tsdf' else 2
                for block in message.blocks:
                    if len(block.data) != stride * side ** 3:
                        raise AssertionError('invalid Layer word count')
                    key = (int(block.x_index), int(block.y_index), int(block.z_index))
                    blocks[key] = [int(value) for value in block.data]
                latest[name][kind] = blocks
                summary['events'].append({'node': name, 'topic': kind + '_map_out',
                                          'blocks': len(blocks), 'action': int(message.action)})

            for name in names:
                prefix = '/' + name + '/'
                publishers[name] = {'transform': probe.create_publisher(TransformStamped, prefix + 'transform', qos),
                                    'points': probe.create_publisher(PointCloud2, prefix + 'points', qos),
                                    'esdf': probe.create_publisher(Layer, prefix + 'esdf_map_in', qos)}
                clients[name] = {operation: probe.create_client(Trigger, prefix + operation)
                                 for operation in ('mapper_status', 'publish_map')}
                files[name] = {operation: probe.create_client(FilePath, prefix + operation)
                               for operation in ('save_map', 'load_map')}
                for kind in kinds:
                    subscriptions.append(probe.create_subscription(Layer, prefix + kind + '_map_out',
                        lambda message, name=name, kind=kind: consume(name, kind, message), qos))

            def spin_until(predicate, label):
                while True:
                    if active is not None and processes[active].poll() is not None:
                        raise RuntimeError(f'{active} mapper exited ({processes[active].returncode})')
                    if time.monotonic() >= deadline:
                        raise TimeoutError(label)
                    value = predicate()
                    if value:
                        return value
                    rclpy.spin_once(probe, timeout_sec=.01)

            def call(client, request, label, status_response=False):
                future = client.call_async(request)
                spin_until(future.done, label)
                response = future.result()
                if response is None or (not status_response and not response.success):
                    raise RuntimeError(f'{label}: {None if response is None else response.message}')
                return response

            def status(name):
                response = call(clients[name]['mapper_status'], Trigger.Request(), name + ' status', True)
                latest_status[name] = json.loads(response.message)
                return latest_status[name]

            def check(name, condition):
                summary['checks'][name] = bool(condition)
                if not condition:
                    raise AssertionError(name)

            def launch(name):
                nonlocal active
                parameters = {
                    'frame_id': 'map', 'sensor_frame': 'lidar', 'input_sensors': ['main'],
                    'input.main.topic': 'points', 'input.main.frame': 'lidar', 'use_tf_transforms': False,
                    'map.storage': 'sparse', 'map.resolution': resolution, 'tsdf.voxels_per_side': side,
                    'tsdf.method': 'simple', 'tsdf.truncation_distance': .5,
                    'esdf.enabled': True, 'esdf.clear_sphere_for_planning': True,
                    'esdf.clear_sphere_radius': .5, 'esdf.occupied_sphere_radius': 1.,
                    'publish_tsdf_map': True, 'publish_esdf_map': True,
                    'publish_map_every_n_sec': 0., 'update_mesh_every_n_sec': 0.,
                }
                params = Path(temporary) / (name + '.yaml')
                params.write_text(json.dumps({'/' + name + '/voxblox_mapper': {'ros__parameters': parameters}}))
                logs[name] = args.output.with_name(args.output.stem + '_' + name + '.log').open('w')
                processes[name] = subprocess.Popen(
                    ['ros2', 'run', 'gbplanner_voxblox', 'voxblox_mapper_node', '--ros-args',
                     '-r', '__ns:=/' + name, '--params-file', str(params)],
                    stdout=logs[name], stderr=subprocess.STDOUT, start_new_session=True, env=dict(os.environ))
                active = name
                spin_until(lambda: all(client.service_is_ready() for client in (*clients[name].values(), *files[name].values())),
                           name + ' services')
                spin_until(lambda: all(publisher.get_subscription_count() > 0 for publisher in publishers[name].values()),
                           name + ' inputs')
                spin_until(lambda: all(probe.count_publishers('/' + name + '/' + kind + '_map_out') > 0 for kind in kinds),
                           name + ' outputs')

            def snapshot(name):
                latest[name].clear()
                call(clients[name]['publish_map'], Trigger.Request(), name + ' full map publication')
                spin_until(lambda: all(kind in latest[name] for kind in kinds), name + ' both layers')
                return dict(latest[name])

            def scan(name, stamp, origin):
                transform = TransformStamped()
                transform.header.frame_id, transform.child_frame_id, transform.header.stamp.sec = 'map', 'tracking', stamp
                (transform.transform.translation.x, transform.transform.translation.y,
                 transform.transform.translation.z) = origin
                transform.transform.rotation.w = 1.
                cloud = PointCloud2()
                cloud.header.frame_id, cloud.header.stamp.sec = 'lidar', stamp
                cloud.height, cloud.width, cloud.point_step, cloud.row_step = 1, 1, 12, 12
                cloud.is_dense = True
                cloud.fields = [PointField(name=axis, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                                for i, axis in enumerate(('x', 'y', 'z'))]
                cloud.data = struct.pack('<fff', 1., .2, .1)
                publishers[name]['transform'].publish(transform)
                publishers[name]['points'].publish(cloud)
                spin_until(lambda: status(name)['integrated_scans'] == 1, name + ' real sensor integration')

            launch('source')
            scan('source', 21, (.125, .125, .125))
            sphere = snapshot('source')
            sphere_stats = describe(sphere)
            summary['observations']['actual_sensor_sphere'] = sphere_stats
            summary['observations']['source_after_scan'] = status('source')
            check('sensor_pose_creates_real_esdf_only_free_and_occupied_sphere',
                  latest_status['source']['esdf_clear_sphere_operations'] == 1 and sphere_stats['esdf_only_blocks'] > 0 and
                  sphere_stats['hallucinated_free_voxels'] > 0 and sphere_stats['hallucinated_occupied_voxels'] > 0)

            # Add a disjoint ESDF-only block. MERGE preserves the measured map
            # and real sphere while exercising mixed signed parent bytes.
            injected = Layer(voxel_size=resolution, voxels_per_side=side,
                             layer_type='esdf', action=Layer.ACTION_MERGE)
            block = Block(x_index=special_index[0], y_index=0, z_index=0)
            words = []
            for i in range(side ** 3):
                parent = parents[i % len(parents)]
                flags = 1 | 4 | (2 if i % 2 else 8)
                packed = flags | sum((value & 255) << shift for value, shift in zip(parent, (24, 16, 8)))
                words.extend((float_word(.625 + i / 64.), packed))
            block.data, injected.blocks = words, [block]
            publishers['source']['esdf'].publish(injected)
            spin_until(lambda: status('source')['received_layers'] == 1, 'mixed-parent ESDF MERGE')
            saved = snapshot('source')
            summary['observations']['saved_state'] = describe(saved)
            check('merge_preserves_real_sphere_blocks', all(saved['esdf'][key] == data for key, data in sphere['esdf'].items()) and
                  saved['tsdf'] == sphere['tsdf'])
            check('mixed_negative_parent_samples_are_present_before_save', special_index not in saved['tsdf'] and
                  all(esdf_voxel(saved['esdf'][special_index], i)['parent'] == list(parents[i]) for i in range(3)))
            call(files['source']['save_map'], FilePath.Request(file_path=str(map_path)), 'save combined map')
            check('combined_map_file_written', map_path.is_file() and map_path.stat().st_size > 0)
            summary['observations']['file'] = {'path': str(map_path), 'bytes': map_path.stat().st_size,
                                               'sha256': hashlib.sha256(map_path.read_bytes()).hexdigest()}
            stop('source')
            check('source_stopped_before_target_start', processes['source'].returncode == 0)
            active = None

            launch('target')
            call(files['target']['load_map'], FilePath.Request(file_path=str(map_path)), 'load combined map')
            restored = snapshot('target')
            summary['observations']['restored_state'] = describe(restored)
            for kind in kinds:
                expected, actual = normalized(kind, saved[kind]), normalized(kind, restored[kind])
                if expected != actual:
                    differences = []
                    for key in sorted(set(expected) | set(actual)):
                        if key not in expected or key not in actual:
                            differences.append({'block': list(key), 'expected_present': key in expected,
                                                'actual_present': key in actual})
                        else:
                            for i, (a, b) in enumerate(zip(expected[key], actual[key])):
                                if a != b:
                                    differences.append({'block': list(key), 'word': i, 'expected': a, 'actual': b})
                                    if len(differences) >= 8:
                                        break
                        if len(differences) >= 8:
                            break
                    summary['observations'][kind + '_mismatches'] = differences
                check(kind + '_all_voxel_words_survive_file_roundtrip', expected == actual)
            check('restored_esdf_has_no_old_queue_membership', describe(restored)['in_queue_voxels'] == 0)
            check('restored_esdf_only_blocks_survive', set(restored['esdf']) - set(restored['tsdf']) ==
                  set(saved['esdf']) - set(saved['tsdf']))
            summary['observations']['target_after_load'] = status('target')
            check('load_service_did_not_replay_scans_or_spheres', latest_status['target']['loaded_maps'] == 1 and
                  latest_status['target']['integrated_scans'] == 0 and latest_status['target']['esdf_clear_sphere_operations'] == 0)

            scan('target', 22, (3.125, .125, .125))
            continued = snapshot('target')
            final = status('target')
            summary['observations']['continued_state'] = describe(continued)
            check('loaded_integrators_accept_and_publish_new_sensor_data',
                  final['integrated_scans'] == 1 and final['esdf_clear_sphere_operations'] == 1 and
                  final['rejected_scans'] == 0 and final['rejected_layers'] == 0 and
                  (3, 0, 0) in continued['tsdf'] and (3, 0, 0) not in restored['tsdf'] and
                  any(as_float(continued['tsdf'][(3, 0, 0)][i]) > 0 for i in range(1, 3 * side ** 3, 3)))
            check('unrelated_mixed_parent_block_survives_resumed_fusion',
                  normalized('esdf', {special_index: continued['esdf'][special_index]}) ==
                  normalized('esdf', {special_index: restored['esdf'][special_index]}))
            summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
    finally:
        for name in names:
            stop(name)
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

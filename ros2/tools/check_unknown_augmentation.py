#!/usr/bin/env python3
"""Check native unknown-box transactions, build-count window and map ordering.

Runs one mapper and one local planner in an isolated ROS namespace. The probe
is the sole publisher of the planner's map-input topic, so withheld snapshots
really are unseen by the planner. Service snapshots still travel directly
from mapper to planner. This checks communication and map semantics, not
exploration performance or robot dynamics.
"""
import argparse
import json
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
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))

    import rclpy
    from gbplanner_voxblox.srv import FilePath
    from geometry_msgs.msg import Pose, TransformStamped
    from nav_msgs.msg import Odometry
    from planner_msgs.msg import VoxelMapUpdate
    from planner_msgs.srv import AugmentFreeBox, PlannerSrv
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    summary = {'status': 'failed', 'scope': 'native mapper/local-planner communication; no dynamics',
               'checks': {}, 'observations': {}}
    namespace = '/unknown_augmentation'
    processes, logs, stopped = {}, [], set()
    updates, raw_maps, accepted, metrics = {}, [], [], []
    forwarding = [True]
    probe = None
    started = time.monotonic()
    deadline = started + 25.

    def identity(value):
        return value.source_id, int(value.map_epoch), int(value.update_id)

    def sequence(status):
        return status['source_id'], status['map_epoch'], status['update_id']

    def stamp_ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def records(message):
        offsets = {field.name: field.offset for field in message.fields}
        result = {}
        for row in range(message.height):
            for col in range(message.width):
                offset = row * message.row_step + col * message.point_step
                xyz = tuple(struct.unpack_from('<f', message.data, offset + offsets[name])[0]
                            for name in ('x', 'y', 'z'))
                occupancy = struct.unpack_from('b', message.data, offset + offsets['occupancy'])[0]
                distance, weight = (struct.unpack_from('<f', message.data, offset + offsets[name])[0]
                                    for name in ('tsdf_distance', 'tsdf_weight'))
                result[xyz] = (occupancy, distance, weight)
        return result

    def same_cloud(a, b):
        return (a.header == b.header and a.fields == b.fields and
                a.width == b.width and a.height == b.height and
                a.point_step == b.point_step and a.row_step == b.row_step and
                a.is_bigendian == b.is_bigendian and a.is_dense == b.is_dense and
                bytes(a.data) == bytes(b.data))

    def check(name, condition):
        summary['checks'][name] = bool(condition)
        if not condition:
            raise AssertionError(name)

    def spin_until(predicate, label):
        while True:
            for name, process in processes.items():
                if name not in stopped and process.poll() is not None:
                    raise RuntimeError(f'{name} exited ({process.returncode})')
            if time.monotonic() >= deadline:
                raise TimeoutError(label)
            value = predicate()
            if value:
                return value
            rclpy.spin_once(probe, timeout_sec=.01)

    def settle(seconds):
        end = time.monotonic() + seconds
        spin_until(lambda: time.monotonic() >= end, 'Bounded message delivery')

    def call(client, request, label):
        spin_until(client.service_is_ready, label + ' discovery')
        future = client.call_async(request)
        spin_until(future.done, label)
        response = future.result()
        if response is None:
            raise RuntimeError(label + ': empty response')
        return response

    def status(client):
        return json.loads(call(client, Trigger.Request(), 'Status response').message)

    def wait_status(client, predicate, label):
        while True:
            value = status(client)
            if predicate(value):
                return value
            settle(.02)
            if time.monotonic() >= deadline:
                raise TimeoutError(label)

    def stop(name):
        process = processes.get(name)
        if process is None or name in stopped:
            return
        stopped.add(name)
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        try:
            process.wait(timeout=2.)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=1.)
        summary['checks'][name + '_exited_cleanly'] = process.returncode == 0

    def pose(x, y=.1, z=.1):
        value = Pose()
        value.position.x, value.position.y, value.position.z = float(x), float(y), float(z)
        value.orientation.w = 1.
        return value

    try:
        with tempfile.TemporaryDirectory(prefix='gbplanner-unknown-box-') as temporary:
            trace = Path(temporary) / 'planner_trace.jsonl'
            params = Path(temporary) / 'params.yaml'
            common = {'map.storage': 'sparse', 'map.resolution': .2}
            params.write_text(json.dumps({
                namespace + '/gbplanner_local': {'ros__parameters': dict(common, **{
                    'map.use_update_envelope': True,
                    'planning.augment_free_voxels': True,
                    'planning.augment_free_voxels_time': 1,
                    'robot.size': [.4, .4, .4], 'global.enable': False,
                    'planning.global_min': [-10., -4., -4.],
                    'planning.global_max': [12., 4., 4.],
                    'sampling.min': [-.3, -.3, -.3], 'sampling.max': [.3, .3, .3],
                    'planning.max_vertices': 2, 'planning.max_edges': 2,
                    'planning.max_attempts': 12, 'planning.empty_graph_cutoff': 1,
                    'planning.edge_min': .05, 'planning.edge_max': .3, 'planning.overshoot': 0.,
                    'sensor.range': .6, 'sensor.horizontal_fov': 1.2, 'sensor.vertical_fov': .8,
                    'sensor.horizontal_resolution': .4, 'sensor.vertical_resolution': .4,
                    'diagnostics.map_trace_file': str(trace),
                })},
                namespace + '/voxblox_mapper': {'ros__parameters': dict(common, **{
                    'sensor_frame': 'lidar', 'use_tf_transforms': False,
                    'robot.size': [.2, .2, .2], 'tsdf.method': 'simple',
                    'tsdf.voxels_per_side': 4, 'tsdf.use_const_weight': True,
                    'publish_map_every_n_sec': 0., 'update_mesh_every_n_sec': 0.,
                })},
            }))
            rclpy.init(args=[])
            probe = Node('unknown_augmentation_probe', namespace=namespace)
            durable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            bridge = probe.create_publisher(VoxelMapUpdate, 'voxel_map_update', durable)

            def receive_update(message):
                updates[identity(message)] = message
                if forwarding[0]:
                    bridge.publish(message)

            probe.create_subscription(VoxelMapUpdate, 'mapper_voxel_map_update', receive_update, durable)
            probe.create_subscription(PointCloud2, 'voxel_map', raw_maps.append, durable)
            probe.create_subscription(PointCloud2, 'accepted_voxel_map', accepted.append, durable)
            probe.create_subscription(String, 'planning_metrics', lambda m: metrics.append(json.loads(m.data)), durable)
            odometry = probe.create_publisher(Odometry, 'odometry', 10)
            transforms = probe.create_publisher(TransformStamped, 'transform', 10)
            clouds = probe.create_publisher(PointCloud2, 'points', 10)
            augmentation = probe.create_client(AugmentFreeBox, 'augment_free_box')
            planning = probe.create_client(PlannerSrv, 'planner')
            augmentation_status = probe.create_client(Trigger, 'unknown_augmentation_status')
            mapper_status = probe.create_client(Trigger, 'mapper_status')
            map_status = probe.create_client(Trigger, 'map_status')
            reset = probe.create_client(Trigger, 'reset_map')
            save = probe.create_client(FilePath, 'save_map')
            load = probe.create_client(FilePath, 'load_map')

            for name, package, executable in (
                    ('mapper', 'gbplanner_voxblox', 'voxblox_mapper_node'),
                    ('planner', 'gbplanner_local', 'local_planner_node')):
                log = args.output.with_name(args.output.stem + '_' + name + '.log').open('w')
                logs.append(log)
                command = ['ros2', 'run', package, executable, '--ros-args', '-r', '__ns:=' + namespace,
                           '--params-file', str(params)]
                if name == 'mapper':
                    command += ['-r', 'voxel_map_update:=mapper_voxel_map_update']
                processes[name] = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                                   start_new_session=True, env=dict(os.environ))
            clients = (augmentation, planning, augmentation_status, mapper_status, map_status, reset, save, load)
            spin_until(lambda: all(client.service_is_ready() for client in clients) and
                       bridge.get_subscription_count() == 1 and odometry.get_subscription_count() >= 2 and
                       transforms.get_subscription_count() > 0 and clouds.get_subscription_count() > 0,
                       'Mapper/planner discovery')

            def box(center, size, snapshot=True):
                request = AugmentFreeBox.Request()
                request.header.frame_id = 'map'
                request.header.stamp = probe.get_clock().now().to_msg()
                request.center.x, request.center.y, request.center.z = map(float, center)
                request.size.x, request.size.y, request.size.z = map(float, size)
                request.return_snapshot = snapshot
                response = call(augmentation, request, 'Unknown-box service')
                if not response.success:
                    raise RuntimeError(response.reason)
                if snapshot:
                    key = identity(response)
                    spin_until(lambda: key in updates, 'Matching map-envelope publication')
                    check('service_and_topic_snapshot_match_' + str(response.update_id),
                          identity(response.snapshot) == key and
                          same_cloud(response.snapshot.map, updates[key].map) and
                          response.snapshot.observation_id == updates[key].observation_id and
                          response.snapshot.observation_stamp == updates[key].observation_stamp)
                return response

            def accepted_sequence(message):
                current = wait_status(augmentation_status, lambda s: sequence(s) == identity(message),
                                      'Planner snapshot acceptance')
                envelope = message.snapshot if hasattr(message, 'snapshot') else message
                spin_until(lambda: any(same_cloud(envelope.map, cloud) for cloud in accepted),
                           'Exact accepted map snapshot')
                return current

            def odom(x):
                message = Odometry()
                message.header.frame_id, message.child_frame_id = 'map', 'base_link'
                message.header.stamp = probe.get_clock().now().to_msg()
                message.pose.pose = pose(x)
                odometry.publish(message)
                timestamp = stamp_ns(message.header.stamp)

                def consumed():
                    if not trace.exists():
                        return False
                    for line in trace.read_text().splitlines():
                        event = json.loads(line)
                        if event['phase'] == 'odometry_completed' and event['stamp_ns'] == timestamp:
                            return True
                    return False

                spin_until(consumed, 'Planner odometry callback completion')
                return timestamp

            def plan(x):
                before = len(metrics)
                request = PlannerSrv.Request()
                request.header.frame_id, request.bound_mode, request.root_pose = 'map', 3, pose(x)
                request.header.stamp = probe.get_clock().now().to_msg()
                response = call(planning, request, 'Explicit-root local planning')
                spin_until(lambda: len(metrics) > before, 'Planning metrics')
                return response, metrics[-1]

            first = box((.19, .19, .19), (.2, .2, .2))
            first_status = accepted_sequence(first)
            first_records = records(first.snapshot.map)
            expected_centers = {(x, y, z) for x in (-.1, .1) for y in (-.1, .1) for z in (-.1, .1)}
            check('fractional_center_uses_source_lattice', first.changed_voxels == 8 and
                  {tuple(round(axis, 5) for axis in point) for point in first_records} == expected_centers)
            check('initial_augmentation_has_no_sensor_observation', first.map_epoch == 1 and
                  first.snapshot.observation_id == first_status['observation_revision'] == 0)
            spin_until(lambda: any(same_cloud(first.snapshot.map, message) for message in raw_maps),
                       'Legacy map publication')
            check('legacy_cloud_keeps_six_fields_and_24_bytes', first.snapshot.map.point_step == 24 and
                  len(first.snapshot.map.fields) == 6)
            repeated = box((.19, .19, .19), (.2, .2, .2))
            accepted_sequence(repeated)
            check('unchanged_requested_snapshot_is_new_commit', repeated.changed_voxels == 0 and
                  repeated.reason == 'no_change' and repeated.update_id > first.update_id and
                  records(repeated.snapshot.map) == first_records and repeated.snapshot.observation_id == 0)
            no_snapshot = box((.19, .19, .19), (.2, .2, .2), False)
            check('unchanged_async_box_does_not_republish', no_snapshot.changed_voxels == 0 and
                  no_snapshot.update_id == repeated.update_id and no_snapshot.snapshot.source_id == '')

            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = 'map', 'tracking'
            transform.header.stamp = probe.get_clock().now().to_msg()
            transform.transform.translation.x = 2.1
            transform.transform.translation.y = transform.transform.translation.z = .1
            transform.transform.rotation.w = 1.
            cloud = PointCloud2()
            cloud.header.frame_id, cloud.header.stamp = 'lidar', transform.header.stamp
            cloud.width, cloud.height, cloud.point_step, cloud.row_step = 25, 1, 12, 300
            cloud.is_dense = True
            cloud.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                            for i, name in enumerate(('x', 'y', 'z'))]
            cloud.data = b''.join(struct.pack('<fff', 2., row * .08, col * .08)
                                  for row in range(-2, 3) for col in range(-2, 3))
            transforms.publish(transform)
            clouds.publish(cloud)
            observed = spin_until(lambda: next((m for m in updates.values() if m.change_kind == 'observation'), None),
                                  'Physical scan integration')
            observed_status = accepted_sequence(observed)
            measured = records(observed.map)
            # The planning cloud uses VoxelStatus / OccupancyGrid values:
            # Unknown=-1, Free=0, Occupied=100 (not a boolean 0/1 encoding).
            measured_free = {key: value for key, value in measured.items() if value[0] == 0 and value[2] > .01}
            measured_occupied = {key: value for key, value in measured.items() if value[0] == 100 and value[2] > .01}
            filled = box((3.1, .1, .1), (2.4, 1.2, 1.2))
            filled_status = accepted_sequence(filled)
            after_fill = records(filled.snapshot.map)
            check('real_scan_advances_observation_once', observed.observation_id == 1 and
                  stamp_ns(observed.observation_stamp) == stamp_ns(cloud.header.stamp) and
                  status(mapper_status)['integrated_scans'] == 1 and observed_status['observation_revision'] > 0)
            changed_existing = [{'center': key, 'before': value, 'after': after_fill.get(key)}
                                for key, value in measured.items() if after_fill.get(key) != value]
            summary['observations']['physical_scan'] = {
                'free_cells': len(measured_free), 'occupied_cells': len(measured_occupied),
                'known_before': len(measured), 'known_after': len(after_fill),
                'augmentation_added': filled.changed_voxels,
                'changed_existing_cells': len(changed_existing),
                'changed_existing_samples': changed_existing[:5],
            }
            check('box_preserves_measured_free_and_occupied_tsdf', bool(measured_free) and bool(measured_occupied) and
                  filled.changed_voxels > 0 and not changed_existing)
            check('box_does_not_advance_sensor_revision', filled.snapshot.observation_id == 1 and
                  filled.snapshot.observation_stamp == observed.observation_stamp and
                  filled_status['observation_revision'] == observed_status['observation_revision'])
            before_odom = status(augmentation_status)
            odom(-1.9)
            odom_status = wait_status(augmentation_status, lambda s: s['requests'] > before_odom['requests'] and
                                     not s['odometry_pending'] and s['update_id'] > filled.update_id,
                                     'Single odometry transaction completion')
            check('single_odometry_finishes_without_another_sample', odom_status['requests'] == before_odom['requests'] + 1 and
                  odom_status['changed_voxels'] > before_odom['changed_voxels'] and odom_status['failures'] == 0)
            _, build_metrics = plan(-1.9)
            closed = status(augmentation_status)
            check('one_completed_graph_build_closes_window', closed['completed_builds'] == 1 and
                  closed['build_limit'] == 1 and not closed['odometry_window_open'] and build_metrics['attempts'] > 0)
            before_move = status(map_status)
            moved_stamp = odom(-4.9)
            settle(.08)
            after_move = status(augmentation_status)
            check('odometry_after_window_cannot_fill_unknown_cells', after_move['requests'] == closed['requests'] and
                  after_move['changed_voxels'] == closed['changed_voxels'] and
                  status(map_status)['known_cells'] == before_move['known_cells'])
            summary['observations']['window'] = {'first_build': build_metrics, 'closed': closed,
                'moved_odometry_completed_stamp_ns': moved_stamp, 'after_move': after_move}

            # The topic is now withheld. Root augmentation can only update the
            # planner through the service's synchronous, complete snapshot.
            forwarding[0] = False
            root_before, root_map_before = status(augmentation_status), status(map_status)
            _, root_metrics = plan(6.1)
            root_after, root_map_after = status(augmentation_status), status(map_status)
            check('closed_window_future_root_accepts_service_snapshot',
                  root_after['requests'] == root_before['requests'] + 1 and
                  root_after['changed_voxels'] > root_before['changed_voxels'] and
                  root_after['failures'] == root_before['failures'] and
                  root_after['completed_builds'] == 2 and root_metrics['attempts'] > 0 and
                  root_map_after['known_cells'] > root_map_before['known_cells'] and
                  root_after['observation_revision'] == root_before['observation_revision'] and
                  root_after['observation_id'] == 1)
            summary['observations']['future_root'] = {'before': root_before, 'after': root_after,
                'known_before': root_map_before['known_cells'], 'known_after': root_map_after['known_cells'],
                'planning': root_metrics}

            path = Path(temporary) / 'map.vxblx'
            check('map_save_succeeded', call(save, FilePath.Request(file_path=str(path)), 'Save map').success)
            check('map_reset_succeeded', call(reset, Trigger.Request(), 'Reset map').success)
            reset_status = status(mapper_status)
            old_key = (reset_status['map_source_id'], reset_status['map_epoch'], reset_status['map_update_id'])
            spin_until(lambda: old_key in updates, 'Withheld reset snapshot')
            old_unseen = updates[old_key]
            check('map_load_succeeded', call(load, FilePath.Request(file_path=str(path)), 'Load map').success)
            loaded = status(mapper_status)
            check('reset_load_keep_source_and_advance_epoch_and_update',
                  old_unseen.source_id == loaded['map_source_id'] == root_after['source_id'] and
                  old_unseen.map_epoch == root_after['map_epoch'] + 1 and
                  loaded['map_epoch'] == old_unseen.map_epoch + 1 and
                  root_after['update_id'] < old_unseen.update_id < loaded['map_update_id'] and
                  old_unseen.observation_id == loaded['map_observation_id'] == 0 and
                  stamp_ns(old_unseen.observation_stamp) == 0)
            check('intermediate_epoch_really_withheld', sequence(status(augmentation_status)) == sequence(root_after))
            _, epoch_metrics = plan(8.1)
            new_epoch, new_map = status(augmentation_status), status(map_status)
            check('service_snapshot_can_skip_unseen_epoch', new_epoch['source_id'] == root_after['source_id'] and
                  new_epoch['map_epoch'] == loaded['map_epoch'] and new_epoch['update_id'] > loaded['map_update_id'] and
                  new_epoch['observation_id'] == new_epoch['observation_revision'] == 0 and
                  new_epoch['requests'] == root_after['requests'] + 1 and
                  new_epoch['failures'] == root_after['failures'] and new_epoch['completed_builds'] == 3 and
                  new_map['known_cells'] > root_map_after['known_cells'] and epoch_metrics['attempts'] > 0)

            # Obtain a later unchanged snapshot as an acceptance barrier, then
            # stop the real mapper. Only this one bridge writer can deliver B/D.
            spin_until(lambda: sequence(new_epoch) in updates, 'New epoch service snapshot publication')
            barrier_reply = box((8.1, .1, .1), (.4, .4, .4))
            barrier = barrier_reply.snapshot
            check('barrier_keeps_map_and_observation', barrier_reply.changed_voxels == 0 and
                  barrier.update_id > new_epoch['update_id'] and barrier.map_epoch == new_epoch['map_epoch'] and
                  barrier.observation_id == 0 and
                  records(barrier.map) == records(updates[sequence(new_epoch)].map))
            stop('mapper')
            before_replay = status(augmentation_status)
            summary['observations']['ordering'] = {'old_unseen': list(identity(old_unseen)),
                'synchronous_new': list(sequence(new_epoch)), 'before_replay': before_replay,
                'barrier': list(identity(barrier))}

            def old_replay_ignored(value):
                return (value['map_envelopes_received'] > before_replay['map_envelopes_received'] and
                        value['map_envelopes_ignored'] > before_replay['map_envelopes_ignored'] and
                        (value['last_ignored_source_id'], value['last_ignored_epoch'],
                         value['last_ignored_update_id']) == identity(old_unseen))

            bridge.publish(old_unseen)
            # Confirm this exact envelope reached the planner's sequence gate.
            # DDS acknowledgements and elapsed time cannot prove callback entry.
            after_old = wait_status(augmentation_status, old_replay_ignored,
                                    'Older-epoch envelope received and ignored')
            check('old_epoch_replay_received_and_ignored', old_replay_ignored(after_old))
            after_old_map = status(map_status)
            summary['observations']['ordering']['after_old'] = after_old
            check('unseen_older_epoch_cannot_roll_back_service_snapshot',
                  sequence(after_old) == sequence(new_epoch) and after_old['map_revision'] == new_epoch['map_revision'] and
                  after_old_map['known_cells'] == new_map['known_cells'])
            bridge.publish(barrier)
            after_barrier = accepted_sequence(barrier)
            check('newer_same_epoch_topic_still_accepted_once',
                  after_barrier['map_revision'] == new_epoch['map_revision'] + 1 and
                  after_barrier['observation_revision'] == 0 and status(map_status)['known_cells'] == new_map['known_cells'])
            summary['observations']['ordering']['after_barrier'] = after_barrier
            summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
    finally:
        for name in reversed(processes):
            stop(name)
        for log in logs:
            log.close()
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if not all(summary['checks'].values()):
            summary['status'] = 'failed'
        summary['elapsed_seconds'] = round(time.monotonic() - started, 3)
        args.output.write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

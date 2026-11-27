#!/usr/bin/env python3
"""Check planner snapshot-cap diagnostics and replay a captured real snapshot."""
import argparse
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


def spin_until(node, predicate, timeout_s, description):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(.05, max(0., deadline-time.monotonic())))
        value = predicate()
        if value:
            return value
    raise TimeoutError(f'Timed out waiting for {description}')


class Probe(Node):
    def __init__(self):
        super().__init__('map_capacity_probe')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(PointCloud2, 'voxel_map', qos)
        self.status_client = self.create_client(Trigger, 'map_status')

    def status(self):
        future = self.status_client.call_async(Trigger.Request())
        spin_until(self, future.done, 10., 'map_status response')
        result = future.result()
        if result is None:
            raise RuntimeError('map_status returned no response')
        return json.loads(result.message)

    def publish_and_wait(self, message, predicate, timeout_s, description):
        spin_until(self, lambda: self.publisher.get_subscription_count() > 0,
                   10., 'planner voxel_map subscription')
        self.publisher.publish(message)
        return spin_until(self, predicate, timeout_s, description)


def fields_from(metadata):
    result = []
    for field in metadata['fields']:
        result.append(PointField(name=field['name'], offset=field['offset'],
                                 datatype=field['datatype'], count=field['count']))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True,
                        help='Raw PointCloud2.data bytes from a captured real accepted snapshot')
    parser.add_argument('--metadata', type=Path, required=True,
                        help='cloud_metadata object from the same captured snapshot')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-snapshot-voxels', type=int, default=16_000_000)
    parser.add_argument('--binary', type=Path, help='Optional retained node for a paired snapshot import')
    parser.add_argument('--library-dir', type=Path, help='Matching retained shared-library directory')
    args = parser.parse_args()
    if not 1 <= args.max_snapshot_voxels <= 100_000_000:
        parser.error('--max-snapshot-voxels must be in [1, 100000000]')
    args.snapshot = args.snapshot.resolve()
    args.metadata = args.metadata.resolve()
    args.output = args.output.resolve()
    if args.output.exists():
        parser.error(f'Refusing to overwrite {args.output}')

    metadata_document = json.loads(args.metadata.read_text())
    metadata = metadata_document.get('accepted_map', metadata_document)
    expected_bytes = metadata['row_step'] * metadata['height']
    if (metadata['frame_id'] != 'map' or metadata['height'] < 1 or
            metadata['point_step'] < 1 or metadata['width'] * metadata['height'] > args.max_snapshot_voxels or
            expected_bytes != args.snapshot.stat().st_size):
        parser.error('Snapshot metadata and raw bytes do not form a supported configured input')
    source_sha = hashlib.sha256(args.snapshot.read_bytes()).hexdigest()
    if metadata.get('sha256') and source_sha != metadata['sha256']:
        parser.error('Snapshot bytes do not match the metadata SHA256')
    snapshot_bytes = args.snapshot.read_bytes()
    occupancy_offset = next((f['offset'] for f in metadata['fields']
                             if f['name'] == 'occupancy'), None)
    if occupancy_offset is None or occupancy_offset >= metadata['point_step']:
        parser.error('Snapshot metadata is missing its occupancy field')
    occupancy_values = snapshot_bytes[occupancy_offset::metadata['point_step']]
    unknown_voxels = occupancy_values.count(255)
    expected_known_cells = metadata['width'] * metadata['height'] - unknown_voxels

    domain = os.environ.get('ROS_DOMAIN_ID', '149')
    os.environ['ROS_DOMAIN_ID'] = domain
    node_executable = (args.binary or Path('install_fortress/gbplanner_local/lib/gbplanner_local/local_planner_node')).resolve()
    if not node_executable.is_file():
        parser.error(f'Installed local planner node is unavailable: {node_executable}')
    command = [str(node_executable), '--ros-args',
               '-p', 'use_sim_time:=false', '-p', 'map.storage:=sparse',
               '-p', 'map.resolution:=0.1', '-p',
               f'map.max_snapshot_voxels:={args.max_snapshot_voxels}', '-p',
               'diagnostics.map_trace_file:='+str(args.output.with_suffix('.map_trace.jsonl'))]
    environment = os.environ.copy()
    if args.library_dir:
        environment['LD_LIBRARY_PATH'] = str(args.library_dir.resolve())+os.pathsep+environment.get('LD_LIBRARY_PATH', '')
    process = subprocess.Popen(command, start_new_session=True, env=environment)
    node = None
    result = {'status': 'failed', 'domain': domain, 'command': command,
              'configured_max_snapshot_voxels': args.max_snapshot_voxels,
              'captured_snapshot': {'path': str(args.snapshot), 'metadata': metadata,
                                    'sha256': source_sha}}
    started = time.monotonic()
    try:
        rclpy.init()
        node = Probe()
        spin_until(node, node.status_client.service_is_ready, 20., 'local planner map_status service')
        initial = node.status()
        if initial.get('max_snapshot_voxels') != args.max_snapshot_voxels:
            raise AssertionError(f'Configured capacity was not read back: {initial}')

        oversized_count = args.max_snapshot_voxels + 1
        oversize = PointCloud2()
        oversize.header.frame_id = 'map'
        oversize.height = 1
        oversize.width = oversized_count
        oversize.point_step = 24
        oversize.row_step = oversized_count * oversize.point_step
        over_status = node.publish_and_wait(oversize, lambda: (
            value if (value := node.status()).get('rejected_maps', 0) >= 1 else None),
            10., 'oversized snapshot rejection')
        expected_reason = (f'Map snapshot voxel limit exceeded: {oversized_count} > '
                           f'map.max_snapshot_voxels={args.max_snapshot_voxels}')
        if over_status.get('last_rejected_map_reason') != expected_reason:
            raise AssertionError(f'Unexpected capacity diagnostic: {over_status}')

        message = PointCloud2()
        message.header.frame_id = metadata['frame_id']
        message.header.stamp.sec = metadata['stamp']['sec']
        message.header.stamp.nanosec = metadata['stamp']['nanosec']
        message.height = metadata['height']
        message.width = metadata['width']
        message.fields = fields_from(metadata)
        message.is_bigendian = metadata['is_bigendian']
        message.point_step = metadata['point_step']
        message.row_step = metadata['row_step']
        message.is_dense = metadata['is_dense']
        message.data = snapshot_bytes
        import_started = time.monotonic()
        accepted = node.publish_and_wait(message, lambda: (
            value if (value := node.status()).get('map_revision', 0) >= 1 else None),
            60., 'captured snapshot acceptance')
        if (accepted.get('known_cells') != expected_known_cells or
                accepted.get('last_rejected_map_reason') != expected_reason):
            raise AssertionError(f'Replayed snapshot did not replace the map intact: {accepted}')
        result.update(status='passed', elapsed_s=time.monotonic()-started,
                      import_wall_s=time.monotonic()-import_started,
                      binary_sha256=hashlib.sha256(node_executable.read_bytes()).hexdigest(),
                      oversize_status=over_status, accepted_map_status=accepted,
                      accepted_snapshot_voxels=metadata['width'],
                      accepted_snapshot_expected_known_cells=expected_known_cells,
                      captured_snapshot_unknown_voxels=unknown_voxels,
                      accepted_snapshot_above_legacy_limit=metadata['width'] > 4_000_000)
    except Exception as error:
        result['error'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.)
        result['planner_exit_code'] = process.returncode
        result['clean_shutdown'] = process.returncode == 0
        if result['status'] == 'passed' and not result['clean_shutdown']:
            result['status'] = 'failed'
            result['error'] = 'Planner did not exit cleanly after SIGINT'
        result['elapsed_s'] = time.monotonic()-started
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()

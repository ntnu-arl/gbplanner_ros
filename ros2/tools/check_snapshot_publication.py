#!/usr/bin/env python3
"""Exercise snapshot batching with a small explicit two-lidar ROS fixture."""
import argparse
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from planner_msgs.msg import VoxelMapUpdate
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output = args.output.resolve()
    if args.output.exists():
        parser.error('Use a new output path')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', '147')
    root = Path(__file__).resolve().parents[1]
    binary = root/'install_fortress/gbplanner_voxblox/lib/gbplanner_voxblox/voxblox_mapper_node'
    command = [str(binary), '--ros-args', '-p', 'map.storage:=sparse',
               '-p', 'map.snapshot_publish_period:=1.0', '-p', 'input_sensors:=[front,rear]',
               '-p', 'input.front.topic:=front/points', '-p', 'input.front.frame:=front_lidar',
               '-p', 'input.rear.topic:=rear/points', '-p', 'input.rear.frame:=rear_lidar',
               '-p', 'tsdf.method:=simple', '-p', 'tsdf.threads:=1']
    report = {'status': 'failed', 'scope': 'Small synthetic ROS sensor fixture; no simulator or native-mine claim',
              'command': command, 'domain': os.environ['ROS_DOMAIN_ID'], 'states': {}}
    process = node = None
    started = time.monotonic()
    log = args.output.with_suffix('.log').open('w')
    try:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = Node('snapshot_publication_probe')
        updates = []
        durable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(VoxelMapUpdate, 'voxel_map_update', updates.append, durable)
        clients = {name: node.create_client(Trigger, name) for name in
                   ('mapper_status', 'publish_pointclouds', 'reset_map')}
        publishers = {name: node.create_publisher(PointCloud2, name+'/points', qos_profile_sensor_data)
                      for name in ('front', 'rear')}
        def until(predicate, timeout=8.):
            deadline = time.monotonic()+timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.01)
                value = predicate()
                if value:
                    return value
            raise TimeoutError('Fixture condition not observed')
        def call(name):
            future = clients[name].call_async(Trigger.Request())
            until(future.done)
            return future.result()
        def status():
            return json.loads(call('mapper_status').message)
        def record(name):
            result = status();report['states'][name] = result;return result
        def idle(seconds):
            end = time.monotonic()+seconds
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.02)
        until(lambda: all(client.service_is_ready() for client in clients.values()) and
              all(pub.get_subscription_count() for pub in publishers.values()))
        broadcaster = StaticTransformBroadcaster(node)
        transforms = []
        for name in publishers:
            transform = TransformStamped()
            transform.header.frame_id = 'map';transform.child_frame_id = name+'_lidar'
            transform.transform.rotation.w = 1.
            transforms.append(transform)
        broadcaster.sendTransform(transforms)
        idle(.1)
        def scan(name, stamp):
            cloud = PointCloud2()
            cloud.header.frame_id = name+'_lidar';cloud.header.stamp = stamp
            cloud.height = 1;cloud.width = 4;cloud.point_step = 12;cloud.row_step = 48
            cloud.is_dense = True
            cloud.fields = [PointField(name=n, offset=i*4, datatype=7, count=1)
                            for i, n in enumerate(('x', 'y', 'z'))]
            cloud.data = b''.join(struct.pack('<fff', *p) for p in
                                  ((1.,0.,0.), (1.,.5,0.), (1.,-.5,0.), (1.,0.,.5)))
            publishers[name].publish(cloud)
        scan('front', node.get_clock().now().to_msg())
        until(lambda: status()['integrated_scans'] == 1)
        initial = record('initial_scan')
        assert initial['snapshot_publications'] == 1 and initial['map_observation_id'] == 1
        stamp = node.get_clock().now().to_msg()
        scan('front', stamp);scan('rear', stamp)
        until(lambda: status()['integrated_scans'] == 3)
        final_stamp = node.get_clock().now().to_msg()
        scan('front', final_stamp);scan('rear', final_stamp)
        until(lambda: status()['integrated_scans'] == 5)
        pending = record('five_integrated')
        assert pending['pending_observation_scans'] == 4 and pending['snapshot_publications'] == 1
        until(lambda: status()['snapshot_publications'] == 2)
        flushed = record('stopped_input_flushed')
        assert flushed['integrated_scans'] == 5 and flushed['pending_observation_scans'] == 0
        assert flushed['coalesced_observation_scans'] == 3 and flushed['map_observation_id'] == 2
        until(lambda: updates and updates[-1].update_id == 2)
        assert updates[-1].observation_stamp == final_stamp and updates[-1].map.header.stamp == final_stamp
        idle(1.1)
        assert record('idle_no_new_observations')['snapshot_publications'] == 2
        assert call('publish_pointclouds').success
        explicit = record('explicit_snapshot')
        assert explicit['snapshot_publications'] == 3 and explicit['map_observation_id'] == 2
        stamp = node.get_clock().now().to_msg()
        scan('front', stamp);scan('rear', stamp)
        until(lambda: status()['integrated_scans'] == 7)
        before_reset = record('pending_before_reset')
        assert before_reset['pending_observation_scans'] == 2 and before_reset['snapshot_publications'] == 3
        assert call('reset_map').success
        reset = record('reset')
        assert reset['snapshot_publications'] == 4 and reset['map_epoch'] == 2
        assert reset['map_observation_id'] == 0 and reset['pending_observation_scans'] == 0
        idle(1.1)
        quiet = record('reset_not_revived')
        assert quiet['snapshot_publications'] == 4 and quiet['integrated_scans'] == 0 and quiet['known_cells'] == 0
        report['updates'] = [{'update_id': m.update_id, 'observation_id': m.observation_id,
                              'epoch': m.map_epoch, 'kind': m.change_kind,
                              'stamp_sec': m.observation_stamp.sec,
                              'stamp_nanosec': m.observation_stamp.nanosec,
                              'width': m.map.width} for m in updates]
        report['status'] = 'passed'
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
                    process.wait(timeout=8.)
                except subprocess.TimeoutExpired:
                    forced = True;os.killpg(process.pid, signal.SIGKILL);process.wait(timeout=2.)
            report['mapper_exit_code'] = process.returncode
            report['forced_kill'] = forced
            if process.returncode != 0:
                report['status'] = 'failed'
        report['elapsed_s'] = time.monotonic()-started
        log.close()
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({k: report.get(k) for k in
                          ('status', 'error', 'elapsed_s', 'mapper_exit_code', 'forced_kill')}))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

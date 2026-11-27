#!/usr/bin/env python3
"""Exercise the installed validation/mission executors without a simulator."""
import argparse
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from planner_msgs.srv import ValidatePath
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if not os.environ.get('ROS_DOMAIN_ID'):
        raise RuntimeError('Set an isolated ROS_DOMAIN_ID for this check')
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    binary = Path(get_package_prefix('gbplanner_local')) / 'lib/gbplanner_local/local_planner_node'
    config = Path(get_package_share_directory('gbplanner_gazebo')) / 'config/rmf_exploration.yaml'
    result = {'status': 'failed', 'domain': os.environ['ROS_DOMAIN_ID'], 'checks': []}
    log = output.with_suffix('.log').open('w')
    process = subprocess.Popen([str(binary), '--ros-args', '--params-file', str(config)],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    node = None
    try:
        rclpy.init()
        node = rclpy.create_node('path_validation_runtime_check')
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        clock = node.create_publisher(Clock, '/clock', 10)
        odometry = node.create_publisher(Odometry, 'odometry', 10)
        maps = node.create_publisher(PointCloud2, 'voxel_map', durable)
        revisions = []
        node.create_subscription(String, 'map_updates', lambda msg: revisions.append(json.loads(msg.data)), durable)
        started = time.monotonic()
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.pose.pose.position.z = .8
        odom.pose.pose.orientation.w = 1.

        def inputs():
            stamp = 1. + time.monotonic() - started
            message = Clock()
            message.clock.sec = int(stamp)
            message.clock.nanosec = int((stamp % 1.) * 1e9)
            clock.publish(message)
            odom.header.stamp = message.clock
            odometry.publish(odom)

        node.create_timer(.02, inputs)

        def wait(predicate, seconds=8.):
            deadline = time.monotonic() + seconds
            while not predicate():
                if process.poll() is not None:
                    raise RuntimeError(f'Planner exited: {process.returncode}')
                if time.monotonic() >= deadline:
                    raise TimeoutError('Runtime validation event did not arrive')
                rclpy.spin_once(node, timeout_sec=.02)

        validation = node.create_client(ValidatePath, 'validate_path')
        mission = node.create_client(Trigger, 'mission_state')
        status = node.create_client(Trigger, 'map_status')
        wait(lambda: all(client.service_is_ready() for client in (validation, mission, status)))

        def call(client, request):
            future = client.call_async(request)
            wait(future.done)
            return future.result()

        request = ValidatePath.Request()
        request.header.frame_id = 'map'
        for x in (0., .4):
            pose = Pose()
            pose.position.x, pose.position.z, pose.orientation.w = x, .8, 1.
            request.path.append(pose)
        assert call(validation, request).reason == 'map_missing'
        result['checks'].append('startup_and_map_missing_rpc')
        cloud = PointCloud2()
        cloud.header.frame_id = 'map'
        cloud.height = 1
        cloud.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z'))]
        cloud.fields.append(PointField(name='occupancy', offset=12, datatype=PointField.INT8, count=1))
        cloud.point_step = 16
        cloud.is_dense = True
        cloud.data = b''.join(struct.pack('<fffbxxx', (x+.5)*.2, (y+.5)*.2, (z+.5)*.2, 0)
                              for x in range(-5, 6) for y in range(-5, 6) for z in range(10))
        cloud.width = len(cloud.data) // 16
        cloud.row_step = len(cloud.data)
        cloud.header.stamp = odom.header.stamp
        maps.publish(cloud)
        wait(lambda: revisions and revisions[-1]['map_revision'] >= 1)
        for _ in range(10):
            # All three executors must keep servicing requests together.
            futures = [validation.call_async(request), mission.call_async(Trigger.Request()),
                       status.call_async(Trigger.Request())]
            wait(lambda: all(future.done() for future in futures))
            assert futures[0].result().valid, futures[0].result().reason
            assert futures[1].result().success and futures[2].result().success
        result['checks'].append('validation_mission_and_default_executors_responsive')
        odom.pose.pose.position.x = 3.
        wait(lambda: call(validation, request).reason == 'current_pose_not_free')
        result['checks'].append('independent_odometry_reaches_validation')
        result['map_revision'] = int(revisions[-1]['map_revision'])
        result['status'] = 'passed'
    except Exception as error:
        result['error'] = str(error)
        raise
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        result['exit_code'] = process.returncode
        if process.returncode != 0:
            result['status'] = 'failed'
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        log.close()
        output.write_text(json.dumps(result, indent=2) + '\n')
    if result['status'] != 'passed':
        raise RuntimeError(result)
    print(json.dumps(result))


if __name__ == '__main__':
    main()

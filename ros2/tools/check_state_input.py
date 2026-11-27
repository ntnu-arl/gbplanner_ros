#!/usr/bin/env python3
"""One stamped sample per upstream pose type, using the source adapter directly."""
import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source = Path(__file__).resolve().parents[1] / 'src/gbplanner_local/scripts/state_input_node.py'
    summary = {'status': 'failed', 'samples': []}
    rclpy.init(args=[])
    probe = Node('state_input_probe')
    broadcaster = StaticTransformBroadcaster(probe)
    try:
        for mode in ('pose', 'pose_stamped'):
            input_topic, output_topic = f'check_{mode}_input', f'check_{mode}_odometry'
            service_name = f'check_{mode}_status'
            received = []
            message_type = PoseWithCovarianceStamped if mode == 'pose' else PoseStamped
            publisher = probe.create_publisher(message_type, input_topic, 10)
            subscription = probe.create_subscription(Odometry, output_topic, received.append, 10)
            client = probe.create_client(Trigger, service_name)
            command = [sys.executable, str(source), '--ros-args',
                       '-p', f'mode:={mode}', '-p', f'input_topic:={input_topic}',
                       '-p', f'output_topic:={output_topic}', '-p', 'body_frame:=base_link',
                       '-r', f'state_input_status:={service_name}']
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            deadline = time.monotonic() + 8.

            def spin_until(predicate):
                while not predicate():
                    if process.poll() is not None:
                        raise RuntimeError('Adapter exited: ' + process.stdout.read()[-2000:])
                    if time.monotonic() > deadline:
                        raise TimeoutError(f'{mode} probe timed out')
                    rclpy.spin_once(probe, timeout_sec=.05)

            try:
                spin_until(lambda: publisher.get_subscription_count() > 0 and client.service_is_ready())
                stamp = probe.get_clock().now().to_msg()
                transform = TransformStamped()
                transform.header.frame_id, transform.child_frame_id = 'map', f'check_{mode}_world'
                transform.header.stamp = stamp
                transform.transform.translation.x = 1.
                transform.transform.translation.y = 2.
                transform.transform.translation.z = .5
                transform.transform.rotation.z = transform.transform.rotation.w = math.sqrt(.5)
                broadcaster.sendTransform(transform)
                # The matching TF is transient local, so no repeated input
                # samples or replacement timestamps are needed for discovery.
                ready_after = time.monotonic() + .2
                spin_until(lambda: time.monotonic() >= ready_after)
                message = message_type()
                message.header.frame_id, message.header.stamp = transform.child_frame_id, stamp
                pose = message.pose.pose if mode == 'pose' else message.pose
                pose.position.x = 1.
                pose.orientation.w = 2.  # The adapter must normalize this input.
                if mode == 'pose':
                    for i, value in enumerate((1., 2., 3., 4., 5., 6.)):
                        message.pose.covariance[i*7] = value
                publisher.publish(message)
                spin_until(lambda: len(received) > 0)
                future = client.call_async(Trigger.Request())
                spin_until(future.done)
                reply = future.result()
                status = json.loads(reply.message)
                output = received[0]
                p, q = output.pose.pose.position, output.pose.pose.orientation
                result = {
                    'mode': mode, 'published': status['published'],
                    'transformed': status['transformed'], 'rejected': status['rejected'],
                    'stamp_preserved': output.header.stamp == stamp,
                    'frame': output.header.frame_id, 'body_frame': output.child_frame_id,
                    'position': [p.x, p.y, p.z], 'quaternion': [q.x, q.y, q.z, q.w],
                }
                result['passed'] = (
                    reply.success and status['published'] == status['received'] == 1
                    and status['transformed'] == 1 and status['rejected'] == 0
                    and output.header.stamp == stamp and output.header.frame_id == 'map'
                    and output.child_frame_id == 'base_link'
                    and math.dist((p.x, p.y, p.z), (1., 3., .5)) < 1e-9
                    and math.dist((q.x, q.y, q.z, q.w), (0., 0., math.sqrt(.5), math.sqrt(.5))) < 1e-9)
                if mode == 'pose':
                    result['covariance_rotated'] = all(
                        abs(output.pose.covariance[i*7] - value) < 1e-9
                        for i, value in enumerate((2., 1., 3., 5., 4., 6.)))
                    result['passed'] &= result['covariance_rotated']
                summary['samples'].append(result)
            finally:
                process.send_signal(signal.SIGINT)
                try:
                    process.communicate(timeout=2.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.communicate(timeout=1.)
                probe.destroy_publisher(publisher)
                probe.destroy_subscription(subscription)
                probe.destroy_client(client)
        summary['status'] = 'passed' if all(item['passed'] for item in summary['samples']) else 'failed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

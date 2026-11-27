#!/usr/bin/env python3
"""Check adopted refinement markers on one real ROS planner with a synthetic map.

No trajectory consumer or physical simulation is started. The visualization
switch stays off; only the mod_path subscription enables diagnostic capture.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

sys.dont_write_bytecode = True
from check_planning_visualization import close, close_points, require, snapshot_record, stamp_ns, xyz


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=workspace/'results/refined_path_vis/runtime1.json')
    args = parser.parse_args()
    output = args.output.resolve()
    log_path = output.with_suffix('.planner.log')
    require(not output.exists() and not log_path.exists(), 'Existing evidence must not be overwritten')
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    process, probe, initialized = None, None, False
    started = time.monotonic()
    summary = {'status': 'failed', 'scope': __doc__, 'domain': os.environ['ROS_DOMAIN_ID'],
               'phases': {}, 'cleanup': {}}
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from planner_msgs.srv import PlannerSrv
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from visualization_msgs.msg import Marker, MarkerArray
        sys.path.insert(0, str(workspace/'src/gbplanner_local/scripts'))
        from local_demo import DIM, ORIGIN, RES, make_map

        rclpy.init(args=[])
        initialized = True
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        parameters = {
            'map.resolution': RES, 'map.origin': list(ORIGIN), 'map.dimensions': list(DIM),
            'global.enable': False, 'mission.enabled': False,
            'visualization.planning_diagnostics': False,
            'planning.path_safety_enhance_enable': True,
            'robot.safety_extension': [1., 1.5, 1.], 'robot.size': [.4, .4, .4],
            'planning.path_interpolation_distance': .5,
            'planning.traverse_length_max': 10., 'planning.traverse_time_max': 100.,
            'planning.adaptive_exploration': False,
            'sampling.seed': 42, 'sampling.seed_mode': 'fixed',
            'sampling.min': [-3., -3., -1.], 'sampling.max': [3., 3., 1.],
            'planning.max_vertices': 100, 'planning.max_edges': 2000, 'planning.max_attempts': 4000,
            'planning.path_length_penalty': .07, 'planning.path_direction_penalty': 0.,
        }
        summary['parameters'] = parameters

        class Probe(Node):
            def __init__(self):
                super().__init__('probe', namespace=f'/refined_path_{os.getpid()}')
                self.rpc_clients, self.markers = {}, []
                self.maps = self.create_publisher(PointCloud2, 'voxel_map', durable)
                self.odometry = self.create_publisher(Odometry, 'odometry', 10)

            def until(self, predicate, label, seconds=20.):
                deadline = min(started+120., time.monotonic()+seconds)
                while not predicate():
                    require(process.poll() is None, f'Planner exited {process.returncode}')
                    if time.monotonic() >= deadline:
                        raise TimeoutError(label)
                    rclpy.spin_once(self, timeout_sec=.02)

            def call(self, name, request, kind=Trigger):
                if name not in self.rpc_clients:
                    self.rpc_clients[name] = self.create_client(kind, name)
                client = self.rpc_clients[name]
                self.until(client.service_is_ready, name+' discovery')
                future = client.call_async(request)
                self.until(future.done, name+' response', 40.)
                require(future.result() is not None, name+' empty response')
                return future.result()

            def status(self, name):
                return json.loads(self.call(name, Trigger.Request()).message)

            def metadata(self, since, request):
                # Subscribe only after the service has completed, so this
                # diagnostic subscription cannot enable the tested capture.
                received = []
                subscription = self.create_subscription(String, 'refinement_geometry',
                    lambda message:received.append(json.loads(message.data)), durable)
                try:
                    self.until(lambda:received and received[-1]['stamp_ns'] >= since and
                        received[-1]['request'] == request, 'Retained refinement metadata')
                    return received[-1]
                finally:
                    self.destroy_subscription(subscription)

            def plan(self, frame='map'):
                request = PlannerSrv.Request()
                request.header.frame_id, request.bound_mode = frame, 3
                request.root_pose.orientation.w = 0.
                self.until(lambda:self.count_subscribers('refinement_geometry') == 0,
                           'Metadata subscriber removed before planning')
                since = self.get_clock().now().nanoseconds
                return since, self.call('planner', request, PlannerSrv)

            def snapshot(self, metadata, cleared=False):
                self.until(lambda:self.markers and self.markers[-1].markers and
                    stamp_ns(self.markers[-1].markers[0].header) == metadata['stamp_ns'],
                    'Marker and refinement metadata share one snapshot')
                snapshot = self.markers[-1]
                require(snapshot.markers[0].action == Marker.DELETEALL, 'Snapshot does not clear stale geometry')
                for marker in snapshot.markers:
                    require(marker.header.frame_id == 'map' and stamp_ns(marker.header) == metadata['stamp_ns'],
                            'Snapshot frame or timestamp differs')
                if cleared:
                    require(len(snapshot.markers) == 1 and not metadata['accepted_path'],
                            'Failure or map reset retained an old refined path')
                else:
                    require(metadata['response_success'] and metadata['geometry']['path_accepted'] and
                            len(metadata['accepted_path']) >= 2 and len(snapshot.markers) == 2,
                            'Successful refinement is missing its complete adopted path')
                    line = snapshot.markers[1]
                    require(line.action == Marker.ADD and line.type == Marker.LINE_LIST and
                            line.ns == 'mod_path' and line.id == 0 and line.scale.x == .25,
                            'Marker differs from upstream mod_path style')
                    close([line.color.r, line.color.g, line.color.b, line.color.a],
                          [244/255, 217/255, 66/255, 1.], 'Upstream yellow', 1.e-7)
                    chain = metadata['accepted_path']
                    expected = [state[:3] for pair in zip(chain, chain[1:]) for state in pair]
                    close_points([xyz(point) for point in line.points], expected, 'Complete adopted segments')
                return snapshot_record(snapshot)

        probe = Probe()
        command = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
                   '-r', '__ns:='+probe.get_namespace()]
        for key, value in parameters.items():
            command += ['-p', key+':='+json.dumps(value)]
        summary['launch'] = command
        with log_path.open('x') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True, env=dict(os.environ))
        probe.until(lambda:probe.maps.get_subscription_count() > 0 and
                    probe.odometry.get_subscription_count() > 0, 'Map and odometry discovery')
        probe.maps.publish(make_map())
        probe.until(lambda:probe.status('map_status')['map_revision'] > 0, 'Map accepted')
        odometry = Odometry()
        odometry.header.frame_id, odometry.child_frame_id = 'map', 'base_link'
        odometry.header.stamp = probe.get_clock().now().to_msg()
        odometry.pose.pose.position.x, odometry.pose.pose.orientation.w = -2., 1.
        probe.odometry.publish(odometry)
        probe.until(lambda:math.dist(probe.status('sensor_gain').get('root', [1.e6]*4)[:3], [-2., 0., 0.]) < 1.e-8,
                    'Odometry accepted')

        since, uncaptured = probe.plan()
        require(uncaptured.status == PlannerSrv.Response.K_FORWARD and uncaptured.path, 'Unsubscribed local plan failed')
        metadata = probe.metadata(since, 'local')
        require(not metadata['geometry']['captured'] and not metadata['accepted_path'],
                'Diagnostic capture occurred without an enabled switch or subscriber')
        summary['phases']['no_subscriber'] = metadata

        subscription = probe.create_subscription(MarkerArray, 'vis/mod_path', probe.markers.append, durable)
        probe.until(lambda:probe.markers and probe.count_subscribers('refinement_geometry') == 0,
                    'Only mod_path subscription is active')
        since, captured = probe.plan()
        require(captured.status == PlannerSrv.Response.K_FORWARD and captured.path, 'Subscribed local plan failed')
        metadata = probe.metadata(since, 'local')
        record = {'metadata': metadata, 'markers': probe.snapshot(metadata)}
        close_points([xyz(pose.position) for pose in captured.path],
                     [xyz(pose.position) for pose in uncaptured.path], 'Subscription must not change path')
        returned = [xyz(pose.position) for pose in captured.path]
        chain = [state[:3] for state in metadata['accepted_path']]
        cursor = 0
        for knot in chain:
            while cursor < len(returned) and math.dist(returned[cursor], knot) > 1.e-8:
                cursor += 1
            require(cursor < len(returned), 'Original refined knot absent from interpolated response')
            cursor += 1
        require(len(returned) > len(chain), 'Fixture did not distinguish pre/post interpolation')
        record.update({'refined_points': len(chain), 'response_points': len(returned),
                       'response_xyz': returned, 'subscription_does_not_change_path': True})
        summary['phases']['mod_path_only'] = record

        since, failed = probe.plan('wrong_frame')
        require(not failed.path, 'Wrong-frame local plan unexpectedly succeeded')
        metadata = probe.metadata(since, 'local')
        summary['phases']['failed_response'] = {'metadata': metadata, 'markers': probe.snapshot(metadata, True)}
        # A late subscriber must also receive the durable clear, not the old line.
        retained = []
        late = probe.create_subscription(MarkerArray, 'vis/mod_path', retained.append, durable)
        probe.until(lambda:retained, 'Late subscriber received clear snapshot')
        require(snapshot_record(retained[-1]) == summary['phases']['failed_response']['markers'],
                'Late subscriber received stale successful geometry')
        probe.destroy_subscription(late)

        since, recovered = probe.plan()
        require(recovered.status == PlannerSrv.Response.K_FORWARD and recovered.path, 'Successful path did not recover after failed request')
        metadata = probe.metadata(since, 'local')
        probe.snapshot(metadata)
        previous = probe.status('map_status')['map_revision']
        since = probe.get_clock().now().nanoseconds
        empty = make_map()
        data = bytearray(empty.data)
        for offset in range(12, len(data), empty.point_step):
            data[offset] = 255  # int8 -1: valid dense snapshot, all cells Unknown.
        empty.data = bytes(data)
        probe.maps.publish(empty)
        probe.until(lambda:probe.status('map_status')['map_revision'] == previous+1, 'Empty map accepted')
        metadata = probe.metadata(since, 'clear')
        summary['phases']['empty_map'] = {'metadata': metadata, 'markers': probe.snapshot(metadata, True)}
        probe.destroy_subscription(subscription)
        summary['checks'] = {'subscriber_only_capture': True, 'exact_adopted_chain': True,
            'before_output_interpolation': True, 'path_unchanged': True,
            'failed_request_clears': True, 'late_subscriber_clear': True, 'empty_map_clears': True}
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'], summary['traceback'] = str(error), traceback.format_exc()
    finally:
        if process is not None:
            summary['cleanup']['forced'] = False
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    summary['cleanup']['forced'] = True
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3.)
            summary['cleanup']['exit_code'] = process.returncode
            if process.returncode != 0 or summary['cleanup']['forced']:
                summary['status'] = 'failed'
        if probe is not None:
            probe.destroy_node()
        if initialized:
            rclpy.shutdown()
        summary['elapsed_seconds'] = time.monotonic()-started
        output.write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps({'status': summary['status'], 'output': str(output), 'error': summary.get('error')}))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

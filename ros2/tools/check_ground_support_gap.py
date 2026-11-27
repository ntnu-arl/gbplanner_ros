#!/usr/bin/env python3
"""Exercise supported-bank rejection and genuine hanging lookahead over ROS 2.

A synthetic voxel snapshot has free body space over two supported banks and
an unknown support strip. No trajectory executor or physical robot is used.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time
import traceback

sys.dont_write_bytecode = True
from check_planning_visualization import require, stamp_ns, xyz


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    require(not output.exists(), 'Result already exists')
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    report = {'status': 'failed', 'scope': __doc__, 'phases': {}}
    start, node, process, initialized = time.monotonic(), None, None, False
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from planner_msgs.msg import Graph
        from planner_msgs.srv import PlanAhead, PlannerSrv
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        rclpy.init(args=[])
        initialized = True
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        class Probe(Node):
            def __init__(self):
                super().__init__('probe', namespace=f'/ground_support_gap_{os.getpid()}')
                self.rpc, self.graphs, self.metrics = {}, [], {'local': [], 'ahead': []}
                self.maps = self.create_publisher(PointCloud2, 'voxel_map', durable)
                self.odom = self.create_publisher(Odometry, 'odometry', 10)
                self.create_subscription(Graph, 'local_graph', self.graphs.append, durable)
                for label, topic in (('local', 'planning_metrics'), ('ahead', 'plan_ahead_metrics')):
                    self.create_subscription(String, topic,
                        lambda message, values=self.metrics[label]: values.append(json.loads(message.data)), durable)

            def until(self, predicate, label):
                end = min(start+100., time.monotonic()+20.)
                while not predicate():
                    require(process.poll() is None, 'Planner process exited')
                    if time.monotonic() >= end:
                        raise TimeoutError(label)
                    rclpy.spin_once(self, timeout_sec=.02)

            def call(self, name, request, kind=Trigger):
                client = self.rpc.setdefault(name, None)
                if client is None:
                    client = self.rpc[name] = self.create_client(kind, name)
                self.until(client.service_is_ready, name+' discovery')
                future = client.call_async(request)
                self.until(future.done, name+' response')
                return future.result()

            def status(self):
                return json.loads(self.call('map_status', Trigger.Request()).message)

        node = Probe()
        parameters = {
            'robot.type': 'kGround', 'robot.size': [.2, .2, .2],
            'ground.max_ground_height': .8, 'ground.robot_height': .6,
            'ground.max_inclination': .9, 'ground.max_negative_inclination': .9,
            'map.resolution': .1, 'map.origin': [-4., -2., -1.], 'map.dimensions': [80, 40, 30],
            'global.enable': False, 'global.background_enable': False, 'mission.enabled': False,
            'planning.auto_global_planner_enable': False, 'planning.path_safety_enhance_enable': False,
            'planning.adaptive_exploration': False, 'planning.graph_mode': False,
            'planning.edge_min': .05, 'planning.edge_max': 4., 'planning.overshoot': 0.,
            'planning.max_vertices': 2, 'planning.max_attempts': 3,
            'planning.path_length_penalty': 0., 'planning.path_direction_penalty': 0.,
            'planning.traverse_length_max': 10., 'planning.traverse_time_max': 100.,
            'sampling.seed': 42, 'sampling.seed_mode': 'fixed',
            'sampling.min': [-.1, -.5, -.1], 'sampling.max': [3.5, .5, .1],
            'sensor.range': 2., 'sensor.horizontal_resolution': .5, 'sensor.vertical_resolution': .5,
            'gain.unknown': 1., 'gain.free': .1, 'gain.occupied': 0.,
        }
        for axis in ('x', 'y', 'z', 'heading'):
            parameters['sampling.'+axis+'.distribution'] = 'constant'
            parameters['sampling.'+axis+'.constant'] = 3. if axis == 'x' else 0.
        command = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
                   '-r', '__ns:='+node.get_namespace()]
        for key, value in parameters.items():
            command += ['-p', key+':='+json.dumps(value)]
        report['parameters'], report['command'] = parameters, command
        with output.with_suffix('.planner.log').open('x') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        node.until(lambda:node.maps.get_subscription_count() and node.odom.get_subscription_count(), 'Inputs ready')

        def publish_map(gap):
            cloud = PointCloud2()
            cloud.header.frame_id, cloud.header.stamp = 'map', node.get_clock().now().to_msg()
            cloud.height, cloud.width, cloud.point_step = 1, 80*40*30, 16
            cloud.row_step, cloud.is_dense = cloud.width*16, True
            cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                            for i, name in enumerate(('x', 'y', 'z'))]
            cloud.fields.append(PointField(name='occupancy', offset=12, datatype=PointField.INT8, count=1))
            data = bytearray(cloud.row_step)
            offset = 0
            for iz in range(30):
                for iy in range(40):
                    for ix in range(80):
                        x, y, z = -4.+(ix+.5)*.1, -2.+(iy+.5)*.1, -1.+(iz+.5)*.1
                        status = 0 if iz >= 10 else (-1 if gap and abs(x) < .75 else 100)
                        struct.pack_into('<fffbxxx', data, offset, x, y, z, status)
                        offset += 16
            cloud.data = bytes(data)
            previous = node.status()['map_revision']
            node.maps.publish(cloud)
            node.until(lambda:node.status()['map_revision'] == previous+1, 'Map accepted')

        def call_plan(label, root, kind):
            service = 'planner' if kind == PlannerSrv else 'planner_ahead'
            key = 'local' if kind == PlannerSrv else 'ahead'
            request = kind.Request()
            request.header.frame_id, request.bound_mode = 'map', 3
            request.root_pose.position.x, request.root_pose.position.z = root, .6
            request.root_pose.orientation.w = 1.
            if key == 'ahead':
                request.arrival_time = 1.
            before, since = len(node.metrics[key]), node.get_clock().now().nanoseconds
            response = node.call(service, request, kind)
            node.until(lambda:len(node.metrics[key]) > before, label+' metrics')
            metrics = node.metrics[key][-1]
            phase = metrics if key == 'local' else metrics['plan']
            require(phase is not None, label+' did not reach planning')
            record = {'metrics': metrics, 'path': [xyz(p.position) for p in response.path]}
            if key == 'local':
                node.until(lambda:node.graphs and stamp_ns(node.graphs[-1].header) >= since, label+' graph')
                record['graph'] = {'vertices': [[v.id, *xyz(v.pose.position)] for v in node.graphs[-1].vertices],
                                   'edges': [[e.source_id, e.target_id] for e in node.graphs[-1].edges]}
            report['phases'][label] = record
            return phase, response

        publish_map(True)
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id = 'map', 'base_link'
        odom.header.stamp = node.get_clock().now().to_msg()
        odom.pose.pose.position.x, odom.pose.pose.position.z, odom.pose.pose.orientation.w = -1.5, .6, 1.
        node.odom.publish(odom)
        node.until(lambda:json.loads(node.call('sensor_gain', Trigger.Request()).message).get('root', [0.])[0] == -1.5,
                   'Odometry accepted')
        for kind in (PlannerSrv, PlanAhead):
            label = 'gap_'+('local' if kind == PlannerSrv else 'ahead')
            phase, response = call_plan(label, -1.5, kind)
            require(phase['vertices'] == 1 and phase['edges'] == 0 and not phase['success'] and
                    phase['collision_rejections'] == 3 and not response.path,
                    label+': supported banks were connected across unknown support')
        phase, response = call_plan('hanging_endpoint', -3., PlannerSrv)
        require(phase['vertices'] == 2 and phase['hanging_vertices'] == 1 and not response.path,
                'Genuine hanging lookahead was removed or became executable')
        publish_map(False)
        for kind in (PlannerSrv, PlanAhead):
            label = 'supported_'+('local' if kind == PlannerSrv else 'ahead')
            phase, response = call_plan(label, -1.5, kind)
            require(phase['vertices'] == 2 and phase['success'] and len(response.path) >= 2 and
                    abs(response.path[-1].position.x-1.5) < 1.e-8,
                    label+': observed support did not restore an executable path')
        report['status'] = 'passed'
    except Exception as error:
        report['error'], report['traceback'] = str(error), traceback.format_exc()
    finally:
        if process is not None:
            forced = False
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    forced = True
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3.)
            report['cleanup'] = {'exit_code': process.returncode, 'forced': forced}
            if forced or process.returncode != 0:
                report['status'] = 'failed'
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()
        report['wall_time_s'] = time.monotonic()-start
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        print(json.dumps({'status': report['status'], 'output': str(output), 'error': report.get('error')}))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

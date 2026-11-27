#!/usr/bin/env python3
"""Check complete local routes with native PCI and zero PlanAhead lead time.

The aerial case executes a real planner response through the native PathFollower
and a test-only timed-trajectory consumer. A separate ground planner previews
its dense terrain polyline without executing motion. Synthetic occupancy and
odometry are inputs; no planner response, graph, or action result is fabricated.
This is not a dynamics or hardware experiment.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time
import traceback


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def pose_values(pose):
    p, q = pose.position, pose.orientation
    return [p.x, p.y, p.z, q.x, q.y, q.z, q.w]


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=workspace/'results/local_full_execution/summary.json')
    parser.add_argument('--timeout', type=float, default=60.)
    args = parser.parse_args()
    require(math.isfinite(args.timeout) and args.timeout>0., 'Timeout must be positive')
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    started, initialized, probes = time.monotonic(), False, []
    deadline = started+args.timeout
    summary = {'status': 'failed', 'scope': __doc__, 'launches': [], 'cases': {}, 'checks': {}}
    try:
        import rclpy
        from ament_index_python.packages import get_package_prefix
        from nav_msgs.msg import Odometry, Path as RosPath
        from planner_msgs.srv import PciTrigger, ValidatePath
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from trajectory_msgs.msg import MultiDOFJointTrajectory
        sys.dont_write_bytecode = True
        sys.path.insert(0, str(Path(get_package_prefix('gbplanner_execution'))/'lib/gbplanner_execution'))
        from execution_demo import decode, sample
        sys.path.insert(0, str(workspace/'src/gbplanner_local/scripts'))
        from local_demo import DIM, ORIGIN, RES, make_map

        rclpy.init(args=[])
        initialized = True
        durable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        common = {
            'global.enable': True, 'global.auto_home': True, 'global.background_enable': False,
            'planning.auto_global_planner_enable': False,
            'planning.path_safety_enhance_enable': False, 'planning.path_interpolation_distance': .25,
            'sampling.min': [-.5, -.5, -.1], 'sampling.max': [4.5, .5, .1],
            'planning.max_vertices': 3, 'planning.max_attempts': 10, 'planning.graph_mode': False,
            'planning.edge_min': .05, 'planning.edge_max': 1.2, 'planning.overshoot': 0.,
            'planning.v_max': .3, 'planning.traverse_length_max': 2., 'planning.traverse_time_max': 100.,
            'planning.path_length_penalty': 0., 'planning.path_direction_penalty': 0.,
            'planning.cluster_vertices_for_gain': False, 'robot.size': [.1, .1, .1],
            'gain.free': 1., 'gain.unknown': 1., 'gain.frontier_threshold': .05,
            'sensor.range': .75, 'sensor.horizontal_fov': 1., 'sensor.horizontal_resolution': 1.,
            'sensor.vertical_fov': .2, 'sensor.vertical_resolution': .2,
        }
        for axis in ('x', 'y', 'z', 'heading'):
            common[f'sampling.{axis}.distribution'] = 'constant'
            common[f'sampling.{axis}.constant'] = 4. if axis=='x' else 0.

        def ground_map():
            origin, dimensions, resolution = (-3., -1., -.4), (60, 20, 20), .1
            cloud = PointCloud2()
            cloud.header.frame_id, cloud.height, cloud.width = 'map', 1, math.prod(dimensions)
            cloud.point_step, cloud.row_step, cloud.is_dense = 16, cloud.width*16, True
            cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                            for i, name in enumerate(('x', 'y', 'z'))]
            cloud.fields.append(PointField(name='occupancy', offset=12, datatype=PointField.INT8, count=1))
            data, offset = bytearray(cloud.row_step), 0
            for z in range(dimensions[2]):
                for y in range(dimensions[1]):
                    for x in range(dimensions[0]):
                        point = [origin[i]+(value+.5)*resolution for i, value in enumerate((x, y, z))]
                        struct.pack_into('<fffbxxx', data, offset, *point, 100 if point[2]<0. else 0)
                        offset += 16
            cloud.data = bytes(data)
            return cloud, {'map.origin': list(origin), 'map.dimensions': list(dimensions),
                           'map.resolution': resolution, 'robot.type': 'kGround',
                           'ground.robot_height': .4, 'ground.max_ground_height': .8,
                           'ground.interpolate_projection_distance': False}

        class Probe(Node):
            def __init__(self, label, ground=False):
                self.fixture_namespace = f'/local_full_execution_{os.getpid()}_{label}'
                super().__init__('probe', namespace=self.fixture_namespace)
                self.label, self.ground, self.closed = label, ground, False
                self.position = (-2., 0., .4 if ground else 0., 0.)
                self.initial_position = self.position
                self.processes, self.service_clients, self.command = [], {}, None
                self.native_paths, self.command_paths, self.metrics, self.global_metrics = [], [], [], []
                self.control_events, self.execution_events, self.trajectories = [], [], []
                self.latest_command = None
                self.record = {'scope': 'native terrain preview' if ground else 'native action with kinematic consumer'}
                summary['cases'][label] = self.record
                probes.append(self)
                self.maps = self.create_publisher(PointCloud2, 'voxel_map', durable)
                self.odometry = self.create_publisher(Odometry, 'odometry', 10)
                self.create_subscription(RosPath, 'local_path',
                    lambda m:self.native_paths.append([pose_values(p.pose) for p in m.poses]), durable)
                self.create_subscription(RosPath, 'command_path', self.command_path, durable)
                for topic, rows in (('planning_metrics', self.metrics), ('global_metrics', self.global_metrics),
                                    ('control_status', self.control_events), ('execution_status', self.execution_events)):
                    self.create_subscription(String, topic, lambda m, r=rows:r.append(json.loads(m.data)), durable)
                if not ground:
                    self.create_subscription(MultiDOFJointTrajectory, 'command/trajectory', self.consume, 10)
                self.timer = self.create_timer(.02, self.tick)
                if ground:
                    self.cloud, overrides = ground_map()
                else:
                    self.cloud = make_map(known=True, wall=False)
                    overrides = {'map.origin': list(ORIGIN), 'map.dimensions': list(DIM), 'map.resolution': RES}
                settings = dict(common, **overrides)
                self.record['planner_parameters'] = settings
                self.launch('gbplanner_local', 'local_planner_node', settings)
                self.launch('gbplanner_local', 'control_node.py',
                            {'planner_trigger_lead_time': 0., 'robot_type': 'kGround' if ground else 'kAerial'})
                if not ground:
                    self.launch('gbplanner_execution', 'path_follower_node.py', {'speed': .8, 'yaw_rate': 1.})

            def launch(self, package, executable, parameters):
                command = ['ros2', 'run', package, executable, '--ros-args', '-r', '__ns:='+self.fixture_namespace]
                for key, value in parameters.items():
                    command += ['-p', key+':='+json.dumps(value)]
                log_path = output.parent/f'{self.label}_{executable}.log'
                with log_path.open('w') as stream:
                    process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                               start_new_session=True, env=dict(os.environ))
                self.processes.append((process, log_path))
                summary['launches'].append(command)

            def until(self, predicate, label, seconds=15.):
                limit = min(deadline, time.monotonic()+seconds)
                while not predicate():
                    for process, log in self.processes:
                        if process.poll() is not None:
                            raise RuntimeError(f'{self.label} node exited {process.returncode}; see {log}')
                    if time.monotonic()>=limit:
                        raise TimeoutError(self.label+': '+label)
                    rclpy.spin_once(self, timeout_sec=.02)

            def call(self, name, request, kind=Trigger):
                if name not in self.service_clients:
                    self.service_clients[name] = self.create_client(kind, name)
                client = self.service_clients[name]
                self.until(client.service_is_ready, name+' discovery')
                future = client.call_async(request)
                self.until(future.done, name+' response')
                return future.result()

            def status(self, name='control_state'):
                return json.loads(self.call(name, Trigger.Request()).message)

            def command_path(self, message):
                self.latest_command = message
                self.command_paths.append([pose_values(p.pose) for p in message.poses])

            def consume(self, message):
                points, times = decode(message)
                self.trajectories.append({'points': points, 'times': times})
                self.command = (points, times, time.monotonic(), self.position)

            def tick(self):
                if self.command:
                    points, times, begin, initial = self.command
                    self.position = sample(points, times, time.monotonic()-begin, initial)
                message = Odometry()
                message.header.frame_id, message.child_frame_id = 'map', 'base_link'
                message.header.stamp = self.get_clock().now().to_msg()
                message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = self.position[:3]
                message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(self.position[3]/2), math.cos(self.position[3]/2)
                self.odometry.publish(message)

            def initialize(self):
                self.until(lambda:self.maps.get_subscription_count()>0 and
                           self.odometry.get_subscription_count()>=(2 if self.ground else 3), 'native subscriptions')
                self.maps.publish(self.cloud)
                self.until(lambda:self.status()['known_cells']>0 and self.status('global_status')['vertices']>0,
                           'accepted map and home')
                if not self.ground:
                    self.until(lambda:self.status('execution_state')['command_subscribers']>0, 'native follower ready')

            def start(self):
                request = PciTrigger.Request()
                request.bound_mode, request.not_exe_path, request.set_auto = 3, self.ground, False
                self.until(lambda:self.call('pci_trigger', request, PciTrigger).success, 'PCI trigger accepted')
                self.until(lambda:self.metrics and self.native_paths and self.command_paths and bool(self.command_paths[-1]),
                           'native local and PCI command paths')
                native, commanded = self.native_paths[-1], self.command_paths[-1]
                require(self.metrics[-1]['success'] and self.metrics[-1]['gain']>0. and len(native)>3,
                        'Fixture did not generate a multi-point local route')
                # Execution previews its measured connector before handing the
                # original route to the follower; ground preview adds none.
                require((native==commanded if self.ground else
                         len(commanded)==len(native)+1 and commanded[1:]==native),
                        'PCI changed or truncated the complete native local path')
                lengths = [math.dist(a[:3], b[:3]) for a, b in zip(native, native[1:])]
                require(sum(lengths)>1. and lengths[0]<=.25+1.e-8,
                        'Fixture did not separate first output sample from the full endpoint')
                self.record.update(native_route=native, commanded_route=commanded, local_metrics=self.metrics[-1],
                                   first_segment_m=lengths[0], total_length_m=sum(lengths))
                return native

            def close(self):
                if self.closed:
                    return
                self.closed = True
                self.timer.cancel()
                self.record.update(control_events=self.control_events, execution_events=self.execution_events,
                                   trajectories=self.trajectories, final_position=list(self.position))
                exits = []
                for process, log in reversed(self.processes):
                    if process.poll() is None:
                        try:
                            os.killpg(process.pid, signal.SIGINT)
                        except ProcessLookupError:
                            pass
                        try:
                            process.wait(timeout=3.)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait(timeout=1.)
                    exits.append({'log': str(log), 'exit': process.returncode})
                self.record['process_exits'] = exits
                self.destroy_node()

        aerial = Probe('aerial')
        aerial.initialize()
        native = aerial.start()
        # Cross the first output sample while remaining far from the real
        # endpoint. The native action must still be active, not finish there.
        aerial.until(lambda:math.dist(aerial.position[:3], native[0][:3])>.4, 'travel beyond first output sample')
        halfway, follower = aerial.status(), aerial.status('execution_state')
        require(halfway['state']=='EXECUTING' and halfway['completed_actions']==0 and
                follower['active'] and follower['succeeded']==0 and math.dist(aerial.position[:3], native[-1][:3])>.5,
                'Local action ended at the first sample instead of continuing to its actual target')
        aerial.record['past_first_sample'] = {'control': halfway, 'execution': follower,
                                             'measured_position': list(aerial.position)}
        aerial.until(lambda:aerial.status()['state'] in {'DONE', 'ERROR', 'STOPPED'}, 'full local execution')
        final, execution = aerial.status(), aerial.status('execution_state')
        require(final['state']=='DONE' and final['reason']=='manual_path_complete' and final['mode']=='local'
                and final['completed_actions']==1 and math.dist(aerial.position[:3], native[-1][:3])<.05,
                'PCI did not complete the full local route')
        require(execution['accepted']==1 and execution['succeeded']==1 and execution['cancelled']==0
                and execution['aborted']==0 and execution['continuations']==0 and len(aerial.trajectories)==1,
                'Complete local route did not use one successful unchanged native action')
        require(not aerial.global_metrics and not any(e['reason']=='plan_ahead_requested' for e in final['events']),
                'Zero lead time did not keep speculative/global requests disabled for this local task')
        aerial.record.update(final_control=final, final_execution=execution)
        aerial.close()

        ground = Probe('ground', ground=True)
        ground.initialize()
        native = ground.start()
        ground.until(lambda:ground.status()['state'] in {'PREVIEW', 'ERROR', 'STOPPED'}, 'terrain preview')
        state = ground.status()
        require(state['state']=='PREVIEW' and state['mode']=='local' and state['completed_actions']==0,
                'Ground fixture did not produce a native local preview')
        require(ground.record['first_segment_m']<.25 and len(native)>5,
                'Ground fixture did not preserve terrain knots denser than output interpolation')
        validation = ValidatePath.Request()
        validation.header.frame_id, validation.bound_mode = 'map', 3
        validation.path = [entry.pose for entry in ground.latest_command.poses]
        valid = ground.call('validate_path', validation, ValidatePath)
        require(valid.valid, 'Complete ground preview failed native validation: '+valid.reason)
        require(all(abs(point[2]-.4)<1.e-8 for point in native), 'Terrain preview changed base-link height')
        ground.record.update(final_control=state,
                             native_validation={'valid': valid.valid, 'reason': valid.reason, 'map_revision': valid.map_revision})
        ground.close()
        summary['checks'] = dict.fromkeys(['zero_lead_preserves_full_local_route',
            'native_action_continues_past_first_output_sample', 'native_follower_reaches_exact_local_target',
            'single_action_no_ahead_or_global', 'ground_preview_preserves_all_terrain_knots',
            'ground_full_preview_passes_native_validation'], True)
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'], summary['traceback'] = str(error), traceback.format_exc()
    finally:
        for probe in reversed(probes):
            try:
                probe.close()
                if any(row['exit']!=0 for row in probe.record.get('process_exits', [])):
                    summary['status'], summary['cleanup_error'] = 'failed', 'Native node exited nonzero'
            except Exception as error:
                summary['status'], summary['cleanup_error'] = 'failed', str(error)
        if initialized and rclpy.ok():
            rclpy.shutdown()
        summary['elapsed_s'] = time.monotonic()-started
        output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'status': summary['status'], 'output': str(output),
                      'elapsed_s': summary['elapsed_s'], 'error': summary.get('error')}))
    return 0 if summary['status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())

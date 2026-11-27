#!/usr/bin/env python3
"""Check fully explored homing through native global services and the PCI.

Runs real planner, controller and follower processes with synthetic occupancy,
odometry and a trajectory consumer. This does not simulate robot dynamics.
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


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def xyz(poses):
    return [[p.position.x, p.position.y, p.position.z] for p in poses]


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=workspace/'results/global_homing/summary.json')
    parser.add_argument('--timeout', type=float, default=120.)
    args = parser.parse_args()
    require(math.isfinite(args.timeout) and args.timeout > 0., 'Timeout must be positive')
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    started = time.monotonic()
    deadline = started+args.timeout
    fixtures = []
    initialized = False
    summary = {'status': 'failed', 'scope': __doc__, 'launches': [], 'checks': {}}
    try:
        import rclpy
        from ament_index_python.packages import get_package_prefix
        from nav_msgs.msg import Odometry, Path as RosPath
        from planner_msgs.msg import Graph
        from planner_msgs.srv import PciGlobal, PciTrigger, PlannerGlobal
        from planner_semantic_msgs.msg import SemanticClass, SemanticPoint
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from trajectory_msgs.msg import MultiDOFJointTrajectory
        # Reuse only the established test consumer's trajectory decoding and
        # interpolation; no planner service response is replaced or fabricated.
        sys.path.insert(0, str(Path(get_package_prefix('gbplanner_execution'))/'lib/gbplanner_execution'))
        from execution_demo import decode, sample

        rclpy.init(args=[])
        initialized = True
        durable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        origin, dimensions, resolution = (-4., -4., -2.), (32, 16, 8), .5
        common = {
            'map.resolution': resolution, 'map.origin': list(origin), 'map.dimensions': list(dimensions),
            'global.enable': True, 'global.auto_home': True, 'global.background_enable': False,
            'planning.go_home_if_fully_explored': True,
            'sampling.min': [-1., -1., -.2], 'sampling.max': [1., 1., .2], 'sampling.seed': 42,
            'planning.max_vertices': 20, 'planning.max_attempts': 400,
            'planning.path_safety_enhance_enable': False,
            'sensor.range': 2., 'sensor.horizontal_fov': 2.*math.pi, 'sensor.vertical_fov': .3,
            'sensor.horizontal_resolution': .3, 'sensor.vertical_resolution': .15,
            'gain.free': 0., 'gain.unknown': 1., 'gain.frontier_threshold': .05,
        }

        def make_map(remote=False, wall=False):
            cloud = PointCloud2()
            cloud.header.frame_id = 'map'
            cloud.height, cloud.width, cloud.point_step = 1, math.prod(dimensions), 16
            cloud.row_step, cloud.is_dense = cloud.width*16, True
            cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                            for i, name in enumerate(('x', 'y', 'z'))]
            cloud.fields.append(PointField(name='occupancy', offset=12, datatype=PointField.INT8, count=1))
            data = bytearray(cloud.row_step)
            offset = 0
            for z in range(dimensions[2]):
                for y in range(dimensions[1]):
                    for x in range(dimensions[0]):
                        point = [origin[i]+(value+.5)*resolution for i, value in enumerate((x, y, z))]
                        occupancy = -1 if remote and point[0] >= 9. else 0
                        if wall and 4. <= point[0] < 4.5:
                            occupancy = 100
                        struct.pack_into('<fffbxxx', data, offset, *point, occupancy)
                        offset += 16
            cloud.data = bytes(data)
            return cloud

        class Probe(Node):
            def __init__(self, label, overrides=None, controller=False):
                namespace = f'/global_home_{os.getpid()}_{label}'
                super().__init__('probe', namespace=namespace)
                self.fixture_namespace = namespace
                self.label, self.processes, self.service_clients = label, [], {}
                self.position, self.active_command = (-2., 0., 0., 0.), None
                self.graph, self.metrics, self.commands, self.trajectories = None, [], [], []
                self.maps = self.create_publisher(PointCloud2, 'voxel_map', durable)
                self.odometry = self.create_publisher(Odometry, 'odometry', 10)
                self.semantic = self.create_publisher(SemanticPoint, 'semantic_location', 10)
                self.create_subscription(Graph, 'global_graph', lambda m:setattr(self, 'graph', m), durable)
                self.create_subscription(String, 'global_metrics', lambda m:self.metrics.append(json.loads(m.data)), durable)
                self.create_subscription(RosPath, 'command_path', self.commands.append, durable)
                self.create_subscription(MultiDOFJointTrajectory, 'command/trajectory', self.consume, 10)
                self.create_timer(.02, self.tick)
                fixtures.append(self)
                parameters = dict(common)
                parameters.update(overrides or {})
                self.launch('gbplanner_local', 'local_planner_node', parameters)
                if controller:
                    self.launch('gbplanner_local', 'control_node.py', {'planner_trigger_lead_time': 0.})
                    self.launch('gbplanner_execution', 'path_follower_node.py', {'yaw_rate': 1.0})

            def launch(self, package, executable, parameters):
                command = ['ros2', 'run', package, executable, '--ros-args', '-r', '__ns:='+self.fixture_namespace]
                for key, value in parameters.items():
                    command += ['-p', key+':='+json.dumps(value)]
                log_path = output.parent/f'{self.label}_{executable}.log'
                with log_path.open('w') as log:
                    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True, env=dict(os.environ))
                self.processes.append((process, log_path))
                summary['launches'].append(command)

            def until(self, predicate, label, seconds=20.):
                limit = min(deadline, time.monotonic()+seconds)
                while not predicate():
                    for process, log in self.processes:
                        if process.poll() is not None:
                            raise RuntimeError(f'{self.label} exited {process.returncode}; see {log}')
                    if time.monotonic() >= limit:
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

            def status(self, name='global_status'):
                return json.loads(self.call(name, Trigger.Request()).message)

            def consume(self, message):
                points, times = decode(message)
                self.trajectories.append({'points': points, 'times': times})
                self.active_command = (points, times, time.monotonic(), self.position)

            def tick(self):
                if self.active_command:
                    points, times, start, initial = self.active_command
                    self.position = sample(points, times, time.monotonic()-start, initial)
                message = Odometry()
                message.header.frame_id, message.child_frame_id = 'map', 'base_link'
                message.header.stamp = self.get_clock().now().to_msg()
                message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = self.position[:3]
                message.pose.pose.orientation.z = math.sin(self.position[3]/2)
                message.pose.pose.orientation.w = math.cos(self.position[3]/2)
                self.odometry.publish(message)

            def publish_map(self, cloud):
                self.until(lambda:self.maps.get_subscription_count()>0 and self.odometry.get_subscription_count()>0,
                           'map/odometry discovery')
                previous = self.status('map_status')['map_revision']
                # A static fixture has no sensor observation clock. Re-publish
                # after completion to test that the PCI stays terminal.
                self.maps.publish(cloud)
                self.tick()
                self.until(lambda:self.status('map_status')['map_revision']==previous+1, 'map acceptance')
                self.until(lambda:self.graph is not None and len(self.graph.vertices)>0, 'home initialization')

            def seed(self):
                self.publish_map(make_map())
                self.home = xyz([next(v.pose for v in self.graph.vertices if v.id==0)])[0]
                require(math.dist(self.home, (-2., 0., 0.))<1e-9, 'Home was not initialized at the original pose')
                for i in range(1, 9):
                    x = -2.+.5*i
                    self.position = (x, 0., 0., 0.)
                    self.tick()
                    self.until(lambda:any(abs(v.pose.position.x-x)<1e-9 for v in self.graph.vertices), 'odometry graph')
                self.target = next(v.id for v in self.graph.vertices if abs(v.pose.position.x)<1e-9)
                require(self.status()['frontiers']==0, 'Seed graph unexpectedly has a frontier')

            def global_call(self, **overrides):
                before = len(self.metrics)
                request = PlannerGlobal.Request()
                request.bound_mode, request.ignore_time = 3, True
                for key, value in overrides.items():
                    setattr(request, key, value)
                response = self.call('planner_global', request, PlannerGlobal)
                self.until(lambda:len(self.metrics)>before, 'global metrics')
                return response, self.metrics[-1]

            def close(self):
                for process, _ in reversed(self.processes):
                    if process.poll() is not None:
                        continue
                    try:
                        os.killpg(process.pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
                    try:
                        process.wait(timeout=3.)
                    except subprocess.TimeoutExpired:
                        try:
                            os.killpg(process.pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                        process.wait(timeout=1.)
                self.destroy_node()

        def record(response, metric):
            return {'homing': response.homing, 'path': xyz(response.path), 'metrics': metric}

        def at_home(response, probe):
            require(response.homing and len(response.path)>1 and math.dist(xyz(response.path)[-1], probe.home)<1e-9,
                    'Global service did not return the original home route')

        enabled = Probe('enabled', controller=True)
        enabled.seed()
        response, metric = enabled.global_call()
        at_home(response, enabled)
        require(metric['mode']=='homing' and metric['reason']=='fully_explored_homing', 'Wrong homing diagnostics')
        summary['direct_service'] = record(response, metric)
        response, metric = enabled.global_call(id=enabled.target)
        at_home(response, enabled)
        summary['checked_explicit_target'] = record(response, metric)
        response, metric = enabled.global_call(id=enabled.target, not_check_frontier=True)
        require(not response.homing and response.path and math.dist(xyz(response.path)[-1], (0., 0., 0.))<1e-9,
                'Explicit frontier bypass was redirected home')
        summary['explicit_bypass'] = record(response, metric)
        for label, options, reason in [('invalid_target', {'id': 999999}, 'invalid_global_target'),
                                       ('budget_disabled', {'ignore_time': False}, 'time_budget_disabled')]:
            response, metric = enabled.global_call(**options)
            require(not response.path and not response.homing and metric['reason']==reason, label+' bypassed validation')
            summary[label] = record(response, metric)

        enabled.until(lambda:enabled.status('control_state')['known_cells']>0, 'PCI map')
        before = len(enabled.commands)
        request = PciGlobal.Request()
        request.bound_mode, request.ignore_time, request.not_exe_path = 3, True, True
        require(enabled.call('pci_global', request, PciGlobal).success, 'PCI global preview refused')
        enabled.until(lambda:enabled.status('control_state')['state'] in {'PREVIEW', 'ERROR', 'DONE'}, 'PCI preview')
        control = enabled.status('control_state')
        require(control['state']=='PREVIEW' and control['mode']=='homing' and not control['auto'],
                'PCI did not identify the global response as homing')
        enabled.until(lambda:len(enabled.commands)>before, 'PCI command preview')
        require(math.dist(xyz([p.pose for p in enabled.commands[-1].poses])[-1], enabled.home)<1e-9 and
                not enabled.trajectories and sum(e['reason']=='global_requested' for e in control['events'])==1,
                'PCI changed the home endpoint, dispatched motion, or repeated planning')
        summary['pci_preview'] = control

        # Retain an actual semantic frontier, then disconnect its graph branch.
        # Its current ray gain stays positive, so this is not full exploration.
        enabled.publish_map(make_map(remote=True))
        enabled.until(lambda:enabled.semantic.get_subscription_count()>0, 'semantic discovery')
        semantic = SemanticPoint()
        semantic.header.frame_id, semantic.type.value, semantic.point.x = 'map', SemanticClass.K_DOOR, 8.
        enabled.semantic.publish(semantic)
        enabled.until(lambda:enabled.status('semantic_state')['received']>0, 'semantic route creation')
        require(enabled.status('semantic_state')['success'], 'Native semantic search did not retain a frontier')
        response, metric = enabled.global_call()
        require(response.path and not response.homing and metric['frontiers']>0, 'Native frontier gain is missing')
        enabled.publish_map(make_map(remote=True, wall=True))
        response, metric = enabled.global_call()
        require(not response.path and not response.homing and metric['frontiers']>0,
                'An unreachable retained frontier incorrectly triggered fully explored homing')
        summary['unreachable_frontier'] = record(response, metric)

        # A real automatic PCI task goes through local retries -> global ->
        # home. Consume the native follower trajectory, then deliver a fresh
        # map to prove completion does not restart local exploration.
        enabled.publish_map(make_map())
        enabled.global_call()  # Recompute the former frontier's now-zero gain.
        enabled.until(lambda:enabled.status('execution_state')['command_subscribers']>0, 'native follower readiness')
        request = PciTrigger.Request()
        request.bound_mode, request.set_auto = 3, True
        require(enabled.call('pci_trigger', request, PciTrigger).success, 'Automatic PCI trigger refused')
        enabled.until(lambda:enabled.status('control_state')['state'] in {'EXECUTING', 'ERROR', 'DONE'}, 'automatic homing start')
        active = enabled.status('control_state')
        require(active['mode']=='homing' and not active['auto'] and active['state']=='EXECUTING',
                'Automatic global fallback failed to switch to terminal homing')
        enabled.until(lambda:enabled.status('control_state')['state'] in {'DONE', 'ERROR'}, 'automatic return completion', seconds=35.)
        completed = enabled.status('control_state')
        require(completed['state']=='DONE' and completed['reason']=='returned_home' and
                completed['completed_actions']==1 and math.dist(enabled.position[:3], enabled.home)<.05,
                'Native automatic home route did not complete')
        requests = [e for e in completed['events'] if e['state']=='PLANNING']
        require(sum(e['reason']=='global_requested' for e in requests)==1 and
                not any(e['reason']=='homing_requested' for e in requests), 'PCI repeated global/home planning')
        enabled.publish_map(make_map())
        final = enabled.status('control_state')
        require(final['state']=='DONE' and final['mode']=='homing' and not final['auto'] and final['events']==completed['events'],
                'Fresh observation restarted local exploration after returning home')
        summary['automatic_homing'] = {'active': active, 'completed': final, 'trajectories': enabled.trajectories}
        enabled.close()
        fixtures.remove(enabled)

        for label, overrides, options, reason in [
                ('disabled', {'planning.go_home_if_fully_explored': False}, {}, 'no_global_frontier'),
                ('landing', {'mission.auto_landing_enable': True, 'mission.time_budget_before_landing': 3600.}, {}, 'no_global_frontier'),
                ('expired', {'mission.enabled': True, 'mission.time_limit': .001}, {'ignore_time': False}, 'mission_budget_exhausted')]:
            probe = Probe(label, overrides)
            probe.seed()
            response, metric = probe.global_call(**options)
            require(not response.path and not response.homing and metric['reason']==reason, label+' policy was bypassed')
            summary[label] = record(response, metric)
            probe.close()
            fixtures.remove(probe)
        summary['checks'] = dict.fromkeys([
            'direct_service_returns_original_home', 'pci_preview_identifies_homing',
            'automatic_fallback_stops_after_home', 'explicit_frontier_bypass_preserved',
            'invalid_target_rejected_before_homing', 'unreachable_frontier_is_not_fully_explored',
            'disabled_policy_preserved', 'landing_takes_precedence', 'mission_budget_preserved'], True)
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
        summary['last_metrics'] = {fixture.label: fixture.metrics[-6:] for fixture in fixtures}
    finally:
        stopped = True
        for fixture in reversed(fixtures):
            try:
                fixture.close()
            except Exception as error:
                summary.setdefault('cleanup_errors', []).append(str(error))
                stopped = False
        if initialized and rclpy.ok():
            rclpy.shutdown()
        summary['processes_stopped'] = stopped
        summary['elapsed_s'] = round(time.monotonic()-started, 3)
        if not stopped:
            summary['status'] = 'failed'
        output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(summary, ensure_ascii=False))
    return 0 if summary['status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())

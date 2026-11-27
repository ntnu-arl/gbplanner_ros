#!/usr/bin/env python3
"""Native ROS check of persistent frontier switching and full PCI preview paths.

Uses synthetic occupancy and odometry with real planner/control processes. The
remote frontier graph is generated and saved by another native local planner;
no planner service responses or serialized graph contents are manufactured.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def coordinates(poses):
    return [[p.position.x, p.position.y, p.position.z,
             p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
            for p in poses]


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=workspace/'results/frontier_switch/summary.json')
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
    summary = {'status': 'failed', 'scope': 'native services and PCI preview; synthetic map, no robot dynamics',
               'domain': os.environ['ROS_DOMAIN_ID'], 'launches': [], 'checks': {}}
    try:
        import rclpy
        from geometry_msgs.msg import Pose
        from nav_msgs.msg import Odometry, Path as RosPath
        from planner_msgs.srv import PciTrigger, PlannerSrv, PlannerStringTrigger
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        rclpy.init(args=[])
        initialized = True
        durable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        origin, dimensions, resolution = (-4., -5., -3.), (36, 20, 12), .5
        common = {
            'map.resolution': resolution, 'map.origin': list(origin), 'map.dimensions': list(dimensions),
            'global.enable': True, 'global.auto_home': True, 'global.background_enable': False,
            'sampling.min': [-1., -1., -.2], 'sampling.max': [1., 1., .2], 'sampling.seed': 42,
            'planning.max_vertices': 32, 'planning.max_attempts': 1200,
            # Keep this fixture's selected paths above the original 0.3 m execution gate.
            'planning.edge_min': .35, 'planning.edge_max': 1., 'planning.neighbor_max': 1.5,
            'planning.path_safety_enhance_enable': False,
            'sensor.range': 2., 'sensor.horizontal_fov': 2.*math.pi, 'sensor.vertical_fov': .3,
            'sensor.horizontal_resolution': .3, 'sensor.vertical_resolution': .15,
            'gain.free': .05, 'gain.unknown': 1., 'gain.frontier_threshold': .05,
        }
        # Omit auto_global_planner_enable and go_home_if_fully_explored so the
        # actual native defaults (true / false) are exercised in both cases.

        def make_map(remote_frontier=False):
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
                        occupancy = -1 if remote_frontier and point[0] >= 6. else 0
                        struct.pack_into('<fffbxxx', data, offset, *point, occupancy)
                        offset += 16
            cloud.data = bytes(data)
            return cloud

        class Probe(Node):
            def __init__(self, label, parameters, controller=False):
                namespace = f'/frontier_switch_{os.getpid()}_{label}'
                super().__init__('probe', namespace=namespace)
                self.label, self.processes, self.service_clients = label, [], {}
                self.metrics, self.global_metrics, self.local_paths, self.commands = [], [], [], []
                self.maps = self.create_publisher(PointCloud2, 'voxel_map', durable)
                self.odometry = self.create_publisher(Odometry, 'odometry', 10)
                self.create_subscription(String, 'planning_metrics', lambda m:self.metrics.append(json.loads(m.data)), durable)
                self.create_subscription(String, 'global_metrics', lambda m:self.global_metrics.append(json.loads(m.data)), durable)
                self.create_subscription(RosPath, 'local_path', self.local_paths.append, durable)
                self.create_subscription(RosPath, 'command_path', self.commands.append, durable)
                self.create_timer(.05, self.publish_odometry)
                fixtures.append(self)
                self.launch('local_planner_node', namespace, parameters)
                if controller:
                    self.launch('control_node.py', namespace, {'planner_trigger_lead_time': 0.})

            def launch(self, executable, namespace, parameters):
                command = ['ros2', 'run', 'gbplanner_local', executable,
                           '--ros-args', '-r', '__ns:='+namespace]
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
                    rclpy.spin_once(self, timeout_sec=.03)

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

            def publish_odometry(self):
                message = Odometry()
                message.header.frame_id, message.child_frame_id = 'map', 'base_link'
                message.header.stamp = self.get_clock().now().to_msg()
                message.pose.pose.orientation.w = 1.
                self.odometry.publish(message)

            def publish_map(self, cloud):
                self.until(lambda:self.maps.get_subscription_count()>0 and
                           self.odometry.get_subscription_count()>0, 'map/odometry discovery')
                previous = self.status('map_status')['map_revision']
                cloud.header.stamp = self.get_clock().now().to_msg()
                self.maps.publish(cloud)
                self.publish_odometry()
                self.until(lambda:self.status('map_status')['map_revision'] == previous+1, 'map acceptance')
                self.until(lambda:self.status()['vertices']>0, 'home initialized from odometry')

            def plan(self, hypothetical=False):
                first = len(self.metrics)
                request = PlannerSrv.Request()
                request.header.frame_id, request.bound_mode = 'map', 3
                # Upstream's odometry sentinel is all-zero, including w.
                request.root_pose.orientation.w = 0.
                if hypothetical:
                    request.root_pose = Pose()
                    request.root_pose.orientation.w = 1.
                response = self.call('planner', request, PlannerSrv)
                self.until(lambda:len(self.metrics)>first, 'planning metrics')
                return response, self.metrics[-1]

            def storage(self, load, path):
                request = PlannerStringTrigger.Request()
                request.message = str(path)
                response = self.call('gbplanner/'+('load_graph' if load else 'save_graph'),
                                     request, PlannerStringTrigger)
                require(response.success, self.label+' graph storage failed')

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

        def no_frontier_round(probe, expected, hypothetical=False):
            before = probe.status()['low_gain_iterations']
            response, metric = probe.plan(hypothetical)
            switch = metric['auto_global']
            require(metric['gain_evaluated'] and not metric['frontier_exists'] and metric['gain']>0.,
                    'Fixture must have positive free-space gain but no frontier: '+str(metric))
            require(switch['enabled'] and switch['count_before']==before and switch['count_after']==expected
                    and probe.status()['low_gain_iterations']==expected, 'Counter did not persist across requests')
            require(switch['counted'] == (not hypothetical), 'Hypothetical request changed counter eligibility')
            require(switch['triggered'] == (not hypothetical and before==3), 'Fourth-round trigger is wrong')
            if hypothetical:
                require(switch['reason']=='hypothetical_root', 'Hypothetical root was not distinguished')
            else:
                expected_status = response.K_REPOSITIONING if before==3 else response.K_FORWARD
                require(response.status==expected_status, f'Unexpected native PlannerSrv status {response.status}, expected {expected_status}: {metric}')
            if before!=3 or hypothetical:
                require(len(response.path)>1, 'Positive local gain did not return an executable path')
            return response, {'status':response.status, 'path':coordinates(response.path), 'metrics':metric}

        def pci_fourth(probe, expect_path):
            for expected in (1, 2, 3):
                no_frontier_round(probe, expected)
            probe.until(lambda:probe.status('control_state')['known_cells']>0, 'PCI map received')
            probe.publish_odometry()
            first_metric, first_global, first_path, first_command = (
                len(probe.metrics), len(probe.global_metrics), len(probe.local_paths), len(probe.commands))
            request = PciTrigger.Request()
            request.bound_mode, request.not_exe_path = 3, True
            accepted = probe.call('pci_trigger', request, PciTrigger)
            require(accepted.success, 'PCI preview trigger rejected')
            probe.until(lambda:len(probe.metrics)>first_metric, 'PCI local planning result')
            probe.until(lambda:probe.status('control_state')['state'] in {'PREVIEW', 'STOPPED', 'ERROR'},
                        'PCI terminal state')
            control = probe.status('control_state')
            probe.until(lambda:len(probe.global_metrics)>first_global and len(probe.local_paths)>first_path,
                        'Native global selection publication')
            require(len(probe.metrics)==first_metric+1 and len(probe.global_metrics)==first_global+1,
                    'Reposition response caused another local or global request')
            metric = probe.metrics[-1]
            switch = metric['auto_global']
            require(metric['gain']>0. and metric['gain_evaluated'] and not metric['frontier_exists']
                    and switch['counted'] and switch['triggered'] and switch['count_before']==3
                    and switch['count_after']==0 and probe.status()['low_gain_iterations']==0,
                    'PCI fourth request did not perform the persistent switch')
            require(sum(event['reason']=='local_requested' for event in control['events'])==1 and
                    not any('retry_bound_' in event['reason'] or event['reason']=='global_requested'
                            for event in control['events']), 'PCI retried a native reposition response')
            route = coordinates([entry.pose for entry in probe.local_paths[-1].poses])
            if expect_path:
                require(control['state']=='PREVIEW' and control['mode']=='global', 'Reposition did not become global preview')
                probe.until(lambda:len(probe.commands)>first_command and bool(probe.commands[-1].poses), 'Preview command path')
                command = coordinates([entry.pose for entry in probe.commands[-1].poses])
                require(len(route)>2 and command==route, 'PCI truncated or changed the native global route')
                require(probe.global_metrics[-1]['success'] and switch['reason']=='ok', 'No native frontier selected')
            else:
                require(control['state']=='STOPPED' and control['reason']=='global_frontier_unavailable'
                        and not route, 'Empty reposition did not stop without retrying')
                command = []
                require(switch['reason']=='no_global_frontier', 'Unexpected empty-global reason')
            return {'control':control, 'local_metrics':metric, 'global_metrics':probe.global_metrics[-1],
                    'native_route':route, 'preview_route':command}

        probe = Probe('check', common, controller=True)
        probe.publish_map(make_map())
        require(probe.status()['low_gain_iterations']==0, 'Counter did not start at zero')
        rows = []
        _, row = no_frontier_round(probe, 1)
        rows.append(row)
        _, hypothetical = no_frontier_round(probe, 1, hypothetical=True)
        for expected in (2, 3):
            _, row = no_frontier_round(probe, expected)
            rows.append(row)
        global_before = len(probe.global_metrics)
        response, fourth = no_frontier_round(probe, 0)
        probe.until(lambda:len(probe.global_metrics)>global_before, 'Fourth request called global planner')
        require(len(probe.global_metrics)==global_before+1 and not response.path
                and fourth['metrics']['auto_global']['reason']=='no_global_frontier',
                'Fourth positive-gain request did not invoke the native empty global branch')
        rows.append(fourth)
        summary['empty_global'] = {'rounds':rows, 'hypothetical':hypothetical,
                                   'pci':pci_fourth(probe, expect_path=False)}

        # A native wide local RRG discovers the distant unknown boundary and
        # retains its actual checked root-to-frontier edges in global memory.
        seed_parameters = dict(common)
        seed_parameters.update({'sampling.min':[-.5, -2., -.2], 'sampling.max':[5.5, 2., .2],
                                'planning.max_vertices':140, 'planning.max_attempts':3000})
        seed = Probe('seed', seed_parameters)
        remote = make_map(remote_frontier=True)
        seed.publish_map(remote)
        seed_response, seed_metric = seed.plan()
        seed_status = seed.status()
        require(seed_metric['success'] and seed_metric['frontier_exists'] and seed_status['frontiers']>0,
                'Native seed RRG failed to retain a distant volumetric frontier: '+str(seed_status))
        snapshot = output.parent/'native_frontier.gbmem'
        seed.storage(False, snapshot)
        require(snapshot.is_file() and snapshot.stat().st_size>0, 'Native graph snapshot missing')
        summary['seed'] = {'metrics':seed_metric, 'global_status':seed_status,
                           'local_path':coordinates(seed_response.path), 'snapshot':str(snapshot)}
        seed.close()
        fixtures.remove(seed)

        probe.publish_map(remote)
        probe.storage(True, snapshot)
        loaded = probe.status()
        require(loaded['frontiers']>0 and loaded['low_gain_iterations']==0, 'Native frontier graph did not load')
        summary['remote_frontier'] = {'loaded':loaded, 'pci':pci_fourth(probe, expect_path=True)}

        # With automatic homing enabled, an exhausted frontier set at the home
        # position yields one pose. It is already a global route and must not
        # be rejected by the local-reference-path two-point requirement.
        home_parameters = dict(common)
        home_parameters['planning.go_home_if_fully_explored'] = True
        home = Probe('home', home_parameters, controller=True)
        home.publish_map(make_map())
        for expected in (1, 2, 3):
            no_frontier_round(home, expected)
        home.until(lambda:home.status('control_state')['known_cells']>0, 'Homing PCI map received')
        first_metric, first_global, first_path = len(home.metrics), len(home.global_metrics), len(home.local_paths)
        request = PciTrigger.Request()
        request.bound_mode, request.not_exe_path = 3, True
        require(home.call('pci_trigger', request, PciTrigger).success, 'Homing PCI preview rejected')
        home.until(lambda:len(home.metrics)>first_metric and len(home.global_metrics)>first_global
                   and len(home.local_paths)>first_path, 'Single-pose homing publication')
        home.until(lambda:home.status('control_state')['state'] in {'DONE', 'PREVIEW', 'STOPPED', 'ERROR'},
                   'Single-pose homing terminal state')
        control = home.status('control_state')
        metric, global_metric = home.metrics[-1], home.global_metrics[-1]
        switch = metric['auto_global']
        route = coordinates([entry.pose for entry in home.local_paths[-1].poses])
        require(metric['success'] and metric['reason']=='fully_explored_homing' and global_metric['success']
                and len(route)==1 and route[0][:3]==[0., 0., 0.], 'Single-pose native homing route was rejected')
        require(metric['gain']>0. and metric['gain_evaluated'] and not metric['frontier_exists']
                and switch['counted'] and switch['triggered'] and switch['count_before']==3
                and switch['count_after']==0 and home.status()['low_gain_iterations']==0,
                'Single-pose homing did not originate from the fourth no-frontier round')
        # Local PCI changes mode to homing only when PlannerSrv returns K_HOMING.
        require(control['state']=='DONE' and control['reason']=='already_home' and control['mode']=='homing',
                'PCI failed to honor the single-pose K_HOMING response')
        require(len(home.metrics)==first_metric+1 and len(home.global_metrics)==first_global+1
                and sum(event['reason']=='local_requested' for event in control['events'])==1
                and not any('retry_bound_' in event['reason'] or event['reason'] in {'global_requested', 'homing_requested'}
                            for event in control['events']), 'Single-pose homing caused another planner request')
        summary['already_home'] = {'control':control, 'local_metrics':metric,
                                    'global_metrics':global_metric, 'native_route':route}
        summary['checks'] = {'positive_gain_no_frontier_switches_on_fourth':True,
                             'counter_persists_between_local_planners':True,
                             'hypothetical_root_does_not_count':True,
                             'empty_reposition_stops_without_retry':True,
                             'native_local_graph_supplies_remote_frontier':True,
                             'pci_retains_entire_global_preview_path':True,
                             'single_pose_homing_finishes_already_home':True}
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
        summary['last_metrics'] = {fixture.label:fixture.metrics[-12:] for fixture in fixtures}
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

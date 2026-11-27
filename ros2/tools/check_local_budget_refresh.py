#!/usr/bin/env python3
"""Check fresh local mission decisions while a real ROS planning call is busy.

The existing sampling_status event starts a /clock update, acknowledged by the
planner's independent mission_state service before gain/graph completion. Native
output stamps and local_budget_checks must corroborate that interval. Missing
the interval is a fixture failure, never evidence that budget handling passed.
The bounded workload is a 21-vertex native RRG with dense real gain rays; there
are no artificial computation delays, production hooks or motion consumers.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import traceback


RESOLUTION = .25
ORIGIN = (-4., -4., -2.)
DIMENSIONS = (36, 32, 16)
HOME = (-2., 0., 0.)
ROOT = (0., 0., 0.)
START_NS = 100_000_000_000
JUMP_NS = 9_000_000_000


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def coordinates(pose):
    return [pose.position.x, pose.position.y, pose.position.z]


def stamp_ns(header):
    return header.stamp.sec*1_000_000_000+header.stamp.nanosec


def distance(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=workspace/'results/local_budget_refresh/runtime.json')
    parser.add_argument('--timeout', type=float, default=120.)
    args = parser.parse_args()
    output = args.output.resolve()
    require(not output.exists(), 'Use a new output filename')
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    started = time.monotonic()
    deadline = started+args.timeout
    summary = {'status':'failed', 'scope':__doc__, 'fixtures':{}, 'cases':{},
               'scene':{'origin':ORIGIN, 'dimensions':DIMENSIONS, 'resolution':RESOLUTION,
                        'home':HOME, 'current_root':ROOT, 'clock_jump_s':9.},
               'source_sha256':hashlib.sha256((workspace/'src/gbplanner_local/src/local_planner_node.cpp').read_bytes()).hexdigest()}
    process = probe = None
    initialized = False
    try:
        import rclpy
        from nav_msgs.msg import Odometry, Path as RosPath
        from planner_msgs.msg import Graph
        from planner_msgs.srv import PlannerHoming, PlannerSrv
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        rclpy.init(args=[])
        initialized = True
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        scene = PointCloud2()
        scene.header.frame_id, scene.height = 'map', 1
        scene.width, scene.point_step = math.prod(DIMENSIONS), 16
        scene.row_step, scene.is_dense = scene.width*16, True
        scene.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x','y','z'))]
        scene.fields.append(PointField(name='occupancy', offset=12, datatype=PointField.INT8, count=1))
        data = bytearray(scene.row_step)
        offset = 0
        for z in range(DIMENSIONS[2]):
            for y in range(DIMENSIONS[1]):
                for x in range(DIMENSIONS[0]):
                    point = [base+(index+.5)*RESOLUTION for base, index in zip(ORIGIN, (x,y,z))]
                    # A known free corridor connects home and all sampled
                    # states. Unknown surroundings require actual ray queries.
                    known = -3.5 < point[0] < 4. and abs(point[1]) < .75 and abs(point[2]) < .75
                    struct.pack_into('<fffbxxx', data, offset, *point, 0 if known else -1)
                    offset += 16
        scene.data = bytes(data)
        summary['scene']['sha256'] = hashlib.sha256(scene.data).hexdigest()

        common = {
            'use_sim_time':True, 'map.resolution':RESOLUTION,
            'map.origin':list(ORIGIN), 'map.dimensions':list(DIMENSIONS),
            'robot.type':'kAerial', 'robot.size':[.1,.1,.1], 'robot.safety_extension':[.2,.2,.2],
            'global.enable':True, 'global.auto_home':True, 'global.background_enable':False,
            'planning.auto_global_planner_enable':False, 'planning.adaptive_exploration':False,
            'planning.path_safety_enhance_enable':True, 'planning.path_interpolation_distance':.2,
            'planning.traverse_length_max':10., 'planning.traverse_time_max':100.,
            'planning.path_length_penalty':0., 'planning.path_direction_penalty':0.,
            'planning.graph_mode':False, 'planning.max_vertices':21, 'planning.max_edges':100,
            'planning.max_attempts':60, 'planning.edge_min':.05, 'planning.edge_max':.15,
            'planning.overshoot':0., 'planning.leaf_gain_only':False,
            'planning.cluster_vertices_for_gain':False,
            'planning.ray_cast_method':'iterative', 'planning.nonuniform_ray_cast':False,
            'sampling.seed':42, 'sampling.seed_mode':'fixed',
            'sampling.min':[-3.,-3.,-1.5], 'sampling.max':[4.,3.,1.5],
            'sensor.range':6., 'sensor.horizontal_fov':2*math.pi, 'sensor.vertical_fov':math.pi/3,
            'sensor.horizontal_resolution':math.pi/60, 'sensor.vertical_resolution':math.pi/150,
            'gain.unknown':1., 'gain.free':.1, 'gain.occupied':0.,
            'visualization.planning_diagnostics':True, 'visualization.gain_vertex_id':20,
            'mission.enabled':True, 'mission.auto_homing':True,
            'mission.time_limit':30., 'mission.homing_speed':1.,
            'mission.auto_landing_enable':False, 'mission.time_budget_before_landing':8.,
        }
        for axis in ('x','y','z','heading'):
            common[f'sampling.{axis}.distribution'] = 'constant'
            common[f'sampling.{axis}.constant'] = 3. if axis=='x' else 0.

        class Probe(Node):
            def __init__(self, label, record):
                self.ns = f'/local_budget_{os.getpid()}_{label}'
                super().__init__('probe', namespace=self.ns)
                self.record = record
                self.sim_ns, self.x, self.next_publication = START_NS, HOME[0], 0.
                self.rpc_clients = {}
                self.metrics, self.gains, self.graphs, self.global_graphs, self.paths = [], [], [], [], []
                self.armed = self.active_future = self.clock_ack = None
                self.clocks = self.create_publisher(Clock, self.ns+'/clock', 10)
                self.odometry = self.create_publisher(Odometry, 'odometry', 10)
                self.maps = self.create_publisher(PointCloud2, 'voxel_map', durable)
                self.create_subscription(String, 'sampling_status', self.sample_started, durable)
                self.create_subscription(String, 'planning_metrics', lambda m:self.metrics.append(json.loads(m.data)), durable)
                self.create_subscription(String, 'planning_gain_geometry', self.gain_finished, durable)
                self.create_subscription(Graph, 'local_graph', self.graphs.append, durable)
                self.create_subscription(Graph, 'global_graph', self.global_graphs.append, durable)
                self.create_subscription(RosPath, 'local_path', self.paths.append, durable)
                self.planner = self.create_client(PlannerSrv, 'planner')
                self.mission = self.create_client(Trigger, 'mission_state')

            def publish_inputs(self):
                clock = Clock()
                clock.clock.sec, clock.clock.nanosec = divmod(self.sim_ns, 1_000_000_000)
                self.clocks.publish(clock)
                odom = Odometry()
                odom.header.frame_id, odom.child_frame_id = 'map', 'base_link'
                odom.header.stamp = clock.clock
                odom.pose.pose.position.x, odom.pose.pose.orientation.w = self.x, 1.
                self.odometry.publish(odom)

            def sample_started(self, message):
                data = json.loads(message.data)
                armed = self.armed
                if armed is None or data.get('caller')!='local' or data['request_index']!=armed['request_index']:
                    return
                if armed.get('sample_event') is not None:
                    return
                armed['sample_event'] = data
                armed['sample_received_wall_s'] = time.monotonic()-started
                armed['in_flight_at_sampling'] = self.active_future is not None and not self.active_future.done()
                self.sim_ns = armed['target_ns']
                self.publish_inputs()
                self.clock_ack = self.mission.call_async(Trigger.Request())

            def gain_finished(self, message):
                data = json.loads(message.data)
                if data.get('request')!='local':
                    return
                geometry = data['geometry']
                self.gains.append({'stamp_ns':data['stamp_ns'], 'map_revision':data['map_revision'],
                    'reason':geometry['reason'], 'entered':geometry['entered'], 'evaluated':geometry['evaluated'],
                    'total_endpoints':geometry['total_endpoints'], 'total_samples':geometry['total_samples'],
                    'received_wall_s':time.monotonic()-started})

            def check_clock_ack(self):
                if self.clock_ack is None or not self.clock_ack.done():
                    return
                acknowledged = json.loads(self.clock_ack.result().message)
                self.clock_ack = None
                armed = self.armed
                if armed is None:
                    return
                armed['mission_queries'] = armed.get('mission_queries',0)+1
                if abs(acknowledged['stamp_s']-armed['target_ns']/1e9) > 1e-8:
                    self.clock_ack = self.mission.call_async(Trigger.Request())
                    return
                armed['acknowledgement'] = {'mission':acknowledged,
                    'received_wall_s':time.monotonic()-started,
                    'local_future_pending':not self.active_future.done(),
                    'gain_not_published':len(self.gains)==armed['gain_before'],
                    'graph_not_published':len(self.graphs)==armed['graph_before'],
                    'metrics_not_published':len(self.metrics)==armed['metrics_before']}

            def spin(self):
                if process.poll() is not None:
                    raise RuntimeError(f'Planner exited {process.returncode}; see {self.record["log"]}')
                if time.monotonic()>=deadline:
                    raise TimeoutError('Local-budget fixture wall deadline')
                if time.monotonic()>=self.next_publication:
                    self.publish_inputs()
                    self.next_publication = time.monotonic()+.02
                rclpy.spin_once(self, timeout_sec=.005)
                self.check_clock_ack()

            def until(self, predicate, label, seconds=30.):
                until = min(deadline,time.monotonic()+seconds)
                while not predicate():
                    if time.monotonic()>=until:
                        raise TimeoutError(label)
                    self.spin()

            def call(self, name, request, kind=Trigger):
                if name not in self.rpc_clients:
                    self.rpc_clients[name] = self.create_client(kind, name)
                client = self.rpc_clients[name]
                self.until(client.service_is_ready, name+' discovery')
                future = client.call_async(request)
                self.until(future.done, name+' reply')
                return future.result()

            def read(self, name):
                return json.loads(self.call(name, Trigger.Request()).message)

            def setup(self):
                self.until(lambda:self.planner.service_is_ready() and self.mission.service_is_ready(), 'Planner discovery')
                self.until(lambda:self.read('mission_state')['stamp_s']==100., 'Initial ROS clock')
                scene.header.stamp.sec, scene.header.stamp.nanosec = divmod(self.sim_ns, 1_000_000_000)
                self.maps.publish(scene)
                self.until(lambda:self.read('map_status')['known_cells']>0, 'Known free corridor')
                for value in (-2.,-1.,0.):
                    self.x = value
                    self.publish_inputs()
                    self.until(lambda:self.global_graphs and any(
                        abs(vertex.pose.position.x-value)<1e-8 for vertex in self.global_graphs[-1].vertices),
                        'Observed graph vertex '+str(value))
                request = PlannerHoming.Request()
                request.header.frame_id, request.bound_mode = 'map', PlannerSrv.Request.K_EXACT_BOUND
                home = self.call('exploration_homing', request, PlannerHoming)
                route = [coordinates(pose) for pose in home.path]
                require(route and math.dist(route[0],ROOT)<1e-8 and math.dist(route[-1],HOME)<1e-8 and
                        math.isclose(distance(route),2.,abs_tol=1e-7), 'Fixture lacks a real 2 m return path')
                self.record['setup_homing_path'] = route
                initial = self.read('mission_state')
                require(initial['valid'] and initial['elapsed_s']==0 and initial['remaining_s']==30.,
                        'Mission clock was not paused at its initial time')
                self.record['initial_mission'] = initial

            def run(self, label, request_index, jump, hypothetical=False):
                request = PlannerSrv.Request()
                request.header.frame_id, request.bound_mode = 'map', PlannerSrv.Request.K_EXACT_BOUND
                request.header.stamp.sec, request.header.stamp.nanosec = divmod(self.sim_ns,1_000_000_000)
                request.root_pose.orientation.w = 1. if hypothetical else 0.
                armed = {'request_index':request_index, 'start_ns':self.sim_ns,
                    'target_ns':self.sim_ns+(JUMP_NS if jump else 0), 'hypothetical':hypothetical,
                    'gain_before':len(self.gains), 'graph_before':len(self.graphs),
                    'metrics_before':len(self.metrics), 'paths_before':len(self.paths)}
                summary['cases'][label] = armed
                self.armed = armed
                self.active_future = self.planner.call_async(request)
                self.until(lambda:self.active_future.done() and len(self.metrics)>armed['metrics_before'] and
                    len(self.gains)>armed['gain_before'] and len(self.graphs)>armed['graph_before'] and
                    len(self.paths)>armed['paths_before'], label+' native completion')
                self.check_clock_ack()
                response = self.active_future.result()
                metric, gain, graph, path = self.metrics[-1], self.gains[-1], self.graphs[-1], self.paths[-1]
                armed.update(metric=metric, gain=gain,
                    graph={'stamp_ns':stamp_ns(graph.header), 'vertices':len(graph.vertices), 'edges':len(graph.edges)},
                    output={'status':response.status, 'path':[coordinates(pose) for pose in response.path],
                            'stamp_ns':stamp_ns(path.header)})
                ack = armed.get('acknowledgement')
                require(armed.get('in_flight_at_sampling') and ack is not None and
                        all(ack[key] for key in ('local_future_pending','gain_not_published','graph_not_published','metrics_not_published')),
                        label+': fixture did not establish a live computation/clock acknowledgement interval')
                require(gain['stamp_ns']==stamp_ns(graph.header)==stamp_ns(path.header)==armed['target_ns'],
                        label+': native completion stamps do not confirm the advanced ROS clock')
                require(metric['sampling_request_index']==request_index and metric['graph_build_completed'] and
                        metric['gain_evaluated'] and metric['gain_evaluations']>=2 and len(graph.vertices)==21 and
                        gain['entered'] and gain['evaluated'] and gain['total_endpoints']>=5000,
                        label+': bounded native graph/gain work did not complete')
                require([coordinates(p.pose) for p in path.poses]==armed['output']['path'], 'Service and published path differ')
                self.armed = self.active_future = self.clock_ack = None
                return armed

        def stop_fixture():
            nonlocal process, probe
            if process is not None:
                cleanup = probe.record.setdefault('cleanup',{})
                forced = False
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=5.)
                    except subprocess.TimeoutExpired:
                        forced = True
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=2.)
                cleanup.update(exit_code=process.returncode, forced=forced)
                process = None
                require(not forced and cleanup['exit_code']==0, 'Planner did not exit normally')
            if probe is not None:
                probe.destroy_node()
                probe = None

        for profile in ('budget','disabled','landing'):
            parameters = dict(common)
            if profile=='disabled':parameters['mission.enabled'] = False
            if profile=='landing':parameters['mission.auto_landing_enable'] = True
            record = summary['fixtures'][profile] = {'parameters':parameters}
            probe = Probe(profile, record)
            command = ['ros2','run','gbplanner_local','local_planner_node','--ros-args',
                       '-r','__ns:='+probe.ns, '-r','/clock:='+probe.ns+'/clock']
            for key,value in parameters.items():command += ['-p',key+':='+json.dumps(value)]
            log = output.with_name(output.stem+'_'+profile+'.log')
            require(not log.exists(), 'Use new log filenames')
            record.update(command=command, log=str(log))
            with log.open('w') as stream:
                process = subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,
                                           start_new_session=True,env=dict(os.environ))
            probe.setup()
            if profile=='budget':
                baseline = probe.run('paused_clock',0,False)
                checks = baseline['metric']['local_budget_checks']
                require([check['stage'] for check in checks]==['entry','post_home','post_local','final'] and
                        all(check['stamp_s']==100. and check['remaining_s']==30. and check['decision']=='continue' for check in checks),
                        'Paused ROS clock was charged for wall-time graph computation')
                require(baseline['output']['status']==PlannerSrv.Response.K_FORWARD and baseline['output']['path'] and
                        baseline['metric']['refinement']['attempted'], 'Paused baseline lacks an actual refined outbound path')
                returning = probe.run('budget_crossed_during_local',1,True)
                checks = returning['metric']['local_budget_checks']
                require([check['stage'] for check in checks]==['entry','post_home','post_local'] and
                        all(check['stamp_s']==100. and check['decision']=='continue' for check in checks[:2]) and
                        checks[-1]['stamp_s']==109. and checks[-1]['remaining_s']==21. and checks[-1]['decision']=='homing' and
                        math.isclose(checks[-1]['home_distance_m'],2.,abs_tol=1e-7), 'Fresh post-local budget did not select home')
                require(returning['metric']['reason']=='mission_time_homing' and
                        returning['output']['status']==PlannerSrv.Response.K_HOMING and
                        math.dist(returning['output']['path'][0],ROOT)<1e-8 and
                        math.dist(returning['output']['path'][-1],HOME)<1e-8 and
                        math.isclose(distance(returning['output']['path']),2.,abs_tol=1e-7), 'Return response is not the real checked home route')
                hypothetical = probe.run('hypothetical_root_preserved',2,True,True)
                require(hypothetical['metric']['local_budget_checks']==[] and hypothetical['output']['status']==PlannerSrv.Response.K_FORWARD and
                        hypothetical['output']['path'], 'Hypothetical root unexpectedly enforced the live mission budget')
            elif profile=='disabled':
                disabled = probe.run('disabled_budget_preserved',0,True)
                require(disabled['metric']['local_budget_checks']==[] and disabled['output']['status']==PlannerSrv.Response.K_FORWARD and
                        disabled['output']['path'], 'Disabled mission budget rejected local planning')
            else:
                landing = probe.run('landing_due_during_local',0,True)
                checks = landing['metric']['local_budget_checks']
                require([check['stage'] for check in checks]==['entry','post_local'] and checks[0]['stamp_s']==100. and
                        checks[0]['decision']=='continue' and checks[-1]['stamp_s']==109. and
                        checks[-1]['landing_due'] and checks[-1]['decision']=='reject', 'Landing deadline did not refresh after real local work')
                require(landing['metric']['reason']=='landing_requested' and not landing['output']['path'] and
                        landing['output']['status']==-1 and landing['metric']['path_length_m']==0., 'Landing deadline released an outbound route')
            stop_fixture()
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'], summary['traceback'] = str(error), traceback.format_exc()
    finally:
        try:
            if process is not None or probe is not None:stop_fixture()
        except Exception as error:
            summary['status'] = 'failed'
            summary['cleanup_error'] = str(error)
        if initialized:rclpy.shutdown()
        summary['elapsed_s'] = time.monotonic()-started
        output.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'status':summary['status'],'cases':list(summary['cases']),'output':str(output),
                      'elapsed_s':summary['elapsed_s'],'error':summary.get('error')}))
    return 0 if summary['status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())

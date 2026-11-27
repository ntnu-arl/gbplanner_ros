#!/usr/bin/env python3
"""Check background eligibility and ROS-clock scheduling on the native planner.

Uses synthetic supported terrain, stationary odometry and a probe-owned /clock.
No motion consumer or explicit expansion request is present. A real local build
and, after a map reset, a real no-gain PlanAhead build enable the timer separately.
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
import sys
import time
import traceback

sys.dont_write_bytecode = True
from check_ground_background import (BASE_ROOT, DIMENSIONS, ORIGIN, PLANNING_ROOT,
                                     RESOLUTION, ROBOT_SIZE, graph_record, require, scene_data)


PERIOD = .5
TIME_BUDGET = .01


def block_root_body(data):
    """Keep all five support rays clear; occupy the body's Y-offset volume."""
    blocked = bytearray(data)
    changed = 0
    for offset in range(0, len(blocked), 16):
        x, y, z = struct.unpack_from('<fff', blocked, offset)
        if -.6 < x < .1 and .1 < y < .4 and .3 < z < .8:
            struct.pack_into('<b', blocked, offset+12, 100)
            changed += 1
    require(changed > 0, 'Root-body patch contains no cells')
    return bytes(blocked), changed


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=workspace/'results/background_clock/native.json')
    args = parser.parse_args()
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    namespace = f'/background_clock_{os.getpid()}'
    started = time.monotonic()
    deadline = started+90.
    process = node = None
    initialized = False
    summary = {'status': 'failed', 'scope': __doc__, 'checks': {},
               'scene': {'resolution': RESOLUTION, 'origin': ORIGIN, 'dimensions': DIMENSIONS,
                         'base_root': BASE_ROOT, 'planning_root': PLANNING_ROOT,
                         'robot_size': ROBOT_SIZE, 'center_offset': [0., .25, 0.],
                         'sample_offset': [6., 0., 0.], 'floor_surface_z': 0.}}
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from planner_msgs.msg import Graph
        from planner_msgs.srv import PlanAhead, PlannerSrv
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        rclpy.init(args=[])
        initialized = True
        # The probe uses wall time so it can query a paused simulated node.
        node = Node('probe', namespace=namespace)
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        clocks = node.create_publisher(Clock, '/clock', 10)
        maps = node.create_publisher(PointCloud2, 'voxel_map', durable)
        odometry = node.create_publisher(Odometry, 'odometry', 10)
        accepted, metrics, ahead_metrics, graphs = [], [], [], []
        node.create_subscription(PointCloud2, 'accepted_voxel_map', accepted.append, durable)
        node.create_subscription(String, 'planning_metrics',
                                 lambda message: metrics.append(json.loads(message.data)), durable)
        node.create_subscription(String, 'plan_ahead_metrics',
                                 lambda message: ahead_metrics.append(json.loads(message.data)), durable)
        node.create_subscription(Graph, 'global_graph', graphs.append, durable)
        planner = node.create_client(PlannerSrv, 'gbplanner')
        ahead = node.create_client(PlanAhead, 'planner_ahead')
        expansion_status = node.create_client(Trigger, 'expansion_status')
        global_status = node.create_client(Trigger, 'global_status')
        sensor_status = node.create_client(Trigger, 'sensor_gain')
        options = {
            'use_sim_time': True,
            'map.resolution': RESOLUTION, 'map.origin': list(ORIGIN), 'map.dimensions': list(DIMENSIONS),
            'robot.type': 'kGround', 'robot.size': list(ROBOT_SIZE), 'robot.center_offset': [0., .25, 0.],
            'ground.max_ground_height': .8, 'ground.robot_height': .55,
            'ground.max_inclination': .85, 'ground.max_negative_inclination': .85,
            'ground.interpolate_projection_distance': False, 'ground.ray_projection_mode': 'voxel_surface',
            'global.enable': True, 'global.auto_home': True, 'global.background_enable': True,
            'global.background_budget_mode': 'time', 'global.background_time_budget': TIME_BUDGET,
            'global.background_period': PERIOD,
            'planning.auto_global_planner_enable': False, 'planning.graph_mode': False,
            'planning.sampling_budget_mode': 'raw_draws', 'planning.max_attempts': 1,
            'planning.max_vertices': 2, 'planning.max_edges': 100,
            'planning.edge_min': .05, 'planning.edge_max': 1., 'planning.overshoot': 0.,
            'planning.path_safety_enhance_enable': False,
            'sampling.min': [-.2, -1.5, -.1], 'sampling.max': [6.5, 1.5, .3],
            'gain.unknown': 0., 'gain.free': 0., 'gain.occupied': 0.,
            'sensor.range': .5, 'sensor.horizontal_fov': 1., 'sensor.horizontal_resolution': 1.,
            'sensor.vertical_fov': .2, 'sensor.vertical_resolution': .2,
        }
        for axis in ('x', 'y', 'z', 'heading'):
            options[f'sampling.{axis}.distribution'] = 'constant'
            options[f'sampling.{axis}.constant'] = 6. if axis == 'x' else 0.
        launch = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args', '-r', '__ns:='+namespace]
        for key, value in options.items():
            launch += ['-p', key+':='+json.dumps(value)]
        summary['parameters'], summary['launch'] = options, launch
        log_path = output.with_suffix('.log')
        summary['log'] = str(log_path)
        with log_path.open('w') as log:
            process = subprocess.Popen(launch, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True, env=dict(os.environ))

        sim_ns = 10_000_000_000
        publish_inputs = True
        next_publication = 0.
        clock_publications = 0

        def stamp(message):
            message.sec, message.nanosec = divmod(sim_ns, 1_000_000_000)

        def spin():
            nonlocal next_publication, clock_publications
            now = time.monotonic()
            if process.poll() is not None:
                raise RuntimeError(f'Native planner exited; see {log_path}')
            if now > deadline:
                raise TimeoutError('Background clock probe exceeded its wall deadline')
            if publish_inputs and now >= next_publication:
                message = Clock()
                stamp(message.clock)
                clocks.publish(message)
                clock_publications += 1
                observed = Odometry()
                observed.header.frame_id = 'map'
                stamp(observed.header.stamp)
                observed.pose.pose.position.x, observed.pose.pose.position.y, observed.pose.pose.position.z = BASE_ROOT
                observed.pose.pose.orientation.w = 1.
                odometry.publish(observed)
                next_publication = now+.03
            rclpy.spin_once(node, timeout_sec=.01)

        def until(predicate, label):
            while not predicate():
                if time.monotonic() > deadline:
                    raise TimeoutError(label)
                spin()

        def hold(seconds):
            stop = time.monotonic()+seconds
            while time.monotonic() < stop:
                spin()

        def call(client, request=None):
            until(client.service_is_ready, 'Service discovery')
            future = client.call_async(Trigger.Request() if request is None else request)
            until(future.done, 'Service response')
            return future.result()

        def read(client):
            response = call(client)
            require(response.success, 'Status service failed: '+response.message)
            return json.loads(response.message)

        def clock_acknowledged():
            response = call(sensor_status)
            if not response.success:
                return False
            state = json.loads(response.message)
            return (state['clock_now_ns'] == sim_ns and state['odometry_stamp_ns'] == sim_ns
                    and math.dist(state['root'], PLANNING_ROOT) < 1e-8)

        def set_clock(seconds):
            nonlocal sim_ns, next_publication
            next_ns = round(seconds*1.e9)
            require(next_ns >= sim_ns, 'Fixture clock must never move backward')
            sim_ns, next_publication = next_ns, 0.
            until(clock_acknowledged, 'Planner clock and stationary odometry acknowledgement')

        def timer_tick(seconds):
            before = read(expansion_status)
            set_clock(seconds)
            until(lambda: read(expansion_status)['timer_ticks'] > before['timer_ticks'], 'ROS timer callback')
            after = read(expansion_status)
            require(after['last_timer_ros_ns'] == sim_ns, 'Timer did not run at the supplied ROS time')
            return after

        def publish_scene(data):
            cloud = PointCloud2()
            cloud.header.frame_id = 'map'
            stamp(cloud.header.stamp)
            cloud.height, cloud.width = 1, len(data)//16
            cloud.point_step, cloud.row_step = 16, len(data)
            cloud.is_dense, cloud.is_bigendian = True, False
            cloud.fields = [PointField(name=name, offset=index*4, datatype=PointField.FLOAT32, count=1)
                            for index, name in enumerate(('x', 'y', 'z'))]
            cloud.fields.append(PointField(name='occupancy', offset=12, datatype=PointField.INT8, count=1))
            cloud.data = data
            first = len(accepted)
            maps.publish(cloud)
            until(lambda: len(accepted) > first, 'Voxel snapshot acceptance')
            require(bytes(accepted[-1].data) == data, 'Accepted map differs from the fixture')

        def snapshot():
            status = read(global_status)
            until(lambda: graphs and len(graphs[-1].vertices) == status['vertices']
                  and len(graphs[-1].edges) == status['edges'], 'Global graph snapshot')
            return {'status': status, 'graph': graph_record(graphs[-1])}

        def live_local():
            request = PlannerSrv.Request()
            request.header.frame_id, request.bound_mode = 'map', PlannerSrv.Request.K_EXACT_BOUND
            stamp(request.header.stamp)
            # Humble's Pose defaults to an identity quaternion (w=1). The
            # service selects live odometry only for an exactly all-zero pose.
            request.root_pose.orientation.w = 0.
            first = len(metrics)
            response = call(planner, request)
            until(lambda: len(metrics) > first, 'Local planning metrics')
            return {'metric': metrics[-1], 'response_status': response.status, 'path_points': len(response.path)}

        until(lambda: maps.get_subscription_count() > 0 and clocks.get_subscription_count() > 0
              and odometry.get_subscription_count() > 0 and planner.service_is_ready(), 'Planner discovery')
        supported, counts = scene_data(True)
        blocked, changed = block_root_body(supported)
        summary['scene'].update(voxel_counts=counts, supported_sha256=hashlib.sha256(supported).hexdigest(),
                                blocked_sha256=hashlib.sha256(blocked).hexdigest(), blocked_body_cells=changed)
        publish_scene(supported)
        set_clock(10.)
        initial = read(expansion_status)
        require(initial['timer_enabled'] and initial['timer_clock'] == 'ros'
                and initial['compute_budget_clock'] == 'steady'
                and initial['trigger_policy'] == 'completed_live_or_ahead_graph'
                and math.isclose(initial['timer_period_seconds'], PERIOD)
                and math.isclose(initial['time_budget_seconds'], TIME_BUDGET), 'Wrong native timer configuration')
        never_built = timer_tick(10.6)
        require(not never_built['local_triggered'] and never_built['updates'] == 0,
                'Clock progression enabled background work without a real graph build')
        summary['never_built'] = never_built

        publish_scene(blocked)
        failed = live_local()
        summary['prebuild_failure'] = failed
        require(math.dist(failed['metric']['planning_root'], PLANNING_ROOT) < 1e-8
                and failed['metric']['auto_global']['reason'] != 'hypothetical_root',
                'The prebuild check did not use live stationary odometry')
        require(failed['metric']['reason'] == 'ground_root_occupied'
                and not failed['metric']['graph_build_completed'] and failed['metric']['attempts'] == 0
                and failed['metric']['sampling_stop_reason'] == 'not_started', 'Root-body failure reached sampling')
        failed['after_timer'] = timer_tick(11.1)
        require(not failed['after_timer']['local_triggered'] and failed['after_timer']['updates'] == 0,
                'A live request that failed before graph building enabled background work')

        publish_scene(supported)
        aligned = timer_tick(12.)
        summary['local'] = {'aligned_timer': aligned}
        require(not aligned['local_triggered'] and aligned['updates'] == 0, 'Restoring the map granted eligibility')
        set_clock(12.01)
        local = live_local()
        summary['local']['request'] = local
        require(math.dist(local['metric']['planning_root'], PLANNING_ROOT) < 1e-8
                and local['metric']['auto_global']['reason'] != 'hypothetical_root',
                'The completed local build did not use live stationary odometry')
        require(local['metric']['graph_build_completed'] and local['metric']['attempts'] == 1
                and local['metric']['vertices'] == 2 and local['metric']['reason'] == 'no_gain',
                'The live no-gain request did not complete the configured graph build')
        eligible = read(expansion_status)
        summary['local']['eligible'] = eligible
        before = snapshot()
        summary['local']['before_graph'] = before
        require(eligible['local_triggered'] and eligible['updates'] == 0
                and before['status']['vertices'] == 1 and before['status']['edges'] == 0,
                'Eligibility or the unvisited root-only graph is incorrect')
        set_clock(12.49)
        hold(.15)
        short = read(expansion_status)
        summary['local']['before_period'] = short
        require(short['timer_ticks'] == aligned['timer_ticks'] and short['updates'] == 0,
                'Background work ran before the next ROS timer period')
        expanded = timer_tick(12.51)
        summary['local']['expansion'] = expanded
        after = snapshot()
        summary['local']['after_graph'] = after
        require(expanded['updates'] == 1 and expanded['added'] > 0
                and after['status']['vertices']-before['status']['vertices'] == expanded['added']
                and after['status']['edges'] > before['status']['edges'],
                'Crossing the timer period did not grow the actual global graph')

        publish_inputs = False
        paused_at, publications_before_pause = time.monotonic(), clock_publications
        hold(.7)
        query_started = time.monotonic()
        paused_status, paused_sensor = read(expansion_status), read(sensor_status)
        paused_graph = snapshot()
        paused_wall = time.monotonic()-paused_at
        require(paused_wall > PERIOD and clock_publications == publications_before_pause,
                'Clock messages were published during the intended pause')
        require(paused_status['updates'] == expanded['updates']
                and paused_status['timer_ticks'] == expanded['timer_ticks']
                and paused_graph['graph'] == after['graph'] and paused_sensor['clock_now_ns'] == sim_ns,
                'Frozen /clock allowed timer callbacks or background graph changes')
        summary['pause'] = {'wall_seconds': paused_wall, 'status_query_seconds': time.monotonic()-query_started,
                            'clock_messages': clock_publications-publications_before_pause,
                            'expansion': paused_status, 'sensor': paused_sensor}
        publish_inputs = True
        resumed = timer_tick(13.01)
        require(resumed['updates'] == paused_status['updates']+1, 'Clock resumption did not resume background work')
        summary['resume'] = resumed

        # Reset qualification, then test the previously missed no-gain Ahead
        # branch independently of the successful ordinary build above.
        publish_scene(b'')
        reset = read(expansion_status)
        require(not reset['local_triggered'] and reset['updates'] == 0, 'Empty-map reset retained background eligibility')
        publish_scene(supported)
        set_clock(13.1)
        ahead_before = snapshot()
        require(not read(expansion_status)['local_triggered'] and ahead_before['status']['vertices'] == 1,
                'Map restoration or odometry granted eligibility before Ahead')
        request = PlanAhead.Request()
        request.header.frame_id, request.bound_mode, request.arrival_time = 'map', PlannerSrv.Request.K_EXACT_BOUND, 1.
        stamp(request.header.stamp)
        request.root_pose.position.x, request.root_pose.position.y, request.root_pose.position.z = BASE_ROOT
        request.root_pose.orientation.w = 1.
        first = len(ahead_metrics)
        response = call(ahead, request)
        until(lambda: len(ahead_metrics) > first, 'Ahead planning metrics')
        ahead_metric = ahead_metrics[-1]
        require(not response.success and response.reason == 'no_gain'
                and ahead_metric['plan']['graph_build_completed'] and ahead_metric['plan']['attempts'] == 1
                and read(expansion_status)['local_triggered'], 'Completed no-gain Ahead build did not enable the timer')
        ahead_expansion = timer_tick(13.51)
        ahead_after = snapshot()
        require(ahead_expansion['updates'] == 1 and ahead_expansion['added'] > 0
                and ahead_after['status']['vertices']-ahead_before['status']['vertices'] == ahead_expansion['added'],
                'No-gain Ahead eligibility did not lead to real timer-driven graph growth')
        summary['ahead'] = {'reset': reset, 'metric': ahead_metric, 'response_reason': response.reason,
                            'response_success': response.success, 'expansion': ahead_expansion,
                            'before_graph': ahead_before, 'after_graph': ahead_after}
        summary['checks'] = {'no_build_no_background': True, 'prebuild_failure_does_not_qualify': True,
                             'live_no_gain_build_qualifies': True, 'ros_period_boundary_respected': True,
                             'timer_grows_actual_graph': True, 'frozen_clock_stops_timer': True,
                             'status_services_respond_while_paused': True, 'clock_resumption_resumes_timer': True,
                             'map_reset_clears_eligibility': True, 'ahead_no_gain_build_qualifies': True}
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'], summary['traceback'] = str(error), traceback.format_exc()
    finally:
        if process is not None:
            forced = False
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    forced = True
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=5)
            summary['planner_exit'], summary['forced_termination'] = process.returncode, forced
            if forced or process.returncode != 0:
                summary['status'] = 'failed'
                summary.setdefault('error', 'Native planner did not exit normally')
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()
        summary['elapsed_s'] = time.monotonic()-started
        output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'status': summary['status'], 'output': str(output),
                      'elapsed_s': summary['elapsed_s'], 'error': summary.get('error')}))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

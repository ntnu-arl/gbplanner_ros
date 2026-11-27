#!/usr/bin/env python3
"""One native ROS stop-observe-replan run, followed by automatic homing.

Production planner, PCI and follower run as separate processes. The fixture
publishes synthetic occupancy and odometry and consumes timed trajectories;
it never supplies planner replies or Action outcomes. This checks recovery
after a dynamic obstacle clears, not robot dynamics or persistent-wall routing.
"""
import argparse
import copy
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


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=workspace/'results/path_invalidation_recovery/summary.json')
    parser.add_argument('--timeout', type=float, default=75.)
    args = parser.parse_args()
    require(math.isfinite(args.timeout) and args.timeout > 0., 'Invalid timeout')
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    started, initialized, node, processes = time.monotonic(), False, None, []
    deadline = started+args.timeout
    report = {'status': 'failed', 'scope': __doc__, 'launches': [], 'checks': {}}
    try:
        import rclpy
        from action_msgs.msg import GoalStatus
        from ament_index_python.packages import get_package_prefix
        from nav_msgs.msg import Odometry, Path as RosPath
        from planner_msgs.srv import PciTrigger
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2
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
        durable = QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        parameters = {
            'map.origin': list(ORIGIN), 'map.dimensions': list(DIM), 'map.resolution': RES,
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
            parameters[f'sampling.{axis}.distribution'] = 'constant'
            parameters[f'sampling.{axis}.constant'] = 4. if axis=='x' else 0.
        report['planner_parameters'] = parameters

        class Probe(Node):
            def __init__(self):
                self.fixture_namespace = f'/invalidation_recovery_{os.getpid()}'
                super().__init__('probe', namespace=self.fixture_namespace)
                self.position = self.home = (-2., 0., 0., 0.)
                self.command, self.control, self.execution = None, {}, {}
                self.service_clients, self.trigger_calls = {}, 0
                self.history = {name: [] for name in
                                ('control', 'execution', 'planning', 'global', 'maps', 'outcomes', 'trajectories')}
                self.native_paths, self.map_publications = [], []
                self.maps = self.create_publisher(PointCloud2, 'voxel_map', durable)
                self.odometry = self.create_publisher(Odometry, 'odometry', 10)
                self.create_subscription(RosPath, 'local_path', self.local_path, durable)
                for topic, key in (('control_status', 'control'), ('execution_status', 'execution'),
                                   ('planning_metrics', 'planning'), ('global_metrics', 'global'), ('map_updates', 'maps')):
                    self.create_subscription(String, topic, lambda message, name=key:self.receive(name, message), durable)
                self.create_subscription(MultiDOFJointTrajectory, 'command/trajectory', self.consume, 10)
                self.timer = self.create_timer(.02, self.tick)

            def now(self):
                return self.get_clock().now().nanoseconds*1.e-9

            def receive(self, name, message):
                data = json.loads(message.data)
                row = {'received_s': time.monotonic()-started, 'received_ros_s': self.now(), 'data': data}
                self.history[name].append(row)
                if name=='control':
                    self.control = data
                elif name=='execution':
                    self.execution = data
                    outcome = data.get('terminal_outcome')
                    if outcome and not any(item['data']==outcome for item in self.history['outcomes']):
                        self.history['outcomes'].append(dict(row, data=copy.deepcopy(outcome)))

            def local_path(self, message):
                if message.poses:
                    self.native_paths.append([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                                              for p in message.poses])

            def consume(self, message):
                points, times = decode(message)
                self.history['trajectories'].append({'received_s': time.monotonic()-started,
                                                     'points': points, 'times': times})
                self.command = points, times, time.monotonic(), self.position

            def tick(self):
                if self.command:
                    points, times, start, initial = self.command
                    self.position = sample(points, times, time.monotonic()-start, initial)
                message = Odometry()
                message.header.frame_id, message.child_frame_id = 'map', 'base_link'
                message.header.stamp = self.get_clock().now().to_msg()
                message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = self.position[:3]
                message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(self.position[3]/2), math.cos(self.position[3]/2)
                self.odometry.publish(message)

            def until(self, predicate, label, seconds=15.):
                limit = min(deadline, time.monotonic()+seconds)
                while not predicate():
                    for process, log in processes:
                        if process.poll() is not None:
                            raise RuntimeError(f'Native node exited {process.returncode}: {log}')
                    if time.monotonic() >= limit:
                        raise TimeoutError(label+'; control='+json.dumps(self.control))
                    rclpy.spin_once(self, timeout_sec=.02)

            def call(self, name, request, kind=Trigger):
                if name not in self.service_clients:
                    self.service_clients[name] = self.create_client(kind, name)
                client = self.service_clients[name]
                self.until(client.service_is_ready, name+' discovery')
                future = client.call_async(request)
                self.until(future.done, name+' reply')
                return future.result()

            def status(self, name):
                return json.loads(self.call(name, Trigger.Request()).message)

            def publish_map(self, cloud, label):
                before = self.status('map_status')['map_revision']
                cloud.header.stamp = self.get_clock().now().to_msg()
                stamp = cloud.header.stamp.sec+cloud.header.stamp.nanosec*1.e-9
                self.maps.publish(cloud)
                self.until(lambda:self.status('map_status')['map_revision']>before, label+' accepted')
                revision = self.status('map_status')['map_revision']
                record = {'label': label, 'stamp_s': stamp, 'revision': revision,
                          'received_s': time.monotonic()-started}
                self.map_publications.append(record)
                return record

            def outcome(self, success):
                return next((row for row in self.history['outcomes']
                             if row['data'].get('success') is success), None)

        node = Probe()
        launches = [('gbplanner_local', 'local_planner_node', parameters),
                    ('gbplanner_local', 'control_node.py',
                     {'planner_trigger_lead_time': 0., 'robot_type': 'kAerial', 'max_cycles': 1,
                      'path_invalidation_retries': 3, 'request_timeout': 10., 'execution_timeout': 30.}),
                    ('gbplanner_execution', 'path_follower_node.py',
                     {'speed': .8, 'yaw_rate': 1., 'output_type': 'kTopic'})]
        for package, executable, values in launches:
            command = ['ros2', 'run', package, executable, '--ros-args', '-r', '__ns:='+node.fixture_namespace]
            for key, value in values.items():
                command += ['-p', key+':='+json.dumps(value)]
            log = output.parent/(executable+'.log')
            with log.open('w') as stream:
                process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                           start_new_session=True, env=dict(os.environ))
            processes.append((process, log));report['launches'].append(command)

        node.until(lambda:node.maps.get_subscription_count()>0 and node.odometry.get_subscription_count()>=3,
                   'native subscriptions')
        free = make_map(known=True, wall=False)
        node.publish_map(copy.deepcopy(free), 'initial_free')
        node.until(lambda:node.status('global_status')['vertices']>0 and
                   node.status('control_state')['known_cells']>0 and
                   node.status('execution_state')['command_subscribers']>0, 'native readiness')
        request = PciTrigger.Request()
        request.bound_mode, request.set_auto, request.not_exe_path = 3, True, False
        node.trigger_calls += 1
        require(node.call('pci_trigger', request, PciTrigger).success, 'Automatic trigger rejected')
        node.until(lambda:node.native_paths and node.control.get('state')=='EXECUTING' and
                   node.control.get('follower_tracks_route') is True and
                   node.execution.get('state')=='EXECUTING' and
                   math.dist(node.position[:3], node.home[:3])>.15, 'first real local movement')
        original_execution = node.status('execution_state')
        original = node.native_paths[0]
        require(original[-1][0]-node.position[0]>1., 'Initial route is too short to stop before an obstacle')
        block_index = math.floor((node.position[0]+.65-ORIGIN[0])/RES)
        wall_min = ORIGIN[0]+block_index*RES
        require(wall_min-node.position[0]>.3 and wall_min+RES<original[-1][0], 'Invalid forward wall placement')
        blocked = copy.deepcopy(free);data = bytearray(blocked.data);changed = 0
        for offset in range(0, len(data), blocked.point_step):
            x = struct.unpack_from('<f', data, offset)[0]
            if wall_min < x < wall_min+RES:
                data[offset+12] = 100;changed += 1
        require(changed>0, 'No obstacle cells selected')
        blocked.data = bytes(data)
        report['obstacle'] = {'x_min': wall_min, 'x_max': wall_min+RES, 'cells': changed,
                              'position_at_publication': list(node.position), 'original_route': original}
        blocked_map = node.publish_map(blocked, 'forward_obstacle')
        node.until(lambda:node.outcome(False) is not None, 'native failed terminal outcome')
        failure_row = node.outcome(False);failure = failure_row['data']
        fields = {'goal_id', 'generation', 'action_status', 'success', 'reason', 'failure_map_revision',
                  'validated_revision', 'stop_confirmed', 'confirmation', 'stop_stamp_s', 'finished_ros_s'}
        require(fields<=failure.keys(), 'Terminal outcome lacks recovery evidence')
        require(failure['reason']=='path_invalidated:path_not_free' and
                failure['action_status']==GoalStatus.STATUS_ABORTED and failure['stop_confirmed'] is True and
                failure['goal_id']==original_execution['goal_id'] and
                failure['generation']==original_execution['generation'] and
                failure['failure_map_revision']==blocked_map['revision'] and
                failure['failure_map_revision']>failure['validated_revision'],
                'Native outcome did not prove a confirmed map-invalidation stop')
        require(failure['stop_stamp_s']>0 and failure['finished_ros_s']>=failure['stop_stamp_s'],
                'Stop and terminal timestamps are inconsistent')
        node.until(lambda:node.control.get('state')=='WAITING_RECOVERY_OBSERVATION',
                   'PCI accepted the matching terminal outcome')
        waiting = node.status('control_state')
        require(waiting['cycles']==0 and waiting['completed_actions']==0 and waiting['bound_mode']==3,
                'Failed action consumed a successful cycle or changed the body bound')
        waiting_recovery = waiting['path_invalidation_recovery']
        pending = waiting_recovery['pending']
        require(waiting_recovery['limit']==3 and waiting_recovery['attempts']==1 and
                waiting_recovery['consecutive_attempts']==1 and pending is not None and
                pending['goal_id']==failure['goal_id'] and pending['generation']==failure['generation'],
                'Pending recovery does not identify the confirmed failed action')
        require(not node.execution.get('active') and node.execution.get('hold_commands', 0)>=1,
                'Follower remained active or omitted hold output')
        require(len(node.history['planning'])==1, 'Recovery planning started before the post-stop observation')
        report['confirmed_stop'] = {'outcome': failure, 'control': waiting, 'position': list(node.position)}
        cleared_map = node.publish_map(copy.deepcopy(free), 'obstacle_cleared_after_stop')
        require(cleared_map['stamp_s']>=failure['stop_stamp_s'], 'Replacement map predates confirmed stop')
        node.until(lambda:node.control.get('state')=='WAITING_OBSERVATION' and
                   node.control.get('mode')=='local' and node.control.get('completed_actions')==1,
                   'replacement local Action success', seconds=25.)
        local_done = node.status('control_state')
        require(local_done['cycles']==1 and local_done['bound_mode']==3,
                'Replacement local action has incorrect cycle or bound accounting')
        require(len(node.history['planning'])==2 and len(node.native_paths)==2,
                'Expected exactly one original and one replacement native local plan')
        require(node.history['planning'][1]['received_s']>=failure_row['received_s'] and
                all(row['data']['bound_mode']==3 for row in node.history['planning']),
                'Replacement planning preceded the stop or changed bound mode')
        recovery = local_done['path_invalidation_recovery']
        require(recovery['limit']==3 and recovery['attempts']==1 and
                recovery['consecutive_attempts']==0 and recovery['pending'] is None,
                'Successful replacement did not clear consecutive failure accounting')
        events = [event for event in recovery['events'] if 'replan_map_revision' in event]
        require(events, 'No recovery replan evidence recorded')
        event = events[-1]
        require(event['goal_id']==failure['goal_id'] and event['generation']==failure['generation'] and
                event['attempt']==1 and event['consecutive_attempt']==1 and event['bound_mode']==3 and
                event['failure_map_revision']==failure['failure_map_revision'] and
                event['confirmed_stop_stamp_s']==failure['stop_stamp_s'] and
                event['replan_map_revision']==cleared_map['revision'] and
                event['replan_map_revision']>failure['failure_map_revision'] and
                event['replan_map_stamp_s']>=failure['stop_stamp_s'],
                'Recovery is not tied to the stopped goal and a new post-stop map')
        node.until(lambda:node.outcome(True) is not None, 'replacement native successful terminal outcome')
        report['replacement_complete'] = {'control': local_done, 'recovery_event': event,
                                           'outcome': node.outcome(True)['data']}
        node.publish_map(copy.deepcopy(free), 'replacement_endpoint_observation')
        node.until(lambda:node.control.get('state') in {'DONE', 'ERROR', 'STOPPED'},
                   'automatic homing completion', seconds=25.)
        final, execution = node.status('control_state'), node.status('execution_state')
        require(final['state']=='DONE' and final['mode']=='homing' and final['reason']=='returned_home' and
                final['completed_actions']==2 and math.dist(node.position[:3], node.home[:3])<.05,
                'Automatic homing did not return to the original home')
        require(execution['accepted']==3 and execution['aborted']==1 and execution['succeeded']==2 and
                execution['cancelled']==0 and execution['continuations']==0,
                'Unexpected Action accounting in the single recovery scenario')
        require(node.trigger_calls==1 and len(node.history['planning'])==2 and
                final['path_invalidation_recovery']['attempts']==1 and
                final['path_invalidation_recovery']['consecutive_attempts']==0,
                'Recovery required another trigger or performed extra local attempts')
        report.update(final_control=final, final_execution=execution, final_position=list(node.position))
        report['checks'] = dict.fromkeys(('native_inflight_map_invalidation', 'confirmed_stop_before_replanning',
            'new_map_acquired_after_stop', 'matching_goal_recovery', 'failed_action_not_counted_as_success',
            'exact_bound_preserved', 'single_external_trigger', 'replacement_action_succeeded', 'automatic_homing_completed'), True)
        report['status'] = 'passed'
    except Exception as error:
        report['error'], report['traceback'] = str(error), traceback.format_exc()
    finally:
        if node is not None:
            node.timer.cancel()
            report.update(history=node.history, map_publications=node.map_publications,
                          trigger_calls=node.trigger_calls, last_position=list(node.position))
        exits, cleanup_errors = [], []
        for process, log in reversed(processes):
            try:
                if process.poll() is None:
                    try:
                        os.killpg(process.pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
                    try:
                        process.wait(timeout=5.)
                    except subprocess.TimeoutExpired:
                        try:
                            os.killpg(process.pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                        process.wait(timeout=2.)
            except Exception as error:
                cleanup_errors.append(str(log)+': '+str(error))
            exits.append({'log': str(log), 'exit': process.returncode})
        report['process_exits'] = exits
        if cleanup_errors or any(item['exit']!=0 for item in exits):
            report['status'], report['cleanup_error'] = 'failed', 'Native node exited nonzero'
        try:
            if node is not None:
                node.destroy_node()
            if initialized and rclpy.ok():
                rclpy.shutdown()
        except Exception as error:
            report['status'] = 'failed';cleanup_errors.append(str(error))
        if cleanup_errors:
            report['cleanup_errors'] = cleanup_errors
        report['elapsed_s'] = time.monotonic()-started
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'status': report['status'], 'output': str(output), 'error': report.get('error')}))
    return 0 if report['status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())

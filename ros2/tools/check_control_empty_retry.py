#!/usr/bin/env python3
"""Native PCI empty-local retries and planner-owned frontier/homing decisions.

Runs the real local planner and controller on synthetic occupancy and stationary
odometry. The test ActionServer only supplies discovery and rejects any goal;
all expected routes are empty or already-home, so no motion is executed.
Metrics are observed here as evidence, never used by production PCI decisions.
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


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=workspace/'results/control_empty_retry/summary.json')
    parser.add_argument('--timeout', type=float, default=75.)
    args = parser.parse_args()
    require(math.isfinite(args.timeout) and args.timeout > 0., 'Timeout must be positive')
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    started, fixtures, initialized = time.monotonic(), [], False
    deadline = started+args.timeout
    summary = {'status': 'failed', 'scope': __doc__, 'checks': {}, 'cases': {}, 'launches': []}
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from planner_msgs.action import PathFollower
        from planner_msgs.srv import PciGlobal, PciHomingTrigger, PciStop, PciTrigger
        from rclpy.action import ActionServer, GoalResponse
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        sys.dont_write_bytecode = True
        sys.path.insert(0, str(workspace/'src/gbplanner_local/scripts'))
        from local_demo import DIM, ORIGIN, RES, make_map

        rclpy.init(args=[])
        initialized = True
        durable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        common = {
            'map.resolution': RES, 'map.origin': list(ORIGIN), 'map.dimensions': list(DIM),
            'global.enable': True, 'global.auto_home': True, 'global.background_enable': False,
            'robot.size': [.1, .1, .1], 'planning.graph_mode': False,
            'planning.max_attempts': 21, 'planning.empty_graph_cutoff': 7,
            'planning.max_vertices': 10, 'planning.max_edges': 100,
            'planning.edge_min': .05, 'planning.edge_max': 5., 'planning.overshoot': 0.,
            'sampling.min': [-.5, -.5, -.1], 'sampling.max': [4.5, .5, .1],
            'gain.free': 0., 'planning.path_direction_penalty': 0.,
            'planning.traverse_length_max': 10., 'planning.traverse_time_max': 100.,
            'planning.path_safety_enhance_enable': False,
            'sensor.range': .5, 'sensor.horizontal_fov': 1., 'sensor.horizontal_resolution': 1.,
            'sensor.vertical_fov': .2, 'sensor.vertical_resolution': .2,
        }
        for axis in ('x', 'y', 'z', 'heading'):
            common[f'sampling.{axis}.distribution'] = 'constant'
            common[f'sampling.{axis}.constant'] = 4. if axis == 'x' else 0.

        class Probe(Node):
            def __init__(self, label, wall, go_home):
                self.namespace_name = f'/control_empty_retry_{os.getpid()}_{label}'
                super().__init__('probe', namespace=self.namespace_name)
                self.label, self.wall, self.closed = label, wall, False
                self.processes, self.service_clients, self.metrics, self.globals, self.goals = [], {}, [], [], []
                self.maps = self.create_publisher(PointCloud2, 'voxel_map', durable)
                self.odometry = self.create_publisher(Odometry, 'odometry', 10)
                self.create_subscription(String, 'planning_metrics',
                    lambda m:self.metrics.append({'received_s': time.monotonic(), 'data': json.loads(m.data)}), durable)
                self.create_subscription(String, 'global_metrics',
                    lambda m:self.globals.append(json.loads(m.data)), durable)
                self.server = ActionServer(self, PathFollower, 'path_follower',
                    goal_callback=self.reject_goal, execute_callback=lambda handle:PathFollower.Result())
                self.timer = self.create_timer(.05, self.publish_odometry)
                fixtures.append(self)
                settings = dict(common, **{'planning.go_home_if_fully_explored': go_home})
                self.launch('local_planner_node', settings)
                # A one-action cap must not turn empty planning attempts into
                # executed cycles or cause an implicit home request.
                self.launch('control_node.py', {'max_cycles': 1, 'planner_trigger_lead_time': 0.})

            def reject_goal(self, request):
                self.goals.append(len(request.path))
                return GoalResponse.REJECT

            def launch(self, executable, settings):
                command = ['ros2', 'run', 'gbplanner_local', executable, '--ros-args',
                           '-r', '__ns:='+self.namespace_name]
                for key, value in settings.items():
                    command += ['-p', key+':='+json.dumps(value)]
                log_path = output.parent/f'{self.label}_{executable}.log'
                with log_path.open('w') as stream:
                    process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                               start_new_session=True, env=dict(os.environ))
                self.processes.append((process, log_path))
                summary['launches'].append(command)

            def until(self, predicate, label, seconds=12.):
                limit = min(deadline, time.monotonic()+seconds)
                while not predicate():
                    for process, log in self.processes:
                        if process.poll() is not None:
                            raise RuntimeError(f'{self.label} exited {process.returncode}; see {log}')
                    if time.monotonic() >= limit:
                        raise TimeoutError(self.label+': '+label)
                    rclpy.spin_once(self, timeout_sec=.02)

            def spin_for(self, seconds):
                until_time = time.monotonic()+seconds
                self.until(lambda:time.monotonic() >= until_time, 'bounded observation window', seconds+1.)

            def call(self, name, request, kind=Trigger):
                if name not in self.service_clients:
                    self.service_clients[name] = self.create_client(kind, name)
                client = self.service_clients[name]
                self.until(client.service_is_ready, name+' discovery')
                pending = client.call_async(request)
                self.until(pending.done, name+' response')
                return pending.result()

            def status(self, name='control_state'):
                return json.loads(self.call(name, Trigger.Request()).message)

            def publish_odometry(self):
                message = Odometry()
                message.header.frame_id, message.child_frame_id = 'map', 'base_link'
                message.header.stamp = self.get_clock().now().to_msg()
                message.pose.pose.position.x, message.pose.pose.position.y = -2., 2.
                message.pose.pose.orientation.w = 1.
                self.odometry.publish(message)

            def initialize(self):
                self.until(lambda:self.maps.get_subscription_count()>0 and
                           self.odometry.get_subscription_count()>=2, 'native subscriptions')
                cloud = make_map(known=True, wall=self.wall)
                cloud.header.stamp = self.get_clock().now().to_msg()
                self.maps.publish(cloud)
                self.until(lambda:self.status()['known_cells']>0, 'accepted map at PCI')
                self.until(lambda:self.status('sensor_gain').get('root') == [-2., 2., 0., 0.],
                           'native odometry acceptance')
                self.until(lambda:self.status('global_status')['vertices']>0, 'home from actual odometry')

            def start(self, automatic=True, bound=3):
                request = PciTrigger.Request()
                request.set_auto, request.not_exe_path, request.bound_mode = automatic, False, bound
                self.until(lambda:self.call('pci_trigger', request, PciTrigger).success, 'PCI task acceptance')

            def wait_terminal(self):
                state = {}
                def terminal():
                    state.update(self.status())
                    return state['state'] in {'DONE', 'STOPPED', 'ERROR', 'PREVIEW'}
                self.until(terminal, 'terminal controller state')
                return state

            def record(self, state):
                data = [item['data'] for item in self.metrics]
                times = [item['received_s'] for item in self.metrics]
                return {'state': state, 'local_metrics': data, 'global_metrics': list(self.globals),
                        'reply_intervals_s': [b-a for a, b in zip(times, times[1:])],
                        'action_requests': list(self.goals)}

            def close(self):
                if self.closed:
                    return
                self.closed = True
                self.timer.cancel()
                exits = []
                for process, log in reversed(self.processes):
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                        try:
                            process.wait(timeout=4.)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait(timeout=2.)
                    exits.append({'log': str(log), 'exit': process.returncode})
                summary['cases'].setdefault(self.label, {})['process_exits'] = exits
                self.server.destroy()
                self.destroy_node()

        blocked = Probe('blocked', wall=True, go_home=False)
        blocked.initialize()
        blocked.start(bound=0)
        blocked.until(lambda:len(blocked.metrics)>=6 and blocked.status()['state']=='WAITING_REPLAN',
                      'six throttled failed expansions')
        require(blocked.call('pci_stop', PciStop.Request(), PciStop).success, 'Stop was rejected')
        stopped = blocked.wait_terminal()
        count = len(blocked.metrics)
        blocked.spin_for(.75)
        require(len(blocked.metrics)==count and blocked.status()['state']=='STOPPED',
                'Cancelled retry restarted or left its stopped state')
        record = blocked.record(stopped)
        require(stopped['reason']=='user_stop' and not stopped['auto'] and stopped['cycles']==0,
                'Stop or execution-cycle accounting changed')
        require([m['bound_mode'] for m in record['local_metrics'][:6]] == [0, 1, 2, 0, 1, 2],
                'Automatic retry did not restart the requested padding sequence')
        require(all(m['reason']=='no_feasible_path' and not m['gain_evaluated']
                    and not m['auto_global']['counted'] for m in record['local_metrics']),
                'Blocked fixture unexpectedly performed a gain evaluation')
        require(sum(record['reply_intervals_s'][:5])>=2.45 and not blocked.globals and not blocked.goals,
                'Empty retries busy-looped or requested global/home/motion')
        summary['cases']['blocked'] = record
        # Single-shot requests report an empty route without changing task type.
        blocked.start(automatic=False)
        manual = blocked.wait_terminal()
        require(manual['state']=='ERROR' and manual['reason']=='local_path_empty' and not blocked.globals,
                'Single empty local task changed into a global or homing task')
        summary['cases']['blocked']['manual_empty'] = manual
        blocked.close()

        for go_home in (False, True):
            label = 'no_home' if not go_home else 'home_enabled'
            probe = Probe(label, wall=False, go_home=go_home)
            probe.initialize()
            probe.start()
            state = probe.wait_terminal()
            probe.until(lambda:len(probe.metrics)>=4 and len(probe.globals)>=1, 'four native evaluations')
            require(len(probe.metrics)==4 and len(probe.globals)==1,
                    'Native switch produced unexpected extra local or global calls')
            record = probe.record(state)
            metrics = record['local_metrics']
            require(all(m['gain_evaluated'] and not m['frontier_exists'] and m['gain']==0.
                        for m in metrics), 'Known-map fixture must evaluate zero gain')
            require([m['auto_global']['count_after'] for m in metrics]==[1, 2, 3, 0],
                    'PCI bypassed the RRG four-round frontier threshold')
            require([m['auto_global']['triggered'] for m in metrics]==[False, False, False, True],
                    'Native global switch happened before the fourth evaluation')
            require(sum(record['reply_intervals_s'])>=1.45,
                    'Empty local rounds did not retain the native 0.5 second delay')
            require([e['mode'] for e in state['events'] if e['reason'].endswith('_requested')]==['local']*4,
                    'PCI sent an extra global/home request instead of using the local response status')
            require(state['cycles']==0 and state['completed_actions']==0 and not probe.goals,
                    'An empty attempt consumed execution budget or sent an action')
            if go_home:
                require(state['state']=='DONE' and state['mode']=='homing' and state['reason']=='already_home'
                        and not state['auto'] and probe.globals[0]['mode']=='homing',
                        'Explicit K_HOMING was not handled as the full planner-owned home task')
            else:
                require(state['state']=='STOPPED' and state['reason']=='global_frontier_unavailable'
                        and probe.globals[0]['mode']=='frontier',
                        'Empty K_REPOSITIONING caused an unrequested home query')
            summary['cases'][label] = record
            if not go_home:
                request = PciGlobal.Request()
                request.bound_mode, request.not_exe_path, request.ignore_time = 3, True, True
                require(probe.call('pci_global', request, PciGlobal).success, 'Explicit global request rejected')
                direct = probe.wait_terminal()
                require(direct['state']=='ERROR' and direct['reason']=='explicit_global_path_empty',
                        'Empty explicit global request changed its destination')
                probe.until(lambda:len(probe.globals)==2, 'explicit global response evidence')
                require(all(m['mode']=='frontier' for m in probe.globals), 'Empty global requested home')
                home = PciHomingTrigger.Request()
                home.not_exe_path = True
                require(probe.call('pci_homing_trigger', home, PciHomingTrigger).success,
                        'Explicit home request rejected after the empty global task')
                explicit_home = probe.wait_terminal()
                require(explicit_home['state']=='DONE' and explicit_home['reason']=='already_home',
                        'Explicit home request no longer succeeds')
                record['explicit_global_empty'], record['explicit_home'] = direct, explicit_home
            probe.close()
        summary['checks'] = {'blocked_retries_restart_padding_with_delay': True,
            'stop_cancels_pending_retry': True, 'single_empty_local_does_not_change_task': True,
            'native_four_evaluations_reach_pci': True, 'default_no_home_policy_preserved': True,
            'native_homing_status_still_completes': True, 'explicit_global_empty_does_not_home': True,
            'explicit_home_still_works': True, 'empty_attempts_do_not_consume_execution_cycles': True,
            'no_action_execution': True}
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'], summary['traceback'] = str(error), traceback.format_exc()
    finally:
        for fixture in reversed(fixtures):
            fixture.close()
        if initialized:
            rclpy.shutdown()
        summary['elapsed_s'] = time.monotonic()-started
        output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'status': summary['status'], 'output': str(output),
                      'elapsed_s': summary['elapsed_s'], 'error': summary.get('error')}))
    return 0 if summary['status']=='passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

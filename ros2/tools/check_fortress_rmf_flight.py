#!/usr/bin/env python3
"""Supervised RMF takeoff, 3-D flight and landing in native Edgar/DART.

This is a controller/model integration run, not an autonomous planner result.
All observations come from native physics, joint and gpu_lidar sensors.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

from check_edgar_mapping import stamp_ns, stop_process_tree, cloud_metadata
from check_fortress_mapping import require


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--autonomy-cycles', type=int, default=0)
    args = parser.parse_args()
    if args.autonomy_cycles < 0:
        parser.error('--autonomy-cycles must be nonnegative')
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))
    started = time.monotonic()
    deadline = started + 180. + args.autonomy_cycles*120.
    result = {'status': 'failed', 'scope': 'Native DART RMF / original Lee / four rotor motor plugins',
              'autonomous': bool(args.autonomy_cycles), 'requested_local_cycles': args.autonomy_cycles,
              'supervised_takeoff_and_landing': True, 'endpoint_tolerance_m': .05, 'yaw_tolerance_rad': .1,
              'initialization': 'Spawn at z=.5 with zero motors; settle under native gravity',
              'phases': {}, 'checks': {}}
    history = {name: [] for name in ('odometry', 'joints', 'motor_commands', 'controller', 'scans',
                                    'trajectories', 'control', 'execution', 'map_updates', 'planning', 'paths', 'mission')}
    contact_counts = {}
    log_path = output.with_suffix('.log')
    metadata_path = output.with_name(output.stem + '_model.json')
    history_path = output.with_name(output.stem + '_history.json')
    process = node = log = None

    import rclpy
    from actuator_msgs.msg import Actuators
    from geometry_msgs.msg import Transform, Twist
    from nav_msgs.msg import Odometry, Path as RosPath
    from planner_msgs.srv import PciTrigger, PciSetHomingPos, ValidatePath
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from ros_gz_interfaces.msg import Contacts
    from sensor_msgs.msg import JointState, PointCloud2
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

    class Probe(Node):
        def __init__(self):
            super().__init__('fortress_rmf_flight_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.phase = 'startup'
            self.sim_s = 0.
            self.current = self.controller = None
            self.control, self.execution, self.map_update = {}, {}, {}
            self.accepted_cloud = None
            self.receipt = 0.
            self.publisher = self.create_publisher(MultiDOFJointTrajectory, 'command/trajectory', 10)
            self.disarm = self.create_client(Trigger, 'rmf/disarm')
            self.pci = self.create_client(PciTrigger, 'pci_trigger')
            self.home = self.create_client(PciSetHomingPos, 'pci_set_homing_pos')
            self.validator = self.create_client(ValidatePath, 'validate_path')
            self.state_clients = {name: self.create_client(Trigger, name) for name in
                             ('control_state', 'execution_state', 'mapper_status', 'map_status', 'global_status')}
            self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
            self.create_subscription(Odometry, 'odometry', self.odom, qos_profile_sensor_data)
            self.create_subscription(JointState, 'joint_states', self.joints, qos_profile_sensor_data)
            self.create_subscription(Actuators, 'rmf/command/motor_speed', self.motors, 10)
            self.create_subscription(String, 'rmf_controller_state', self.state, 10)
            self.create_subscription(PointCloud2, 'points', self.scan, qos_profile_sensor_data)
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            for topic, name in [('control_status', 'control'), ('execution_status', 'execution'),
                                ('map_updates', 'map_updates'), ('planning_metrics', 'planning'),
                                ('mission_status', 'mission')]:
                self.create_subscription(String, topic, lambda msg, n=name: self.status(msg, n), durable)
            self.create_subscription(RosPath, 'command_path', self.path, durable)
            self.create_subscription(PointCloud2, 'accepted_voxel_map',
                                     lambda msg: setattr(self, 'accepted_cloud', msg), durable)
            for part in ('base', 'rotor_0', 'rotor_1', 'rotor_2', 'rotor_3'):
                self.create_subscription(Contacts, '/rmf/contact/' + part,
                    lambda msg, p=part: self.contact(msg, p), qos_profile_sensor_data)

        def record(self, value):
            return {'wall_s': time.monotonic()-started, 'sim_s': self.sim_s, 'phase': self.phase, **value}

        def clock(self, msg):
            self.sim_s = msg.clock.sec + msg.clock.nanosec*1.e-9

        def position(self):
            p = self.current.pose.pose.position
            return [p.x, p.y, p.z]

        def yaw(self):
            q = self.current.pose.pose.orientation
            return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

        def odom(self, msg):
            self.current, self.receipt = msg, time.monotonic()
            q, v, w = msg.pose.pose.orientation, msg.twist.twist.linear, msg.twist.twist.angular
            history['odometry'].append(self.record({'stamp_ns': stamp_ns(msg), 'position': self.position(),
                'quaternion': [q.x, q.y, q.z, q.w], 'body_linear': [v.x, v.y, v.z],
                'body_angular': [w.x, w.y, w.z]}))

        def joints(self, msg):
            history['joints'].append(self.record({'stamp_ns': stamp_ns(msg), 'names': list(msg.name),
                'position': list(msg.position), 'velocity': list(msg.velocity)}))

        def motors(self, msg):
            history['motor_commands'].append(self.record({'stamp_ns': stamp_ns(msg), 'velocity': list(msg.velocity)}))

        def state(self, msg):
            self.controller = json.loads(msg.data)
            history['controller'].append(self.record(self.controller))

        def scan(self, msg):
            history['scans'].append(self.record(cloud_metadata(msg, 'points')))

        def contact(self, msg, part):
            item = contact_counts.setdefault(self.phase, {}).setdefault(part,
                {'messages': 0, 'pairs': 0, 'positions': 0, 'depth_samples': 0, 'wrench_samples': 0})
            item['messages'] += 1
            item['pairs'] += len(msg.contacts)
            item['positions'] += sum(len(c.positions) for c in msg.contacts)
            item['depth_samples'] += sum(len(c.depths) for c in msg.contacts)
            item['wrench_samples'] += sum(len(c.wrenches) for c in msg.contacts)
            if self.phase == 'automatic_exploration':
                require(not msg.contacts, 'Native contact observed during autonomous flight: '+part)

        def status(self, msg, name):
            value = json.loads(msg.data)
            history[name].append(self.record(value))
            if name in ('control', 'execution'):
                setattr(self, name, value)
            elif name == 'map_updates':
                self.map_update = value

        def path(self, msg):
            history['paths'].append(self.record({'path': [[p.pose.position.x, p.pose.position.y,
                p.pose.position.z] for p in msg.poses]}))

        def call(self, client, request):
            end = min(deadline, time.monotonic()+20.)
            while not client.service_is_ready():
                self.tick()
                require(time.monotonic() < end, 'Service discovery deadline')
            future = client.call_async(request)
            while not future.done():
                self.tick()
                require(time.monotonic() < end, 'Service reply deadline')
            return future.result()

        def snapshot(self):
            return {name: json.loads(self.call(client, Trigger.Request()).message)
                    for name,client in self.state_clients.items()}

        def explore(self):
            self.phase = 'preflight'
            self.observe_for('preflight', 1.)
            request = ValidatePath.Request()
            request.header.frame_id = 'map'
            request.header.stamp = self.get_clock().now().to_msg()
            request.bound_mode = 0
            request.path = [self.current.pose.pose]
            request.first_pose_is_observation = True
            checked = self.call(self.validator, request)
            result['root_check'] = {'valid': checked.valid, 'reason': checked.reason, 'revision': checked.map_revision}
            require(checked.valid, 'Observed TSDF rejects aerial root: '+checked.reason)
            require(self.call(self.home, PciSetHomingPos.Request()).success, 'PCI home initialization refused')
            result['mission_home'] = self.position()
            result['before_autonomy'] = self.snapshot()
            self.phase = 'automatic_exploration'
            trigger = PciTrigger.Request()
            trigger.set_auto, trigger.not_exe_path = True, False
            trigger.bound_mode, trigger.vel_max = 0, 0.
            require(self.call(self.pci, trigger).success, 'Automatic PCI trigger refused')
            result['autonomy_trigger_requests'] = 1
            last_print = 0.
            while not (self.control.get('state') == 'DONE' and self.control.get('reason') == 'returned_home'):
                self.tick()
                require(self.control.get('state') not in ('ERROR', 'STOPPED'), 'PCI stopped: '+json.dumps(self.control))
                if time.monotonic()-last_print > 10.:
                    print(json.dumps({'phase': self.phase, 'state': self.control.get('state'),
                        'reason': self.control.get('reason'), 'cycles': self.control.get('cycles'),
                        'position': self.position()}), flush=True)
                    last_print = time.monotonic()
            result['after_autonomy'] = self.snapshot()
            after = result['after_autonomy']['execution_state']
            before = result['before_autonomy']['execution_state']
            # PCI cycles includes the final homing action, and an acknowledged
            # PlanAhead splice counts a local cycle within an existing Action.
            completed = {e['completed_actions']: e['mode'] for e in history['control']
                         if e['phase'] == 'automatic_exploration' and e['state'] == 'WAITING_OBSERVATION'}
            continuations = self.control['plan_ahead']['continuations']
            local_actions = sum(mode == 'local' for mode in completed.values())
            homing_actions = sum(mode == 'homing' for mode in completed.values())
            result['autonomy_counts'] = {'local_actions': local_actions, 'homing_actions': homing_actions,
                'local_continuations': continuations, 'pci_cycles_including_home': self.control.get('cycles')}
            require(local_actions+continuations == args.autonomy_cycles and homing_actions == 1 and
                    self.control.get('cycles') == args.autonomy_cycles+1, 'Unexpected completed local/homing count')
            require(after['succeeded']-before['succeeded'] == local_actions+homing_actions and
                    after['aborted'] == before['aborted'] and after['cancelled'] == before['cancelled'],
                    'Aerial Action totals did not match local plus homing')
            require(after['output_type'] == 'kTopic' and after['action_commands'] == 0, 'Unexpected output backend')
            result['autonomous_home_error_m'] = math.dist(self.position(), result['mission_home'])
            require(result['autonomous_home_error_m'] <= .05, 'Physical homing endpoint outside tolerance')
            result['checks']['autonomous_local_and_home'] = True

        def tick(self):
            require(time.monotonic() < deadline, 'Flight wall deadline')
            require(process.poll() is None, 'Launch exited during flight')
            rclpy.spin_once(self, timeout_sec=.01)
            if self.current:
                p, q = self.position(), self.current.pose.pose.orientation
                require(all(math.isfinite(v) for v in p), 'Nonfinite native position')
                if self.phase != 'automatic_exploration':
                    require(p[2] > -.5 and abs(p[0]) < 5 and abs(p[1]) < 5 and p[2] < 4, 'Flight left supervised volume')
                require(1-2*(q.x*q.x+q.y*q.y) > .5, 'Native airframe tilt exceeded 60 degrees')

        def observe_for(self, phase, duration):
            self.phase = phase
            initial = self.sim_s
            while self.sim_s-initial < duration:
                self.tick()

        def move(self, phase, target, duration, yaw=0.):
            self.phase = phase
            first = self.position()
            initial_yaw = self.yaw()
            yaw_delta = math.atan2(math.sin(yaw-initial_yaw), math.cos(yaw-initial_yaw))
            message = MultiDOFJointTrajectory()
            message.header.frame_id = 'map'
            message.header.stamp = self.get_clock().now().to_msg()
            message.joint_names = ['base_link']
            count = round(duration/.05)
            for index in range(count+1):
                t = duration*index/count
                u = t/duration
                blend = 10*u**3-15*u**4+6*u**5
                speed = (30*u*u-60*u**3+30*u**4)/duration
                accel = (60*u-180*u*u+120*u**3)/(duration*duration)
                point = MultiDOFJointTrajectoryPoint()
                transform, velocity, acceleration = Transform(), Twist(), Twist()
                for axis, a, b in zip('xyz', first, target):
                    setattr(transform.translation, axis, a+(b-a)*blend)
                    setattr(velocity.linear, axis, (b-a)*speed)
                    setattr(acceleration.linear, axis, (b-a)*accel)
                angle = initial_yaw + yaw_delta*blend
                transform.rotation.z, transform.rotation.w = math.sin(angle/2), math.cos(angle/2)
                velocity.angular.z = yaw_delta*speed
                acceleration.angular.z = yaw_delta*accel
                point.transforms, point.velocities, point.accelerations = [transform], [velocity], [acceleration]
                offset = round(t*1.e9)
                point.time_from_start.sec, point.time_from_start.nanosec = divmod(offset, 1_000_000_000)
                message.points.append(point)
            before = self.controller['accepted_commands']
            self.publisher.publish(message)
            origin = stamp_ns(message)*1.e-9
            history['trajectories'].append(self.record({'origin_sim_s': origin, 'start': first, 'target': target,
                'start_yaw': initial_yaw, 'target_yaw': yaw, 'duration_s': duration,
                'samples': len(message.points), 'interpolation': 'quintic position/velocity/acceleration at .05 s'}))
            ack_deadline = time.monotonic()+5.
            while self.controller['accepted_commands'] == before:
                self.tick()
                require(time.monotonic() < ack_deadline, 'Trajectory not accepted: '+json.dumps(self.controller))
            # Preserve existing .05 m / .1 rad endpoint thresholds. Give the
            # original feedback controller the standard 10 s settling grace.
            while True:
                self.tick()
                error = math.dist(self.position(), target)
                yaw_error = abs(math.atan2(math.sin(self.yaw()-yaw), math.cos(self.yaw()-yaw)))
                v = self.current.twist.twist.linear
                if self.sim_s-origin >= duration and error <= .05 and yaw_error <= .1 and math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z) < .1:
                    break
                require(self.sim_s-origin < duration+10., phase+' did not reach endpoint')
            result['phases'][phase] = {'target': target, 'end_position': self.position(), 'position_error_m': error,
                'yaw_error_rad': yaw_error, 'elapsed_sim_s': self.sim_s-origin,
                'controller': self.controller}

    try:
        log = log_path.open('w')
        process = subprocess.Popen(['ros2', 'launch', 'gbplanner_gazebo', 'rmf_fortress.launch.py',
            'mapping:='+str(bool(args.autonomy_cycles)).lower(),
            'planning:='+str(bool(args.autonomy_cycles)).lower(),
            'max_cycles:='+str(max(1, args.autonomy_cycles)),
            'contacts:=true', f'model_metadata_file:={metadata_path}'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = Probe()
        while not (node.current and node.controller and history['joints'] and history['scans'] and
                   node.disarm.service_is_ready() and node.publisher.get_subscription_count()):
            node.tick()
        node.observe_for('gravity_settle', 1.5)
        ground = node.position()
        require(all(abs(v) < 1.e-9 for v in node.controller['motor_speed_rad_s']), 'Initial rotor commands are nonzero')
        require(ground[2] < .45, 'Did not naturally descend from spawn height')
        result['ground_position'] = ground
        result['checks']['zero_motor_natural_settle'] = True
        hover = [ground[0], ground[1], ground[2]+1.0]
        node.move('takeoff', hover, 4.)
        node.observe_for('hover', 1.5)
        node.move('flight_3d', [hover[0]+.7, hover[1]+.4, hover[2]+.3], 5., yaw=.3)
        node.move('return', hover, 5.)
        if args.autonomy_cycles:
            node.explore()
        node.move('landing', ground, 5.)
        future = node.disarm.call_async(Trigger.Request())
        while not future.done():
            node.tick()
        require(future.result().success, 'Disarm refused')
        node.observe_for('landed', 1.5)
        require(math.dist(node.position(), ground) <= .05, 'Landed position outside original endpoint tolerance')
        require(all(abs(v) < 1.e-9 for v in node.controller['motor_speed_rad_s']), 'Motors not zero after disarm')
        require(contact_counts.get('landed', {}).get('base', {}).get('pairs', 0) > 0, 'No native landing contact observed')
        moving_joints = [r for r in history['joints'] if r['phase'] == 'flight_3d' and len(r['velocity']) == 4]
        require(moving_joints and all(any(abs(r['velocity'][i]) > 1 for r in moving_joints) for i in range(4)),
                'Did not observe four rotating native rotor joints')
        require(all(r['frame_id'] == 'lidar' for r in history['scans']), 'Unexpected pointcloud frame')
        result['checks'].update(takeoff=True, flight_3d=True, return_to_hover=True, landed_with_contact=True,
                                zero_motors_after_landing=True, four_rotating_joints=True, native_lidar=True)
        result['status'] = 'passed'
    except Exception as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        if node is not None:
            result['last_controller'] = node.controller
            result['last_control'], result['last_execution'] = node.control, node.execution
            result['last_map_update'] = node.map_update
            result['last_position'] = node.position() if node.current else None
            if node.accepted_cloud is not None:
                cloud = node.accepted_cloud
                cloud_path = output.with_name(output.stem+'_accepted_map.bin')
                cloud_path.write_bytes(bytes(cloud.data))
                result['last_accepted_map'] = {**cloud_metadata(cloud, 'accepted_voxel_map'), 'data_file': str(cloud_path)}
            if rclpy.ok() and node.disarm.service_is_ready():
                future = node.disarm.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=.5)
        if process is not None:
            try:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=7.)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass
        result['cleanup'] = stop_process_tree(process)
        if log:
            log.close()
        log_text = log_path.read_text() if log_path.exists() else ''
        failures = re.findall(r'\[([^\]]+)\]: process has died \[pid \d+, exit code (-?\d+)', log_text)
        result['node_exit_failures'] = [{'name': n, 'exit_code': int(c)} for n,c in failures]
        result['unhandled_callback_errors'] = [line for line in log_text.splitlines()
                                              if 'exception was never retrieved' in line]
        result['checks']['clean_shutdown'] = (not failures and not result['unhandled_callback_errors']
                                              and not result['cleanup'].get('forced_kill', False))
        if not result['checks']['clean_shutdown']:
            result['status'] = 'failed'
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        result['wall_elapsed_s'] = time.monotonic()-started
        result['contact_observations_by_phase'] = contact_counts
        result['counts'] = {k: len(v) for k,v in history.items()}
        missions = history['mission']
        result['mission_delivery'] = {
            'messages': len(missions),
            'maximum_wall_gap_s': max((b['wall_s']-a['wall_s'] for a,b in zip(missions, missions[1:])), default=None),
            'source_start_s': missions[0].get('stamp_s') if missions else None,
            'source_end_s': missions[-1].get('stamp_s') if missions else None}
        result['history_file'], result['model_file'] = str(history_path), str(metadata_path)
        history_path.write_text(json.dumps(history, separators=(',', ':'))+'\n')
        root = Path(__file__).resolve().parents[1]
        result['source_sha256'] = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [root/'src/gbplanner_gazebo/src/rmf_controller_node.cpp',
                      root/'src/gbplanner_gazebo/scripts/fortress_rmf_model.py',
                      root/'src/gbplanner_gazebo/scripts/fortress_state_node.py',
                      root/'src/gbplanner_gazebo/launch/rmf_fortress.launch.py', Path(__file__).resolve()]}
        output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        print(json.dumps({'status': result['status'], 'output': str(output),
                          'phases': list(result['phases']), 'checks': result['checks'],
                          'error': result.get('error', '')[:500]}, indent=2))
    return 0 if result['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

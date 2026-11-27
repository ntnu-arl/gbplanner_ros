#!/usr/bin/env python3
"""Run a short native Fortress wheel / 3-D lidar / TSDF integration experiment.

This is a supervised velocity experiment, not an autonomous planning result.
Gazebo 6.18.0 odometry semantics were checked against its source. The run still
needs a model-origin / native-world-pose comparison. Angular twist conversion
is checked independently with the quaternion derivative of ZYX Euler angles.
"""
import argparse
from collections import deque
import hashlib
import json
import math
import os
import re
from pathlib import Path
import signal
import subprocess
import time

from check_edgar_mapping import cloud_metadata, field_values, stamp_ns, stop_process_tree


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def pose_record(message):
    p, q = message.pose.pose.position, message.pose.pose.orientation
    return [p.x, p.y, p.z,
            math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))]


def angular_from_euler_rates(quaternion, rates):
    """Independent numerical quaternion derivative, not the adapter's matrix."""
    x, y, z, w = quaternion
    angles = [math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
              math.asin(max(-1., min(1., 2*(w*y-z*x)))),
              math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))]

    def orientation(a):
        cr, cp, cy = [math.cos(v/2) for v in a]
        sr, sp, sy = [math.sin(v/2) for v in a]
        return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]

    epsilon = 1.e-6
    plus = orientation([a+epsilon*r for a, r in zip(angles, rates)])
    minus = orientation([a-epsilon*r for a, r in zip(angles, rates)])
    dx, dy, dz, dw = [(p-m)/(2*epsilon) for p, m in zip(plus, minus)]
    x, y, z, w = orientation(angles)
    # Body angular velocity is twice the vector part of conjugate(q) * q_dot.
    return [2*(w*dx-x*dw-y*dz+z*dy),
            2*(w*dy+x*dz-y*dw-z*dx),
            2*(w*dz-x*dy+y*dx-z*dw)]


def describe_cloud(message, topic):
    result = cloud_metadata(message, topic)
    xyz = list(zip(*(field_values(message, field) for field in ('x', 'y', 'z'))))
    finite = [point for point in xyz if all(math.isfinite(value) for value in point)]
    result.update(finite_points=len(finite), nonfinite_points=len(xyz)-len(finite),
                  sha256=hashlib.sha256(message.data).hexdigest())
    if finite:
        result['xyz_min'] = [min(p[i] for p in finite) for i in range(3)]
        result['xyz_max'] = [max(p[i] for p in finite) for i in range(3)]
        result['z_span_m'] = result['xyz_max'][2]-result['xyz_min'][2]
        result['minimum_return_range_m'] = min(math.sqrt(sum(v*v for v in p)) for p in finite)
        result['sample_xyz'] = finite[::max(1, len(finite)//12)][:12]
    return result


def compose_native_pose(model, link):
    """SceneBroadcaster model is world-relative; its link is model-relative."""
    def vector(value):
        return [float(value.get(k, 0.)) for k in ('x', 'y', 'z')]
    def quaternion(value):
        return vector(value)+[float(value.get('w', 0.))]
    a, b = quaternion(model['orientation']), quaternion(link['orientation'])
    t = vector(link['position'])
    u = [2*(a[1]*t[2]-a[2]*t[1]), 2*(a[2]*t[0]-a[0]*t[2]), 2*(a[0]*t[1]-a[1]*t[0])]
    cross = [a[1]*u[2]-a[2]*u[1], a[2]*u[0]-a[0]*u[2], a[0]*u[1]-a[1]*u[0]]
    position = [v+t[i]+a[3]*u[i]+cross[i] for i, v in enumerate(vector(model['position']))]
    q = [a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],
         a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
         a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],
         a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]]
    return position, q


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--wall-budget', type=float, default=180.)
    parser.add_argument('--contacts', action='store_true', help='Observe native chassis and four-wheel contacts')
    args = parser.parse_args()
    if not math.isfinite(args.wall_budget) or args.wall_budget < 30:
        parser.error('--wall-budget must be finite and at least 30 seconds')
    args.output = args.output.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))
    started = time.monotonic()
    deadline = started + args.wall_budget - 10.
    summary = {
        'status': 'failed', 'domain': os.environ['ROS_DOMAIN_ID'],
        'wall_budget_s': args.wall_budget, 'checks': {},
        'scope': 'Native Gazebo Fortress Edgar mesh, wheel joints, gpu_lidar and Voxblox TSDF',
        'motion_requested': True, 'autonomous_planning_requested': False,
        'action_or_pci_requests': 0,
        'odometry_source_semantics_checked': True,
        'odometry_source_version': 'Gazebo Fortress 6.18.0',
        'odometry_source_evidence': 'results/fortress_setup/OdometryPublisher.cc',
        'odometry_world_pose_runtime_verified': False,
        'odometry_world_semantics_note': 'Source uses worldPose(model) with zero configured offset, '
            'body-frame linear velocity, and filtered Euler-angle derivatives. The adapter '
            'converts those derivatives to body angular velocity. Native world pose, '
            'model/base origin and actual TF still require runtime comparison.',
        'contact_diagnostics_available': args.contacts,
        'command_targets': {'forward_m': .4, 'turn_rad': .3},
    }
    process = node = log = None
    log_path = args.output.with_suffix('.log')
    history_path = args.output.with_name(args.output.stem + '_history.json')

    import rclpy
    from controller_manager_msgs.srv import ListControllers
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState, PointCloud2
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener

    class Probe(Node):
        def __init__(self):
            super().__init__('fortress_mapping_probe', parameter_overrides=[
                Parameter('use_sim_time', value=True)])
            self.phase = 'startup'
            self.clock_ns = 0
            self.clock_count = 0
            self.current = self.raw_current = None
            self.clouds = {}
            self.cloud_counts = {}
            self.cloud_stamps = {}
            self.cloud_hashes = {}
            self.odom_counts = {'odometry': 0, 'gazebo/odometry_raw': 0}
            self.odom_samples = {'odometry': {}, 'gazebo/odometry_raw': {}}
            self.joints = None
            self.joint_count = 0
            self.trace = []
            self.raw_trace = []
            self.commands = []
            self.joint_history = []
            self.map_history = []
            self.states = []
            self.recent = deque(maxlen=300)
            self.linear = self.angular = 0.
            self.last_command_stamp = -1
            self.last_command_wall = 0.
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.command_pub = self.create_publisher(Twist, 'cmd_vel', 10)
            self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
            self.create_subscription(Odometry, 'odometry',
                lambda m: self.odom(m, 'odometry'), qos_profile_sensor_data)
            self.create_subscription(Odometry, 'gazebo/odometry_raw',
                lambda m: self.odom(m, 'gazebo/odometry_raw'), qos_profile_sensor_data)
            for topic in ('points', 'gazebo/points_raw'):
                self.create_subscription(PointCloud2, topic,
                    lambda m, t=topic: self.cloud(m, t), qos_profile_sensor_data)
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(PointCloud2, 'voxel_map',
                lambda m: self.cloud(m, 'voxel_map'), durable)
            self.create_subscription(JointState, 'joint_states', self.joint, qos_profile_sensor_data)
            self.create_subscription(String, 'gazebo_state',
                lambda m: self.states.append(self.timed(json.loads(m.data))), 10)
            self.mapper = self.create_client(Trigger, 'mapper_status')
            self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')

        def timed(self, value):
            return {'wall_s': time.monotonic()-started, 'sim_s': self.clock_ns*1.e-9,
                    'phase': self.phase, **value}

        def clock(self, message):
            value = message.clock.sec*1_000_000_000 + message.clock.nanosec
            require(value >= self.clock_ns, 'Native simulation clock moved backwards')
            self.clock_ns, self.clock_count = value, self.clock_count+1

        def odom(self, message, topic):
            pose = pose_record(message)
            q = message.pose.pose.orientation
            twist = message.twist.twist
            values = pose + [q.x, q.y, q.z, q.w, twist.linear.x, twist.linear.y,
                             twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z]
            require(all(math.isfinite(v) for v in values), f'Non-finite {topic}')
            row = self.timed({'stamp_ns': stamp_ns(message), 'pose': pose,
                'frame_id': message.header.frame_id, 'child_frame_id': message.child_frame_id,
                'quaternion': [q.x, q.y, q.z, q.w],
                'linear_velocity': [twist.linear.x, twist.linear.y, twist.linear.z],
                'angular_velocity': [twist.angular.x, twist.angular.y, twist.angular.z]})
            self.odom_counts[topic] += 1
            self.odom_samples[topic][stamp_ns(message)] = values
            if len(self.odom_samples[topic]) > 500:
                self.odom_samples[topic].pop(next(iter(self.odom_samples[topic])))
            if topic == 'odometry':
                self.current = message
                self.trace.append(row)
                self.recent.append(row)
            else:
                self.raw_current = message
                self.raw_trace.append(row)

        def cloud(self, message, topic):
            self.clouds[topic] = message
            self.cloud_counts[topic] = self.cloud_counts.get(topic, 0)+1
            self.cloud_stamps.setdefault(topic, set()).add(stamp_ns(message))
            if topic != 'voxel_map':
                values = self.cloud_hashes.setdefault(topic, {})
                values[stamp_ns(message)] = hashlib.sha256(message.data).hexdigest()
            else:
                self.map_history.append(self.timed({'stamp_ns': stamp_ns(message),
                    'cells': message.width*message.height, 'frame_id': message.header.frame_id}))

        def joint(self, message):
            require(all(math.isfinite(v) for v in list(message.position)+list(message.velocity)),
                    'Non-finite wheel joint state')
            self.joints, self.joint_count = message, self.joint_count+1
            self.joint_history.append(self.timed({'stamp_ns': stamp_ns(message),
                'name': list(message.name), 'position': list(message.position),
                'velocity': list(message.velocity)}))

        def command(self, force=False):
            now = time.monotonic()
            if not force and self.clock_ns-self.last_command_stamp < 50_000_000 and now-self.last_command_wall < .1:
                return
            message = Twist()
            message.linear.x, message.angular.z = self.linear, self.angular
            self.command_pub.publish(message)
            self.commands.append(self.timed({'linear_x': self.linear, 'angular_z': self.angular}))
            self.last_command_stamp, self.last_command_wall = self.clock_ns, now

        def until(self, predicate, label):
            while not predicate():
                if process is not None and process.poll() is not None:
                    raise RuntimeError(f'Launch exited {process.returncode}: {log_path}')
                if time.monotonic() >= deadline:
                    raise TimeoutError(f'Wall budget exhausted during {label}')
                self.command()
                rclpy.spin_once(self, timeout_sec=.01)

        def stable(self, seconds=1.):
            if not self.recent:
                return False
            end = self.recent[-1]['stamp_ns']
            rows = [r for r in self.recent if end-r['stamp_ns'] <= seconds*1.e9]
            return (len(rows) >= 5 and end-rows[0]['stamp_ns'] >= (seconds-.1)*1.e9
                and max(abs(r['pose'][2]-rows[-1]['pose'][2]) for r in rows) < .01
                and max(math.dist(r['pose'][:2], rows[-1]['pose'][:2]) for r in rows) < .025
                and max(abs(wrap(r['pose'][3]-rows[-1]['pose'][3])) for r in rows) < .04
                and max(math.sqrt(sum(v*v for v in r['linear_velocity'])) for r in rows) < .04
                and max(abs(r['angular_velocity'][2]) for r in rows) < .05)

        def read_services(self):
            self.until(lambda: self.mapper.service_is_ready() and self.controllers.service_is_ready(),
                       'mapper and controller services')
            futures = {'mapper': self.mapper.call_async(Trigger.Request()),
                       'controllers': self.controllers.call_async(ListControllers.Request())}
            self.until(lambda: all(f.done() for f in futures.values()), 'service responses')
            response = futures['mapper'].result()
            require(response.success, response.message)
            return {'mapper': json.loads(response.message), 'controllers': [
                {'name': c.name, 'state': c.state, 'type': c.type,
                 'claimed_interfaces': list(c.claimed_interfaces)}
                for c in futures['controllers'].result().controller]}

        def transform(self, target, source):
            self.until(lambda: self.tf_buffer.can_transform(target, source, Time()), f'TF {target} <- {source}')
            value = self.tf_buffer.lookup_transform(target, source, Time())
            t, q = value.transform.translation, value.transform.rotation
            return {'target': target, 'source': source, 'stamp_ns': stamp_ns(value),
                    'translation': [t.x, t.y, t.z], 'quaternion': [q.x, q.y, q.z, q.w]}

    try:
        rclpy.init(args=[])
        node = Probe()
        log = log_path.open('w')
        command = ['ros2', 'launch', 'gbplanner_gazebo', 'edgar_fortress.launch.py',
                   'mapping:=true', 'planning:=false', 'contacts:='+str(args.contacts).lower()]
        summary['launch_command'] = command
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=dict(os.environ))
        node.until(lambda: node.clock_count >= 5 and node.current is not None and
            node.raw_current is not None and node.joints is not None and
            all(node.cloud_counts.get(t, 0) >= 2 for t in ('points', 'gazebo/points_raw', 'voxel_map')),
            'native clock, wheel joints, odometry, 3-D scan and initial TSDF')
        node.phase = 'settling'
        node.until(node.stable, 'stable physical pose')
        summary['initial'] = node.read_services()
        active = {c['name'] for c in summary['initial']['controllers'] if c['state'] == 'active'}
        require({'joint_state_broadcaster', 'smb_drive'} <= active,
                f'Controllers are not active: {summary["initial"]["controllers"]}')
        wheels = {'LF_wheel_joint', 'RF_wheel_joint', 'LH_wheel_joint', 'RH_wheel_joint'}
        require(wheels <= set(node.joints.name), 'Missing physical wheel joint states')
        summary['initial_pose'] = origin = pose_record(node.current)
        summary['initial_sim_s'] = initial_sim = node.clock_ns*1.e-9
        summary['initial_cloud'] = describe_cloud(node.clouds['points'], 'points')
        summary['initial_map_cells'] = node.clouds['voxel_map'].width*node.clouds['voxel_map'].height
        summary['initial_tf'] = [node.transform('map', 'base_link'), node.transform('base_link', 'smb_lidar')]
        # Read the independent native scene transform while continuing ROS
        # callbacks. Do not bridge relative link poses into /tf as world poses.
        snapshot_path = args.output.with_name(args.output.stem+'_native_poses.json')
        native_env = dict(os.environ, IGN_PARTITION='gbplanner_fortress_'+os.environ['ROS_DOMAIN_ID'])
        with snapshot_path.open('w') as snapshot:
            observer = subprocess.Popen(['ign', 'topic', '-e', '-t', '/world/edgar/dynamic_pose/info',
                                         '-n', '1', '--json-output'], stdout=snapshot,
                                        stderr=subprocess.DEVNULL, env=native_env)
            try:
                native_deadline = time.monotonic()+8.
                node.until(lambda: observer.poll() is not None or time.monotonic() > native_deadline,
                           'Independent native scene pose')
                require(observer.poll() == 0, 'Native scene pose command did not complete')
            finally:
                if observer.poll() is None:
                    observer.terminate()
                    observer.wait(timeout=2.)
        native = json.loads(snapshot_path.read_text())
        model_pose = next(p for p in native['pose'] if p.get('name') == 'smb')
        base_pose = next(p for p in native['pose'] if p.get('name', '').split('::')[-1] == 'base_link')
        position, orientation = compose_native_pose(model_pose, base_pose)
        stamp = native['header']['stamp']
        native_ns = int(stamp.get('sec', 0))*1_000_000_000+int(stamp.get('nsec', 0))
        node.until(lambda: node.raw_trace and node.raw_trace[-1]['stamp_ns'] >= native_ns,
                   'Odometry bracketing native scene snapshot')
        measured = min(node.raw_trace, key=lambda row: abs(row['stamp_ns']-native_ns))
        separation = abs(measured['stamp_ns']-native_ns)*1.e-9
        position_error = math.dist(position, measured['pose'][:3])
        quaternion_error = min(math.dist(orientation, measured['quaternion']),
                               math.dist(orientation, [-x for x in measured['quaternion']]))
        require(separation <= .03 and position_error < 1.e-4 and quaternion_error < 1.e-4,
                'Native scene and odometry disagree at stable physical rest')
        summary['native_world_pose_audit'] = {'source': str(snapshot_path),
            'model': model_pose, 'base_relative_to_model': base_pose,
            'composed_world_base_position': position, 'composed_world_base_quaternion': orientation,
            'native_stamp_ns': native_ns, 'odometry_stamp_ns': measured['stamp_ns'],
            'stamp_separation_s': separation, 'position_error_m': position_error,
            'quaternion_component_distance': quaternion_error,
            'comparison_at_rest': True}
        summary['odometry_world_pose_runtime_verified'] = True
        summary['checks']['native_world_model_base_pose'] = True
        require(summary['initial_cloud']['finite_points'] > 100, 'Too few finite lidar points')
        require(summary['initial_cloud'].get('z_span_m', 0.) > .1, 'Native cloud has no observed vertical extent')
        summary['checks'].update(native_inputs=True, controllers_active=True, four_wheel_states=True,
                                 initial_tsdf=True, nonplanar_lidar=True, initial_tf=True)

        node.phase, node.linear = 'forward', .12
        movement_start = node.clock_ns
        def forward_done():
            pose = pose_record(node.current)
            distance = math.dist(pose[:2], origin[:2])
            require(node.clock_ns-movement_start < 8.e9, 'Forward motion exceeded eight simulated seconds')
            require(abs(pose[2]-origin[2]) < .25, 'Large height change during short forward command')
            require(abs(wrap(pose[3]-origin[3])) < .6, 'Large unexpected yaw during forward command')
            return distance >= .4
        node.until(forward_done, '0.4 m forward travel')
        node.linear = 0.
        node.command(force=True)
        node.phase = 'forward_stop'
        node.until(node.stable, 'settling after forward travel')
        summary['forward_endpoint'] = turn_origin = pose_record(node.current)
        summary['forward_displacement_m'] = math.dist(turn_origin[:2], origin[:2])

        node.phase, node.angular = 'turn', .18
        turn_start = node.clock_ns
        def turn_done():
            pose = pose_record(node.current)
            delta = wrap(pose[3]-turn_origin[3])
            require(node.clock_ns-turn_start < 8.e9, 'Turn exceeded eight simulated seconds')
            require(math.dist(pose[:2], turn_origin[:2]) < .3, 'Excessive translation during turn')
            require(delta > -.2, 'Turn moved opposite the commanded direction')
            return delta >= .3
        node.until(turn_done, '0.3 rad physical yaw change')
        node.angular = 0.
        node.command(force=True)
        node.phase = 'final_stop'
        node.until(node.stable, 'final physical stop')
        summary['final'] = node.read_services()
        summary['final_pose'] = final_pose = pose_record(node.current)
        summary['turn_yaw_rad'] = wrap(final_pose[3]-turn_origin[3])
        summary['turn_translation_m'] = math.dist(final_pose[:2], turn_origin[:2])
        summary['final_tf'] = [node.transform('map', 'base_link'), node.transform('base_link', 'smb_lidar')]
        if args.contacts:
            node.until(lambda: node.states and all(
                node.states[-1].get('contact_diagnostics', {}).get('parts', {}).get(part, {}).get('messages', 0) > 0
                for part in ('lf', 'rf', 'lh', 'rh')), 'All four native wheel contact sensors')
            summary['contact_observations'] = node.states[-1]['contact_diagnostics']
            summary['checks']['four_wheel_contact_observations'] = True
        summary['observed_motion_sim_s'] = node.clock_ns*1.e-9-initial_sim
        summary['final_cloud'] = describe_cloud(node.clouds['points'], 'points')
        summary['final_map_cells'] = node.clouds['voxel_map'].width*node.clouds['voxel_map'].height
        summary['known_cell_change'] = summary['final']['mapper']['known_cells']-summary['initial']['mapper']['known_cells']
        moving_wheels = {name for row in node.joint_history if row['phase'] == 'forward'
                         for name, speed in zip(row['name'], row['velocity']) if abs(speed) > .05}
        require(wheels <= moving_wheels, 'Not all four physical wheel joints rotated during forward travel')
        require(summary['final']['mapper']['integrated_scans'] > summary['initial']['mapper']['integrated_scans'],
                'TSDF did not integrate scans during motion')
        require(summary['final']['mapper']['known_cells'] > 0, 'TSDF is empty')
        require(summary['final']['mapper']['freespace']['unknown_box_operations'] == 0,
                'Unexpected unknown-space clearing')
        common_cloud = node.cloud_stamps['points'] & node.cloud_stamps['gazebo/points_raw']
        require(len(common_cloud) >= 3, 'Insufficient shared native/adapted acquisition stamps')
        require(all(node.cloud_hashes['points'][s] == node.cloud_hashes['gazebo/points_raw'][s]
                    for s in common_cloud), 'ROS adapter changed native point bytes')
        common_odom = set(node.odom_samples['odometry']) & set(node.odom_samples['gazebo/odometry_raw'])
        require(len(common_odom) >= 3, 'Insufficient shared raw/adapted odometry stamps')
        require(all(node.odom_samples['odometry'][s][:11] == node.odom_samples['gazebo/odometry_raw'][s][:11]
                    for s in common_odom), 'ROS adapter changed source pose, quaternion or linear velocity')
        angular_errors = []
        for stamp in common_odom:
            raw = node.odom_samples['gazebo/odometry_raw'][stamp]
            adapted = node.odom_samples['odometry'][stamp]
            expected = angular_from_euler_rates(raw[4:8], raw[11:14])
            angular_errors.append(max(abs(a-b) for a, b in zip(adapted[11:14], expected)))
        require(max(angular_errors) < 1.e-7, 'Euler-rate to body angular velocity conversion disagrees with quaternion derivative')
        summary['native_point_preservation_samples'] = len(common_cloud)
        summary['native_pose_quaternion_linear_preservation_samples'] = len(common_odom)
        summary['angular_conversion'] = {'method': 'central quaternion derivative of ZYX angles',
            'samples': len(common_odom), 'max_absolute_error_rad_s': max(angular_errors),
            'epsilon_s': 1.e-6, 'tolerance_rad_s': 1.e-7}
        summary['scan_map_matching_stamps'] = len(node.cloud_stamps['points'] & node.cloud_stamps['voxel_map'])
        require(summary['scan_map_matching_stamps'] >= 3, 'Scan acquisition timestamps not retained in TSDF output')
        summary['checks'].update(forward_travel=True, physical_turn=True, final_stop=True,
            four_wheels_rotated=True,
            scan_integration_during_motion=True, no_unknown_space_clearing=True,
            native_point_bytes_preserved=True, native_pose_quaternion_linear_preserved=True,
            angular_body_conversion=True,
            scan_map_timestamps=True)
        summary['status'] = 'passed'
    except Exception as exc:
        summary['error'] = f'{type(exc).__name__}: {exc}'
        import traceback
        summary['traceback'] = traceback.format_exc()
    finally:
        if node is not None:
            node.linear = node.angular = 0.
            if rclpy.ok():
                node.command(force=True)
            summary['counts'] = {'clock': node.clock_count, 'odometry': node.odom_counts,
                'clouds': node.cloud_counts, 'joint_states': node.joint_count}
            summary['last_sim_s'] = node.clock_ns*1.e-9
            summary['last_phase'] = node.phase
            summary['last_pose'] = pose_record(node.current) if node.current else None
            summary['latest_clouds'] = {topic: cloud_metadata(cloud, topic)
                                       for topic, cloud in node.clouds.items()}
            history_path.write_text(json.dumps({'trajectory': node.trace,
                'raw_odometry': node.raw_trace, 'commands': node.commands,
                'joint_states': node.joint_history, 'map_publications': node.map_history,
                'gazebo_states': node.states}, indent=2) + '\n')
            summary['history_file'] = str(history_path)
        # Renderer shutdown can take longer than the generic helper's short
        # grace period. Give the native launch a bounded normal SIGINT first.
        if process is not None:
            try:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=7.)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass
        summary['cleanup'] = stop_process_tree(process)
        if log:
            log.close()
        failures = re.findall(r'\[([^\]]+)\]: process has died \[pid \d+, exit code (-?\d+)',
                              log_path.read_text() if log_path.exists() else '')
        summary['node_exit_failures'] = [{'name': name, 'exit_code': int(code)}
                                         for name, code in failures]
        summary['checks']['clean_shutdown'] = not failures and not summary['cleanup'].get('forced_kill', False)
        if not summary['checks']['clean_shutdown']:
            summary['status'] = 'failed'
            summary.setdefault('error', 'Native process did not exit cleanly')
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        summary['wall_elapsed_s'] = time.monotonic()-started
        summary['log_file'] = str(log_path)
        args.output.write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
        print(json.dumps({'status': summary['status'], 'output': str(args.output),
                          'error': summary.get('error')}, ensure_ascii=False))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""One short native PCI search on real Edgar lidar/TSDF and wheel physics.

By default the probe attempts no manual initialization: an unknown starting
body/support is reported as blocked. --bootstrap explicitly enables a separate
supervised, physical +2 m/-1.5 m mapping sweep after an offline original-mesh
corridor audit. Mesh geometry is never supplied to the mapper or planner.
"""
import argparse
from collections import deque
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time

from check_edgar_mapping import cloud_metadata, stamp_ns, stop_process_tree


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def pose_record(pose):
    p, q = pose.position, pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    return [p.x, p.y, p.z, yaw]


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class UnobservedStart(RuntimeError):
    pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--bootstrap', action='store_true',
                        help='Perform the separately recorded, mesh-audited physical mapping sweep')
    parser.add_argument('--auto-cycles', type=int,
                        help='After the short search, continue one native automatic mission and return home')
    parser.add_argument('--auto-action-budget', type=float, default=120.,
                        help='Observer wall budget per automatic stage; does not change ROS timeouts')
    args = parser.parse_args()
    if args.auto_cycles is not None and args.auto_cycles < 1:
        parser.error('--auto-cycles must be positive')
    if not math.isfinite(args.auto_action_budget) or args.auto_action_budget <= 0.:
        parser.error('--auto-action-budget must be finite and positive')
    args.output = args.output.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    source = Path(__file__).resolve().parents[1]
    mesh = source/'src/planner_gazebo_sim/collision/edgar_mine.triangles'
    vehicle_source = source/'src/gbplanner_smb/config/edgar_mine.yaml'
    exploration_source = source/'src/gbplanner_smb/config/edgar_exploration.yaml'
    tracking_source = source/'src/gbplanner_tracking/config/smb.yaml'
    started = time.monotonic()
    wall_budget = 120.+(args.auto_cycles+1)*args.auto_action_budget if args.auto_cycles else 120.
    deadline = started+wall_budget-6.  # Six seconds reserved for bounded cleanup.
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    summary = {
        'status': 'failed', 'domain': os.environ['ROS_DOMAIN_ID'], 'wall_budget_s': wall_budget,
        'scope': 'Original Edgar mesh → native lidar → Voxblox TSDF → accepted map → '
                 'native PCI search → public and downstream PathFollower Actions → wheel dynamics',
        'bootstrap_requested': args.bootstrap, 'autonomous_requests': 0,
        'initialization_counts_as_autonomy': False,
        'target_forward_m': .35, 'checks': {},
    }
    process = node = log = None
    log_path = args.output.with_suffix('.log')
    history_path = args.output.with_name(args.output.stem+'_history.json')

    # Imports stay inside main so --help and syntax review require no ROS.
    import yaml
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry, Path as RosPath
    from planner_msgs.msg import Graph
    from planner_msgs.srv import PciSearch, PciTrigger, ValidatePath
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from visualization_msgs.msg import Marker, MarkerArray

    class Probe(Node):
        def __init__(self):
            super().__init__('edgar_motion_probe')
            self.phase = 'settling'
            self.current = None
            self.odom_received = 0.
            self.odom_stamp = 0
            self.speed = self.yaw_rate = math.inf
            self.smb, self.control, self.execution, self.tracking, self.update = {}, {}, {}, {}, {}
            self.trace, self.commands, self.states, self.events, self.map_updates = [], [], [], [], []
            self.paths, self.metrics = [], []
            self.local_metrics, self.global_metrics = [], []
            self.planning_visualization = []
            self.visualization_stats = {topic: {
                'messages': 0, 'nonempty_messages': 0, 'points': 0,
                'invalid_messages': 0, 'namespace_points': {},
            } for topic in ('vis/sampler', 'vis/planning_projected_graph', 'vis/planning_failed')}
            self.global_graph = None
            self.path_message = None
            self.raw_map = self.accepted_map = None
            self.cloud_stamps, self.raw_stamps, self.accepted_stamps = set(), set(), set()
            self.accepted_snapshots = deque(maxlen=8)
            self.last_command = None
            self.failure_metric = self.failure_map = None
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Odometry, 'odometry', self.odometry, qos_profile_sensor_data)
            self.create_subscription(PointCloud2, 'points', self.cloud, qos_profile_sensor_data)
            self.create_subscription(PointCloud2, 'voxel_map', self.raw, durable)
            self.create_subscription(PointCloud2, 'accepted_voxel_map', self.accepted, durable)
            self.create_subscription(String, 'map_updates', self.mapping, durable)
            self.create_subscription(String, 'smb_status', self.vehicle, 10)
            self.create_subscription(Twist, 'cmd_vel', self.velocity, 10)
            self.create_subscription(RosPath, 'command_path', self.command_path, durable)
            self.create_subscription(String, 'search_metrics', self.metric, durable)
            # Subscriptions opt into the planner's existing diagnostics. These
            # snapshots are observed only; none is supplied back as map/path input.
            for topic in self.visualization_stats:
                self.create_subscription(MarkerArray, topic,
                    lambda message, name=topic: self.planning_markers(message, name), durable)
            if args.auto_cycles:
                self.create_subscription(Graph, 'global_graph', self.graph, durable)
                for topic, target in [('planning_metrics', self.local_metrics),
                                      ('global_metrics', self.global_metrics)]:
                    self.create_subscription(String, topic,
                        lambda m, values=target: self.metric(m, values), durable)
            for topic, attr in [('control_status', 'control'), ('execution_status', 'execution'),
                                ('tracking_status', 'tracking')]:
                self.create_subscription(String, topic,
                    lambda message, name=attr: self.status(message, name), durable)
            self.teleop = self.create_publisher(Twist, 'joy_teleop/cmd_vel', 10)
            self.target = self.create_publisher(PoseStamped, 'search_target', durable)
            self.pci = self.create_client(PciSearch, 'pci_search')
            self.local_trigger = self.create_client(PciTrigger, 'pci_trigger')
            self.validator = self.create_client(ValidatePath, 'validate_path')
            self.state_clients = {name: self.create_client(Trigger, name) for name in
                ('smb_state', 'mapper_status', 'map_status', 'control_state', 'execution_state',
                 'tracking_state', 'velocity_mux_state')}

        def timed(self, value):
            return {'time_s': time.monotonic()-started, 'phase': self.phase, **value}

        def odometry(self, message):
            require(message.header.frame_id == 'map' and message.child_frame_id == 'base_link',
                    'Unexpected native odometry frames')
            self.current = copy.deepcopy(message.pose.pose)
            received, callback_ros_ns = time.monotonic(), self.get_clock().now().nanoseconds
            receipt_gap = received-self.odom_received if self.odom_received else None
            self.odom_received, self.odom_stamp = received, stamp_ns(message)
            self.speed = math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)
            self.yaw_rate = message.twist.twist.angular.z
            values = pose_record(self.current)
            require(all(math.isfinite(x) for x in values+[self.speed, self.yaw_rate]),
                    'Non-finite native odometry')
            self.trace.append(self.timed({'pose': values, 'stamp_ns': self.odom_stamp,
                                          'speed_mps': self.speed, 'yaw_rate': self.yaw_rate,
                                          'callback_ros_ns': callback_ros_ns,
                                          'source_age_s': (callback_ros_ns-self.odom_stamp)*1.e-9,
                                          'receipt_gap_s': receipt_gap}))

        def cloud(self, message):
            require(message.header.frame_id == 'smb_lidar', 'Unexpected native lidar frame')
            self.cloud_stamps.add(stamp_ns(message))

        def raw(self, message):
            require(message.header.frame_id == 'map', 'Unexpected TSDF frame')
            self.raw_map = message
            self.raw_stamps.add(stamp_ns(message))

        def accepted(self, message):
            require(message.header.frame_id == 'map', 'Unexpected accepted-map frame')
            self.accepted_map = message
            self.accepted_stamps.add(stamp_ns(message))
            self.accepted_snapshots.append(message)
            self.pin_failure()

        def mapping(self, message):
            self.update = json.loads(message.data)
            self.map_updates.append(self.timed(self.update))
            self.pin_failure()

        def status(self, message, name):
            value = json.loads(message.data)
            setattr(self, name, value)
            self.events.append(self.timed({'layer': name, **value}))
            failure = value.get('validation_failure') if name == 'execution' else None
            if failure is not None and (self.failure_metric is None or
                    self.failure_metric.get('source') != 'execution_validation'):
                # Freeze the map that actually rejected the live route. A
                # prior failed planning attempt may have pinned another map.
                self.failure_metric = {**failure, 'source': 'execution_validation'}
                self.failure_map = None
                self.pin_failure()

        def metric(self, message, history=None):
            metric = self.timed(json.loads(message.data))
            (self.metrics if history is None else history).append(metric)
            if not metric.get('success', False):
                self.failure_metric = metric
                self.pin_failure()

        def graph(self, message):
            self.global_graph = message

        def planning_markers(self, message, topic):
            markers, point_count = [], 0
            finite, frame_valid, line_pairs_valid = True, True, True
            namespace_points = {}
            for marker in message.markers:
                frame_valid = frame_valid and marker.header.frame_id == 'map'
                points = []
                for point in marker.points:
                    value = [point.x, point.y, point.z]
                    finite = finite and all(math.isfinite(x) for x in value)
                    # Preserve a malformed diagnostic without breaking the
                    # physical observer's allow_nan=False evidence writer.
                    points.append([x if math.isfinite(x) else None for x in value])
                if marker.action == Marker.ADD:
                    point_count += len(points)
                    namespace_points[marker.ns] = namespace_points.get(marker.ns, 0)+len(points)
                    if marker.type == Marker.LINE_LIST:
                        line_pairs_valid = line_pairs_valid and len(points) % 2 == 0
                markers.append({
                    'namespace': marker.ns, 'id': marker.id, 'type': marker.type,
                    'action': marker.action, 'frame_id': marker.header.frame_id,
                    'stamp_ns': stamp_ns(marker), 'points': points,
                })
            reset = bool(message.markers and message.markers[0].action == Marker.DELETEALL)
            valid = finite and frame_valid and line_pairs_valid and reset
            self.planning_visualization.append(self.timed({
                'topic': topic, 'markers': markers, 'point_count': point_count,
                'namespace_points': namespace_points, 'finite_geometry': finite,
                'map_frame': frame_valid, 'line_pairs_valid': line_pairs_valid,
                'leading_deleteall': reset, 'valid_snapshot': valid,
            }))
            stats = self.visualization_stats[topic]
            stats['messages'] += 1
            stats['nonempty_messages'] += int(point_count > 0)
            stats['points'] += point_count
            stats['invalid_messages'] += int(not valid)
            for name, count in namespace_points.items():
                stats['namespace_points'][name] = stats['namespace_points'].get(name, 0)+count

        def pin_failure(self):
            if self.failure_metric is None or self.failure_map is not None:
                return
            update = next((u for u in reversed(self.map_updates)
                           if u.get('map_revision') == self.failure_metric.get('map_revision')), None)
            if update is not None:
                stamp = int(update['stamp_sec'])*1_000_000_000+int(update['stamp_nanosec'])
                self.failure_map = next((m for m in reversed(self.accepted_snapshots)
                                         if stamp_ns(m) == stamp), None)

        def command_path(self, message):
            require(message.header.frame_id == 'map', 'Unexpected commanded-path frame')
            self.path_message = message
            self.paths.append(self.timed({'path': [pose_record(p.pose) for p in message.poses]}))

        def velocity(self, message):
            self.last_command = (message.linear.x, message.angular.z)
            require(all(math.isfinite(x) for x in self.last_command), 'Non-finite wheel command')
            self.commands.append(self.timed({'linear_x': message.linear.x, 'angular_z': message.angular.z}))

        def vehicle(self, message):
            state = json.loads(message.data)
            require(state['physics'] == 'Bullet' and state['scene'] == {
                'infinite_ground': False, 'room_boxes': False,
                'mesh_vertices': 13770, 'mesh_triangles': 22557}, 'Scene is not the original Edgar mine')
            require(state['chassis_contacts'] == 0 and state['chassis_contact_steps'] == 0,
                    'Chassis touched original mesh')
            self.smb = state
            self.states.append(self.timed(state))
            if self.phase == 'supervised_mapping':
                x, y, z = state['position']
                require(-.04 <= x <= 2.10 and abs(y) <= .10 and .38 <= z <= .44,
                        'Physical initialization left its audited corridor')
                require(state['wheel_contacts'] >= 3, 'Initialization lost wheel support')

        def freshness(self):
            callback_ros_ns, received = self.get_clock().now().nanoseconds, time.monotonic()
            age = (callback_ros_ns-self.odom_stamp)*1.e-9
            receipt_age = received-self.odom_received
            return {'fresh': self.current is not None and receipt_age <= .5 and -.1 <= age <= .5,
                    'odom_present': self.current is not None, 'source_age_s': age,
                    'receipt_age_s': receipt_age, 'now_ros_ns': callback_ros_ns,
                    'last_source_stamp_ns': self.odom_stamp,
                    'last_receipt_elapsed_s': self.odom_received-started}

        def fresh(self):
            return self.freshness()['fresh']

        def stationary(self):
            return (self.fresh() and self.speed < .03 and abs(self.yaw_rate) < .05
                    and self.smb and self.smb['wheel_contacts'] == 4
                    and max(map(abs, self.smb['wheel_target'])) < 1.e-6)

        def until(self, predicate, label, seconds=8.):
            end = min(deadline, time.monotonic()+seconds)
            while not predicate():
                require(process is None or process.poll() is None, 'Launch exited; see '+str(log_path))
                if time.monotonic() >= end:
                    raise TimeoutError(label+'; control='+str(self.control))
                rclpy.spin_once(self, timeout_sec=.01)

        def spin_for(self, seconds):
            end = time.monotonic()+seconds
            self.until(lambda: time.monotonic() >= end, 'Physical observation interval', seconds+.3)

        def call(self, client, request, seconds=8.):
            self.until(client.service_is_ready, 'Service discovery', seconds)
            future = client.call_async(request)
            self.until(future.done, 'Service response', seconds)
            return future.result()

        def state(self, name):
            response = self.call(self.state_clients[name], Trigger.Request())
            require(response.success, name+': '+response.message)
            return json.loads(response.message)

        def settle(self):
            self.until(self.stationary, 'Natural four-wheel stop', 5.)
            before = pose_record(self.current)
            self.spin_for(.35)
            require(self.stationary() and math.dist(before[:3], pose_record(self.current)[:3]) < .015,
                    'Vehicle did not remain stopped')

        def validate(self, path, observation=False):
            request = ValidatePath.Request()
            request.header.frame_id = 'map'
            request.header.stamp = self.get_clock().now().to_msg()
            request.bound_mode, request.path = 3, path
            request.first_pose_is_observation = observation
            response = self.call(self.validator, request)
            return {'valid': response.valid, 'reason': response.reason,
                    'map_revision': response.map_revision, 'first_pose_is_observation': observation}

        def mapping_leg(self, signed_distance):
            start = pose_record(self.current)
            sign = 1. if signed_distance > 0 else -1.
            command = Twist()
            command.linear.x = sign*.2
            def traveled():
                value = pose_record(self.current)
                return sign*((value[0]-start[0])*math.cos(start[3])
                             +(value[1]-start[1])*math.sin(start[3]))
            end = min(deadline, time.monotonic()+16.)
            try:
                while traveled() < abs(signed_distance):
                    require(time.monotonic() < end, 'Supervised mapping leg timed out')
                    freshness = self.freshness()
                    if not freshness['fresh']:
                        summary['initialization_freshness_failure'] = self.timed(freshness)
                        raise AssertionError('Supervised mapping lost fresh source/receive odometry')
                    require(abs(wrap(pose_record(self.current)[3])) <= .08,
                            'Supervised initialization heading left its audited corridor')
                    # This remains a separately supervised mapping sweep. Hold
                    # the audited world-X corridor using measured odometry;
                    # a constant wheel command is not a straight-line guarantee
                    # on a triangle surface. Reverse travel needs the opposite
                    # cross-track correction, with the same heading feedback.
                    current = pose_record(self.current)
                    command.angular.z = max(-.15, min(.15,
                        -2.*wrap(current[3])-sign*current[1]))
                    self.teleop.publish(command)
                    self.spin_for(.04)
            finally:
                self.teleop.publish(Twist())
            self.settle()
            return {'requested_signed_distance_m': signed_distance, 'command_mps': command.linear.x,
                    'start': start, 'end': pose_record(self.current), 'measured_progress_m': traveled()}

    try:
        if args.bootstrap:
            from check_edgar_bootstrap_geometry import audit_geometry
            audit = audit_geometry(mesh)
            summary['bootstrap_geometry'] = audit
            require(audit['status'] == 'passed', 'Original-mesh bootstrap geometry audit failed')
        configs = {}
        for name, path in [('vehicle', vehicle_source), ('exploration', exploration_source),
                           ('tracking', tracking_source)]:
            configs[name] = yaml.safe_load(path.read_text())
        vehicle = configs['vehicle']['gbplanner_smb']['ros__parameters']
        require(not vehicle['room'] and not vehicle['add_ground'], 'Synthetic geometry is enabled')
        planner = configs['exploration']['gbplanner_local']['ros__parameters']
        mapper = configs['exploration']['voxblox_mapper']['ros__parameters']
        require(not mapper['clear_unknown_box_on_start'] and not mapper['esdf.clear_sphere_for_planning']
                and not mapper['use_freespace_pointcloud'] and not planner['planning.freespace_cloud_enable'],
                'Unknown-space bootstrap or synthetic freespace is enabled')
        # Private copies bound this one short search and speed. All physical
        # dimensions, TSDF projection, contact physics and tolerances are kept.
        planner['search.target_radius'] = .04
        if not args.auto_cycles:
            planner['planning.traverse_length_max'] = 1.
            planner['planning.traverse_time_max'] = 8.
        else:
            configs['exploration']['gbplanner_control']['ros__parameters']['max_cycles'] = args.auto_cycles
        configs['tracking']['gbplanner_pure_pursuit']['ros__parameters']['desired_vel'] = .2
        configs['tracking']['gbplanner_path_follower']['ros__parameters']['speed'] = .2
        saved_configs = {}
        for name, config in configs.items():
            path = args.output.with_name(args.output.stem+'_'+name+'.yaml')
            path.write_text(yaml.safe_dump(config, sort_keys=False, allow_unicode=True))
            saved_configs[name] = str(path)
        summary['configuration'] = {'saved_copies': saved_configs, 'source_hashes': {
            str(path.relative_to(source)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (vehicle_source, exploration_source, tracking_source, mesh)}}
        provenance_paths = [Path(name) for name in (
            'tools/check_edgar_motion.py', 'tools/edgar_native_exploration.py',
            'tools/check_edgar_bootstrap_geometry.py',
            'src/gbplanner_smb/src/smb_vehicle.cpp', 'src/gbplanner_smb/src/smb_node.cpp',
            'src/gbplanner_smb/src/mesh_contacts.hpp',
            'src/gbplanner_local/src/local_planner_node.cpp',
            'src/gbplanner_local/src/ground_terrain.cpp',
            'src/gbplanner_local/src/voxel_map.cpp',
            'src/gbplanner_local/src/sparse_voxel_map.cpp',
            'src/gbplanner_local/include/gbplanner_local/ground_terrain.hpp',
            'src/gbplanner_local/include/gbplanner_local/sampling_seed_sequence.hpp',
            'src/gbplanner_local/include/gbplanner_local/voxel_map.hpp',
            'src/gbplanner_local/include/gbplanner_local/observed_tsdf_simplex.hpp',
            'src/gbplanner_local/include/gbplanner_local/global_bound.hpp',
            'src/gbplanner_local/include/gbplanner_local/geofence.hpp',
            'src/gbplanner_local/src/global_memory.cpp',
            'src/gbplanner_local/src/global_persistence.cpp',
            'src/gbplanner_local/include/gbplanner_local/global_memory.hpp',
            'src/gbplanner_local/scripts/control_node.py',
            'src/gbplanner_local/scripts/path_progress.py',
            'src/gbplanner_voxblox/src/voxblox_mapper_node.cpp',
            'src/gbplanner_tracking/include/gbplanner_tracking/pure_pursuit.hpp',
            'src/gbplanner_tracking/src/pure_pursuit.cpp',
            'src/gbplanner_tracking/src/pure_pursuit_node.cpp',
            'src/gbplanner_execution/scripts/path_follower_node.py',
            'src/gbplanner_execution/scripts/trajectory_tools.py',
            'install/gbplanner_smb/lib/gbplanner_smb/smb_node',
            'install/gbplanner_smb/lib/libsmb_vehicle.so',
            'install/gbplanner_local/lib/gbplanner_local/local_planner_node',
            'install/gbplanner_local/lib/libgbplanner_local.so',
            'install/gbplanner_local/lib/gbplanner_local/control_node.py',
            'install/gbplanner_local/lib/gbplanner_local/path_progress.py',
            'install/gbplanner_voxblox/lib/libgbplanner_tsdf.so',
            'install/gbplanner_voxblox/lib/gbplanner_voxblox/voxblox_mapper_node',
            'install/gbplanner_tracking/lib/libpure_pursuit.so',
            'install/gbplanner_tracking/lib/gbplanner_tracking/pure_pursuit_node',
            'install/gbplanner_execution/lib/gbplanner_execution/path_follower_node.py')]
        summary['provenance'] = {str(path): hashlib.sha256((source/path).read_bytes()).hexdigest()
                                 for path in provenance_paths if (source/path).is_file()}
        rclpy.init(args=[])
        node = Probe()
        log = log_path.open('w')
        launch = ['ros2', 'launch', 'gbplanner_smb', 'exploration.launch.py',
                  'vehicle_config:='+saved_configs['vehicle'],
                  'exploration_config:='+saved_configs['exploration'],
                  'tracking_config:='+saved_configs['tracking'], 'collision_mesh:='+str(mesh),
                  'control:=true', 'description:=false', 'visualization:=false', 'rviz:=false']
        summary['launch_command'] = launch
        process = subprocess.Popen(launch, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=dict(os.environ))
        node.until(lambda: node.current is not None and bool(node.smb) and node.control.get('state') == 'IDLE'
                   and len(node.accepted_stamps) >= 3 and len(node.cloud_stamps) >= 3
                   and all(client.service_is_ready() for client in node.state_clients.values())
                   and node.validator.service_is_ready() and node.pci.service_is_ready(),
                   'Native mine chain startup', 20.)
        node.settle()
        initial_pose = pose_record(node.current)
        require(max(map(abs, initial_pose[:2])) < .03 and abs(initial_pose[3]) < .03
                and .38 < initial_pose[2] < .44, 'Unexpected mine starting pose')
        summary['initial_states'] = {name: node.state(name) for name in node.state_clients}
        summary['checks']['before_initialization_root'] = node.validate([copy.deepcopy(node.current)], True)
        if args.bootstrap:
            node.phase = 'supervised_mapping'
            record = {'autonomous': False, 'method': 'wheel commands stopped by measured odometry',
                      'corridor_hold': {'heading_gain_per_s': 2.,
                                        'cross_track_gain_per_m_s': 1.,
                                        'angular_limit_radps': .15},
                      'mesh_only_used_for_offline_initialization_audit': True, 'legs': []}
            summary['physical_initialization'] = record
            for distance in (2., -1.5):
                record['legs'].append(node.mapping_leg(distance))
            revision = node.update['map_revision']
            node.until(lambda: node.update.get('map_revision', 0) >= revision+3,
                       'Fresh physical lidar integrations after initialization', 8.)
            node.settle()
            node.until(lambda: node.state('velocity_mux_state')['gate'] == 'waiting_for_goal',
                       'Manual takeover released before autonomous Action', 8.)
            record['final_states'] = {name: node.state(name) for name in node.state_clients}
        node.phase = 'autonomous_preflight'
        root_check = node.validate([copy.deepcopy(node.current)], True)
        summary['checks']['observed_start_before_autonomy'] = root_check
        if not root_check['valid']:
            raise UnobservedStart('Real accepted map rejects current body/support: '+str(root_check))
        before = {name: node.state(name) for name in node.state_clients}
        start = pose_record(node.current)
        target = copy.deepcopy(node.current)
        target.position.x += .35*math.cos(start[3])
        target.position.y += .35*math.sin(start[3])
        stamped = PoseStamped()
        stamped.header.frame_id = 'map'
        stamped.header.stamp = node.get_clock().now().to_msg()
        stamped.pose = target
        node.until(lambda: node.target.get_subscription_count() > 0, 'PCI search target subscription')
        node.target.publish(stamped)
        node.spin_for(.15)
        node.phase = 'autonomous_search'
        mission = {'start': start, 'target': pose_record(target), 'before': before}
        summary['autonomous_search'] = mission
        command_index, event_index, metric_index = len(node.paths), len(node.events), len(node.metrics)
        request = PciSearch.Request()
        request.header.frame_id = 'map'
        request.header.stamp = node.get_clock().now().to_msg()
        request.use_current_state, request.not_exe_path, request.bound_mode = True, False, 3
        summary['autonomous_requests'] += 1
        response = node.call(node.pci, request, 30.)
        mission['request_accepted'] = response.success
        mission['planner_path'] = [pose_record(p) for p in response.path]
        require(response.success and len(response.path) >= 2, 'Native PCI search did not return a motion path')
        node.until(lambda: len(node.paths) > command_index, 'Native command_path publication')
        # This is an evidence check. Native PCI and the execution bridge own
        # pre-dispatch/live map validation; this observer never replaces paths.
        mission['commanded_path'] = node.paths[-1]
        mission['native_validation'] = node.validate([p.pose for p in node.path_message.poses], True)
        require(mission['native_validation']['valid'], 'Native dispatched route became invalid')
        node.until(lambda: node.control.get('state') in {'DONE', 'ERROR', 'STOPPED'},
                   'Native PCI/Action completion', 45.)
        require(node.control.get('state') == 'DONE' and node.control.get('mode') == 'search',
                'Native autonomous search did not complete: '+str(node.control))
        node.settle()
        after = {name: node.state(name) for name in node.state_clients}
        end = pose_record(node.current)
        endpoint = mission['commanded_path']['path'][-1]
        mission.update(end=end, after=after,
                       commanded_endpoint=endpoint,
                       observed_displacement_m=math.dist(start[:2], end[:2]),
                       measured_distance_m=after['smb_state']['distance_m']-before['smb_state']['distance_m'],
                       endpoint_xy_error_m=math.dist(end[:2], endpoint[:2]),
                       target_xy_error_m=math.dist(end[:2], pose_record(target)[:2]),
                       endpoint_yaw_error_rad=abs(wrap(end[3]-endpoint[3])),
                       control_events=node.events[event_index:], search_metrics=node.metrics[metric_index:])
        for name in ('execution_state', 'tracking_state'):
            require(all(after[name][key] == before[name][key]+1 for key in ('accepted', 'succeeded'))
                    and all(after[name][key] == before[name][key] for key in ('rejected', 'aborted', 'cancelled'))
                    and not after[name]['active'], 'Incomplete native Action counters: '+name)
        execution, tracking = after['execution_state'], after['tracking_state']
        require(execution['output_type'] == 'kAction'
                and execution['action_commands'] == before['execution_state']['action_commands']+1
                and execution['trajectory_commands'] == before['execution_state']['trajectory_commands'],
                'Execution bypassed the downstream Action')
        require(after['control_state']['completed_actions'] == before['control_state']['completed_actions']+1,
                'Native PCI completion counter mismatch')
        require(mission['observed_displacement_m'] > .20 and mission['measured_distance_m'] > .20
                and mission['endpoint_xy_error_m'] <= .04 and mission['target_xy_error_m'] <= .08
                and mission['endpoint_yaw_error_rad'] <= .1, 'Actual vehicle missed its short target')
        require(tracking['position_tolerance'] == .04 and tracking['yaw_tolerance'] == .1
                and node.last_command == (0., 0.), 'Tracker stop/tolerances changed')
        driven = [s for s in node.states if s['phase'] == 'autonomous_search'
                  and s['wheel_contacts'] >= 2 and max(map(abs, s['wheel_speed'])) > .1]
        require(driven, 'No driven wheel-ground contact during autonomous motion')
        mapper, accepted = after['mapper_status'], after['map_status']
        require(mapper['backend'] == 'voxblox_tsdf' and mapper['storage'] == 'sparse'
                and mapper['integrated_scans'] > before['mapper_status']['integrated_scans']
                and accepted['map_revision'] > before['map_status']['map_revision']
                and accepted['tsdf_fields'] and accepted['tsdf_cells'] > 0, 'Real TSDF pipeline did not advance')
        require(mapper['freespace']['unknown_box_operations'] == 0, 'Unknown-box clearing occurred')
        matching = node.cloud_stamps & node.raw_stamps & node.accepted_stamps
        require(len(matching) >= 5, 'Source acquisition stamps did not traverse the full map pipeline')
        summary['checks'].update(native_action_completed=True, physically_driven_wheel_contacts=len(driven),
                                 matching_scan_tsdf_accepted_stamps=len(matching), unknown_box_operations=0)
        if args.auto_cycles:
            from edgar_native_exploration import run_native_exploration
            run_native_exploration(node, summary, args.auto_cycles, args.auto_action_budget,
                                   pose_record, require)
        summary['status'] = 'passed'
    except Exception as error:
        summary['status'] = 'blocked' if isinstance(error, UnobservedStart) else 'failed'
        summary['error'] = f'{type(error).__name__}: {error}'
        if node is not None:
            summary['last_state'] = {'phase': node.phase, 'smb': node.smb, 'control': node.control,
                                     'execution': node.execution, 'tracking': node.tracking, 'map': node.update}
            failed_phase = ('initialization' if 'initialization_freshness_failure' in summary
                            else 'automatic' if 'automatic_freshness_failure' in summary else None)
            if failed_phase:
                # The run remains failed. Stop first, then retain one bounded
                # interval so a delayed source can report the stalled tick
                # before the launch tree is torn down.
                node.teleop.publish(Twist())
                node.phase = 'failed_'+failed_phase+'_stop_observation'
                try:
                    observe_until = time.monotonic()+1.
                    while time.monotonic() < observe_until:
                        node.teleop.publish(Twist())
                        node.spin_for(.04)
                except Exception as observation_error:
                    summary['stop_observation_error'] = str(observation_error)
                summary[failed_phase+'_stop_observation'] = {
                    'freshness': node.freshness(), 'smb': node.smb,
                    'pose': pose_record(node.current) if node.current is not None else None}
    finally:
        # End the physical run as a whole; a probe exception cannot leave a
        # detached launch tree or continuing supervised motor command behind.
        if node is not None and node.phase == 'supervised_mapping':
            node.teleop.publish(Twist())
        summary['cleanup'] = stop_process_tree(process)
        if node is not None:
            history_path.write_text(json.dumps({'odometry': node.trace, 'commands': node.commands,
                'vehicle': node.states, 'events': node.events, 'map_updates': node.map_updates,
                'paths': node.paths, 'search_metrics': node.metrics,
                'planning_metrics': node.local_metrics, 'global_metrics': node.global_metrics,
                'planning_visualization': node.planning_visualization},
                indent=2, allow_nan=False)+'\n')
            summary['history'] = str(history_path)
            summary['failure_input_context'] = node.failure_metric
            summary['planning_visualization'] = {
                'source': 'native planner MarkerArray subscriptions; observer only',
                'topics': node.visualization_stats,
                'geometry_history_key': 'planning_visualization',
            }
            for label, cloud in [('tsdf', node.raw_map), ('accepted', node.accepted_map),
                                 ('failure_input', node.failure_map)]:
                if cloud is not None:
                    data_path = args.output.with_name(args.output.stem+'_'+label+'.bin')
                    info_path = data_path.with_suffix('.json')
                    data = bytes(cloud.data)
                    data_path.write_bytes(data)
                    metadata = cloud_metadata(cloud, label)
                    metadata.update(data_file=str(data_path), sha256=hashlib.sha256(data).hexdigest())
                    info_path.write_text(json.dumps(metadata, indent=2)+'\n')
                    summary[label+'_snapshot'] = str(info_path)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if log is not None:
            log.close()
            failures = re.findall(r'\[([^\]]+)\]: process has died \[pid \d+, exit code (-?\d+)',
                                  log_path.read_text())
            summary['node_exit_failures'] = [{'node': name, 'exit_code': int(code)} for name, code in failures]
            if failures:
                summary['functional_checks_passed'] = summary['status'] == 'passed'
                summary['status'] = 'failed'
        summary['elapsed_s'], summary['log'] = time.monotonic()-started, str(log_path)
        if summary['elapsed_s'] > wall_budget:
            summary['status'] = 'failed'
            summary['budget_error'] = 'Exceeded the complete wall budget'
        args.output.write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
        print(json.dumps({'status': summary['status'], 'output': str(args.output),
                          'error': summary.get('error'), 'elapsed_s': summary['elapsed_s']}, indent=2))
    return 0 if summary['status'] == 'passed' else 2 if summary['status'] == 'blocked' else 1


if __name__ == '__main__':
    sys.exit(main())

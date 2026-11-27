#!/usr/bin/env python3
"""Observe real Fortress lidar/TSDF, one PCI search and local exploration/home.

The optional bootstrap is a separately recorded, supervised wheel-driven sweep
of an offline-audited original-mesh corridor. No geometry or map is injected
into the mapper. This experiment does not claim collision-free operation when
native contact observations are unavailable.
"""
import argparse
from collections import deque
import copy
import hashlib
import json
import math
import os
import re
from pathlib import Path
import signal
import subprocess
import sys
import time

sys.dont_write_bytecode = True
from check_edgar_mapping import cloud_metadata, stamp_ns, stop_process_tree
from check_edgar_motion import pose_record, wrap, require
from check_fortress_mapping import describe_cloud


def shutdown_log_events(log_text):
    """Keep launch's signal escalation even when a node eventually exits zero."""
    plain = re.sub(r'\x1b\[[0-9;]*m', '', log_text)
    failures = re.findall(r'\[([^\]]+)\]: process has died \[pid \d+, exit code (-?\d+)', plain)
    escalations = re.findall(
        r"\[([^\]]+)\]: process\[[^\]]+\] failed to terminate '[^']+' seconds "
        r"after receiving '(SIGINT|SIGTERM)', escalating to '(SIGTERM|SIGKILL)'", plain)
    return ([{'name': name, 'exit_code': int(code)} for name, code in failures],
            [{'name': name, 'from_signal': previous, 'to_signal': following}
             for name, previous, following in escalations])


def bootstrap_waypoints(value):
    """Outbound XY stops, following the fixed mission-home position (0.5, 0)."""
    try:
        points = json.loads(value)
    except (TypeError, ValueError) as error:
        raise argparse.ArgumentTypeError('Expected a JSON array of [x, y] waypoints') from error
    if not isinstance(points, list) or not 1 <= len(points) <= 32:
        raise argparse.ArgumentTypeError('Supply between 1 and 32 outbound waypoints')
    previous = [.5, 0.]
    for point in points:
        if (not isinstance(point, list) or len(point) != 2 or
                any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                    not math.isfinite(v) for v in point)):
            raise argparse.ArgumentTypeError('Every waypoint must contain two finite numbers')
        if math.dist(previous, point) < .08:
            raise argparse.ArgumentTypeError('Successive stops must be at least 0.08 m apart')
        previous = point
    return points


def segment_coordinates(point, start, target):
    dx, dy = target[0]-start[0], target[1]-start[1]
    length = math.hypot(dx, dy)
    if length < 1.e-12:
        return 0., math.dist(point[:2], start), 0.
    x, y = point[0]-start[0], point[1]-start[1]
    return (x*dx+y*dy)/length, (dx*y-dy*x)/length, length


def audit_waypoint_bootstrap(mesh, points, body_z=(.27, .58)):
    """Cover continuous outbound/return sweeps and all in-place rotations."""
    from check_edgar_bootstrap_geometry import audit_geometry
    route = [[0., 0.], [.5, 0.], *points]
    sections = []
    # 0.9 m covers the rotated 1.27 m square; the original 0.1 m
    # tracking tolerance is included in the audited envelope, not added later.
    for index, (a, b) in enumerate(zip(route, route[1:])):
        count = math.ceil(math.dist(a, b)/.1)
        require(count <= 1000, 'Bootstrap segment exceeds the finite geometry-audit limit')
        for j in range(count):
            p = [a[k]+(b[k]-a[k])*j/count for k in range(2)]
            q = [a[k]+(b[k]-a[k])*(j+1)/count for k in range(2)]
            corridor = tuple((min(p[k], q[k])-1., max(p[k], q[k])+1.) for k in range(2))
            audit = audit_geometry(mesh, corridor=corridor, body_z=body_z)
            sections.append({'leg': index, 'from': p, 'to': q,
                'status': audit['status'], 'checks': audit['checks'],
                'body_intersection_indices': audit['body_prism_intersecting_triangle_indices_zero_based'],
                'above_floor_intersection_indices':
                    audit['whole_swept_envelope']['intersecting_triangle_indices_zero_based']})
    return {'status': 'passed' if all(s['status'] == 'passed' for s in sections) else 'failed',
        'scope': 'Original-mesh continuous polyline envelopes, including all yaw orientations; no map injection',
        'mesh': str(mesh), 'mesh_sha256': hashlib.sha256(mesh.read_bytes()).hexdigest(),
        'outbound_xy_m': route, 'return_xy_m': list(reversed(route[1:-1])),
        'envelope_half_width_m': 1., 'included_tracking_tolerance_m': .1,
        'maximum_subsegment_m': .1, 'body_prism_z_m': list(body_z),
        'floor_filter': {'maximum_slope_degrees': 1., 'z_range_m': [.12, .20]},
        'return_uses_same_audited_segments': True, 'sections': sections}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scene', choices=('edgar', 'pittsburgh'), default='edgar',
                        help='Retained native world and matching scene configuration')
    parser.add_argument('--bootstrap', action='store_true')
    parser.add_argument('--contacts', action='store_true',
                        help='Record native contact diagnostics and stop on an observed chassis contact')
    parser.add_argument('--bootstrap-forward', type=float, default=3.2,
                        help='Length of the audited straight forward sweep')
    parser.add_argument('--bootstrap-return-x', type=float, default=.5,
                        help='Straight bootstrap return X before choosing home; must lie between zero and the forward endpoint')
    parser.add_argument('--bootstrap-waypoints', type=bootstrap_waypoints,
                        help='JSON outbound XY stops after [0.5,0], e.g. [[2.5,0],[3,0.7]]; enables bootstrap and returns along the same stops')
    parser.add_argument('--auto-cycles', type=int, default=1,
                        help='Native automatic local cycles before native homing; zero means search only')
    parser.add_argument('--ground-maneuvers', action='store_true',
                        help='Enable explicit pre-departure SE2 planning and primitive execution for local/search/homing')
    parser.add_argument('--homing-after-search', action='store_true',
                        help='With --auto-cycles 0, explicitly return after search; separate from full automatic exploration')
    parser.add_argument('--planner-profile', type=Path,
                        help='Planner-only YAML overlay passed to the native Fortress launch')
    parser.add_argument('--sensor-profile', choices=('compact', 'upstream'),
                        help='Defaults to compact for Edgar and original dual lidar for Pittsburgh')
    parser.add_argument('--renderer-prefix', type=Path,
                        help='Use the isolated project renderer through the normal ROS launch')
    parser.add_argument('--gpu-rays-min-cubemap-size', type=int, choices=(128, 256, 512, 1024), default=512,
                        help='Internal renderer minimum; used with --renderer-prefix, output scans stay unchanged')
    parser.add_argument('--observe-background', action='store_true',
                        help='Before search/exploration, build one nonexecuted live local graph and observe two timer updates')
    parser.add_argument('--global-first', action='store_true',
                        help='Select and execute a native global frontier before local cycles; seeds its graph with the live nonexecuting local request')
    parser.add_argument('--wall-budget', type=float, default=180.)
    parser.add_argument('--capture-map-over', type=int, default=0,
                        help='Save the first unmodified planner voxel_map with at least this many cells')
    parser.add_argument('--capture-home', action='store_true',
                        help='Capture exact planner map, retained graph and home edge reasons on home setup/disconnection/homing failure')
    parser.add_argument('--trace-voxels', type=json.loads,
                        help='JSON list of integer XYZ triples to trace through native TSDF integration')
    parser.add_argument('--trace-transitions-only', action='store_true',
                        help='With --trace-voxels, log only occupancy/presence changes; retain full values in bounded raw-scan snapshots')
    parser.add_argument('--lateral-slip-compliance', type=float, default=0.,
                        help='Optional native WheelSlip coefficient; saved with results, original rigid model by default')
    parser.add_argument('--wheel-separation-multiplier', type=float, default=1.,
                        help='Explicit diff_drive effective-track calibration; nominal physical geometry is unchanged')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    if args.sensor_profile is None:
        args.sensor_profile = 'upstream' if args.scene == 'pittsburgh' else 'compact'
    if args.scene == 'pittsburgh':
        if not args.contacts:
            parser.error('Pittsburgh motion requires --contacts')
        if args.sensor_profile != 'upstream':
            parser.error('Pittsburgh mission validation uses the original upstream sensor head')
        if args.bootstrap_waypoints is not None:
            parser.error('Pittsburgh currently has a straight bootstrap corridor audit only')
        if args.planner_profile is None:
            args.planner_profile = root/'src/gbplanner_smb/config/upstream_planning.yaml'
    if args.trace_voxels is not None and (not isinstance(args.trace_voxels, list)
            or not 1 <= len(args.trace_voxels) <= 2048
            or any(not isinstance(v, list) or len(v) != 3
                   or any(type(x) is not int or abs(x) > 1000000 for x in v)
                   for v in args.trace_voxels)):
        parser.error('--trace-voxels requires 1-2048 integer XYZ triples within +/-1000000')
    if args.trace_voxels is not None and len({tuple(v) for v in args.trace_voxels}) != len(args.trace_voxels):
        parser.error('--trace-voxels must not contain duplicate XYZ triples')
    if args.trace_transitions_only and args.trace_voxels is None:
        parser.error('--trace-transitions-only requires --trace-voxels')
    if args.global_first:
        args.observe_background = True
    if args.homing_after_search and (args.auto_cycles != 0 or args.global_first):
        parser.error('--homing-after-search requires --auto-cycles 0 and no --global-first')
    sensor_topics = ({'smb_lidar': {'raw': 'gazebo/points_raw', 'adapted': 'points'}}
        if args.sensor_profile == 'compact' else {
            'velodyne': {'raw': 'gazebo/velodyne_points_raw', 'adapted': 'points/velodyne'},
            'os1': {'raw': 'gazebo/os1_points_raw', 'adapted': 'points/os1'}})
    body_z = (.27, 1.18 if args.sensor_profile == 'upstream' else .58)
    if args.bootstrap_waypoints is not None:
        args.bootstrap = True
    if not math.isfinite(args.lateral_slip_compliance) or args.lateral_slip_compliance < 0:
        parser.error('--lateral-slip-compliance must be finite and nonnegative')
    if not math.isfinite(args.wheel_separation_multiplier) or args.wheel_separation_multiplier <= 0:
        parser.error('--wheel-separation-multiplier must be finite and positive')
    if args.auto_cycles < 0 or not math.isfinite(args.wall_budget) or args.wall_budget < 40:
        parser.error('Nonnegative cycles and finite wall budget >= 40 seconds are required')
    if args.capture_map_over < 0 or args.capture_map_over > 2147483647:
        parser.error('--capture-map-over must be zero or a positive signed 32-bit cell count')
    if not math.isfinite(args.bootstrap_forward) or args.bootstrap_forward < .6:
        parser.error('--bootstrap-forward must be finite and at least 0.6 m')
    if not math.isfinite(args.bootstrap_return_x) or not 0. < args.bootstrap_return_x < args.bootstrap_forward:
        parser.error('--bootstrap-return-x must be finite and satisfy 0 < return X < bootstrap forward')
    if args.bootstrap_waypoints is not None and args.bootstrap_return_x != .5:
        parser.error('--bootstrap-return-x applies only to a straight bootstrap; polyline returns to [0.5,0]')
    if args.planner_profile is not None:
        args.planner_profile = args.planner_profile.expanduser().resolve()
        if not args.planner_profile.is_file():
            parser.error('--planner-profile must name an existing YAML file')
    if args.renderer_prefix is not None:
        args.renderer_prefix = args.renderer_prefix.expanduser().resolve()
        if not args.renderer_prefix.is_dir():
            parser.error('--renderer-prefix must name an existing renderer installation')
    args.output = args.output.resolve()
    log_path = args.output.with_suffix('.log')
    history_path = args.output.with_name(args.output.stem+'_history.json')
    map_trace_path = args.output.with_name(args.output.stem+'_map_trace.jsonl')
    snapshot_trace_path = args.output.with_name(args.output.stem+'_snapshot_trace.jsonl')
    background_dir = args.output.with_name(args.output.stem+'_background')
    home_snapshot_dir = args.output.with_name(args.output.stem+'_home') if args.capture_home else None
    voxel_capture_dir = args.output.with_name(args.output.stem+'_voxels') if args.trace_voxels else None
    if voxel_capture_dir is not None and voxel_capture_dir.exists():
        parser.error('Voxel trace output already exists: '+str(voxel_capture_dir))
    if home_snapshot_dir is not None and home_snapshot_dir.exists():
        parser.error('Home snapshot output already exists: '+str(home_snapshot_dir))
    existing = [path for path in (args.output, log_path, history_path, map_trace_path, snapshot_trace_path,
                background_dir, args.output.with_name(args.output.stem+'_failure_map.bin')) if path.exists()]
    capture_map_path = args.output.with_name(args.output.stem+'_capacity_map.bin')
    if args.capture_map_over and capture_map_path.exists():
        existing.append(capture_map_path)
    if existing:
        parser.error('Use a new --output name; existing evidence will not be overwritten: '+
                     ', '.join(str(path) for path in existing))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[1]
    started, deadline = time.monotonic(), time.monotonic()+args.wall_budget-10.
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    summary = {'status': 'failed', 'domain': os.environ['ROS_DOMAIN_ID'],
        'started_monotonic_s': started,
        'scene': args.scene,
        'scope': args.scene+' retained mesh in Gazebo Fortress → native gpu_lidar → Voxblox → '
                 'planner → PCI → public and downstream PathFollower Actions → native wheel dynamics',
        'wall_budget_s': args.wall_budget, 'bootstrap_requested': args.bootstrap,
        'bootstrap_return_x_m': args.bootstrap_return_x,
        'sensor_profile': args.sensor_profile, 'sensor_topics': sensor_topics,
        'renderer': {'selection': 'project' if args.renderer_prefix is not None else 'inherit',
                     'prefix': str(args.renderer_prefix) if args.renderer_prefix is not None else None,
                     'minimum_cubemap_size': args.gpu_rays_min_cubemap_size
                         if args.renderer_prefix is not None else None},
        'bootstrap_waypoints': args.bootstrap_waypoints,
        'background_observation_requested': args.observe_background,
        'global_first_requested': args.global_first,
        'requested_auto_cycles': args.auto_cycles, 'autonomous_requests': 0,
        'explicit_homing_after_search': args.homing_after_search,
        'lateral_slip_compliance': args.lateral_slip_compliance,
        'initialization_counts_as_autonomy': False,
        'native_contact_diagnostics_available': args.contacts,
        'collision_free_claim': False, 'checks': {}, 'observation_captures': {}}
    process = node = log = None

    import yaml
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
    from nav_msgs.msg import Odometry, Path as RosPath
    from planner_msgs.msg import Graph
    from planner_msgs.srv import PciSearch, PciTrigger, PciGlobal, PciSetHomingPos, PciHomingTrigger, PlannerSrv, ValidatePath
    from rcl_interfaces.srv import GetParameters
    from rclpy.node import Node
    from rclpy.parameter import Parameter, parameter_value_to_python
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import JointState, PointCloud2
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from rosidl_runtime_py.convert import message_to_ordereddict

    class Probe(Node):
        def __init__(self, collect_background=False):
            super().__init__('fortress_autonomy_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.phase = 'startup'
            self.current = self.last_command = self.global_graph = self.path_message = None
            self.global_graph_received = None
            self.state_history = self.state_history_received = None
            self.state_history_publications = []
            self.odom_stamp = 0
            self.odom_received = 0.
            self.odom_received_ros_ns = 0
            self.speed = self.yaw_rate = math.inf
            self.mapping_segment = None
            self.gazebo, self.control, self.execution, self.tracking, self.update = {}, {}, {}, {}, {}
            self.trace, self.commands, self.states, self.events, self.map_updates = [], [], [], [], []
            self.terminal_outcomes = {}
            self.paths, self.metrics, self.local_metrics, self.global_metrics, self.joints = [], [], [], [], []
            self.limited_commands = []
            self.clouds = {}
            self.capacity_map_capture = None
            self.sensor_stamps = {topic: set() for pair in sensor_topics.values() for topic in pair.values()}
            self.sensor_counts = {topic: 0 for topic in self.sensor_stamps}
            self.sensor_receipts = {}
            # One lookup window, two references per container to the same two
            # messages. Do not retain twelve large TSDF frames for pictures.
            self.accepted_snapshots = deque(maxlen=2)
            self.accepted_receipts = deque(maxlen=2)
            self.failure_map = self.failure_map_revision = None
            self.failure_map_request = None
            self.cloud_stamps, self.raw_stamps, self.accepted_stamps = set(), set(), set()
            durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Odometry, 'odometry', self.odometry, qos_profile_sensor_data)
            for topic in self.sensor_stamps:
                self.create_subscription(PointCloud2, topic, lambda m, t=topic: self.cloud(m, t), qos_profile_sensor_data)
            self.create_subscription(PointCloud2, 'voxel_map', lambda m: self.cloud(m, 'voxel_map'), durable)
            self.create_subscription(PointCloud2, 'accepted_voxel_map', lambda m: self.cloud(m, 'accepted_voxel_map'), durable)
            self.create_subscription(String, 'map_updates', self.mapping, durable)
            self.create_subscription(String, 'gazebo_state', self.vehicle, 10)
            self.create_subscription(Twist, 'cmd_vel', self.velocity, 10)
            self.create_subscription(TwistStamped, 'smb_drive/cmd_vel_out', self.limited_velocity, 10)
            self.create_subscription(JointState, 'joint_states', self.joint, qos_profile_sensor_data)
            self.create_subscription(RosPath, 'command_path', self.command_path, durable)
            self.create_subscription(Graph, 'global_graph', self.graph, durable)
            self.create_subscription(String, 'state_history', self.history, durable)
            for topic, target in [('search_metrics', self.metrics), ('planning_metrics', self.local_metrics),
                                  ('global_metrics', self.global_metrics)]:
                self.create_subscription(String, topic,
                    lambda m, rows=target: rows.append(self.timed(json.loads(m.data))), durable)
            for topic, attr in [('control_status', 'control'), ('execution_status', 'execution'),
                                ('tracking_status', 'tracking')]:
                # These publishers retain state transitions. Discovery may
                # finish after the initial IDLE publication, so request its
                # retained sample instead of waiting for a new transition.
                self.create_subscription(String, topic, lambda m, a=attr: self.status(m, a), durable)
            self.teleop = self.create_publisher(Twist, 'joy_teleop/cmd_vel', 10)
            self.target = self.create_publisher(PoseStamped, 'search_target', durable)
            self.pci = self.create_client(PciSearch, 'pci_search')
            self.local_trigger = self.create_client(PciTrigger, 'pci_trigger')
            self.global_trigger = self.create_client(PciGlobal, 'pci_global')
            self.home_setter = self.create_client(PciSetHomingPos, 'pci_set_homing_pos')
            self.home_trigger = self.create_client(PciHomingTrigger, 'pci_homing_trigger')
            self.local_planner = self.create_client(PlannerSrv, 'gbplanner') if args.observe_background else None
            self.planner_parameters = self.create_client(GetParameters, 'gbplanner_local/get_parameters')
            self.mapper_parameters = self.create_client(GetParameters, 'voxblox_mapper/get_parameters')
            self.drive_parameters = self.create_client(GetParameters, 'smb_drive/get_parameters')
            self.validator = self.create_client(ValidatePath, 'validate_path')
            self.state_clients = {name: self.create_client(Trigger, name) for name in
                ('gazebo_state', 'mapper_status', 'map_status', 'control_state', 'execution_state',
                 'tracking_state', 'velocity_mux_state')}
            # Read timer counters during ordinary exploration too. Graph
            # publication growth also includes local absorption and cannot
            # establish how many vertices the background timer inserted.
            if args.observe_background or collect_background:
                self.state_clients.update({name: self.create_client(Trigger, name)
                    for name in ('expansion_status', 'global_status')})

        def timed(self, value):
            return {'time_s': time.monotonic()-started, 'sim_s': self.get_clock().now().nanoseconds*1.e-9,
                    'phase': self.phase, **value}

        def odometry(self, message):
            require(message.header.frame_id == 'map' and message.child_frame_id == 'base_link', 'Unexpected odometry frames')
            self.current = copy.deepcopy(message.pose.pose)
            twist = message.twist.twist
            self.speed = math.sqrt(twist.linear.x**2+twist.linear.y**2+twist.linear.z**2)
            self.yaw_rate = twist.angular.z
            self.odom_received, self.odom_stamp = time.monotonic(), stamp_ns(message)
            self.odom_received_ros_ns = self.get_clock().now().nanoseconds
            pose = pose_record(self.current)
            require(all(math.isfinite(v) for v in pose+[self.speed, self.yaw_rate]), 'Non-finite physical odometry')
            self.trace.append(self.timed({'pose': pose, 'stamp_ns': self.odom_stamp,
                'speed_mps': self.speed, 'yaw_rate': self.yaw_rate,
                'body_linear_velocity': [twist.linear.x, twist.linear.y, twist.linear.z],
                'body_angular_velocity': [twist.angular.x, twist.angular.y, twist.angular.z]}))
            if self.phase == 'supervised_mapping':
                if args.bootstrap_waypoints is None:
                    require(-.04 <= pose[0] <= args.bootstrap_forward+.10 and abs(pose[1]) <= .10 and .38 <= pose[2] <= .44,
                            'Physical supervised motion left the audited corridor')
                else:
                    require(self.mapping_segment is not None, 'Missing active supervised segment')
                    progress, cross, length = segment_coordinates(pose, *self.mapping_segment)
                    # Endpoint circles also cover drift while turning at a stop.
                    outside = math.hypot(cross, max(-progress, progress-length, 0.))
                    require(outside <= .10 and .38 <= pose[2] <= .44,
                            'Physical supervised motion left the audited polyline envelope')

        def cloud(self, message, topic):
            self.clouds[topic] = message
            if (topic == 'voxel_map' and args.capture_map_over
                    and message.width * message.height >= args.capture_map_over
                    and self.capacity_map_capture is None):
                # Retain one message. Writing a 100 MB snapshot here would
                # delay odometry callbacks during active motion; serialize it
                # with the other evidence after the mission has stopped.
                self.capacity_map_capture = message
            if topic in self.sensor_stamps:
                self.sensor_stamps[topic].add(stamp_ns(message))
                self.sensor_counts[topic] += 1
                self.sensor_receipts[topic] = self.timed({})
                if any(topic == pair['adapted'] for pair in sensor_topics.values()):
                    self.cloud_stamps.add(stamp_ns(message))
            else:
                {'voxel_map': self.raw_stamps, 'accepted_voxel_map': self.accepted_stamps}[topic].add(stamp_ns(message))
            if topic == 'accepted_voxel_map':
                self.accepted_snapshots.append(message)
                self.accepted_receipts.append((message, self.timed({})))
                self.match_failure_map()

        def graph(self, message):
            self.global_graph = message
            self.global_graph_received = self.timed({})

        def history(self, message):
            self.state_history = json.loads(message.data)
            self.state_history_received = self.timed({})
            self.state_history_publications.append({'received': self.state_history_received,
                                                    'payload': self.state_history})

        def mapping(self, message):
            self.update = json.loads(message.data)
            self.map_updates.append(self.timed(self.update))
            self.match_failure_map()

        def match_failure_map(self):
            request = self.failure_map_request
            if request is None:
                return
            revision = request['requested_revision']
            updates = [u for u in self.map_updates if u.get('map_revision') == revision]
            if not updates:
                request['pending'] = True
                return
            update = updates[-1]
            source_stamp = int(update['stamp_sec'])*1_000_000_000+int(update['stamp_nanosec'])
            same_stamp_updates = [u for u in self.map_updates
                if int(u['stamp_sec'])*1_000_000_000+int(u['stamp_nanosec']) == source_stamp]
            request.update(map_update_revision=update.get('map_revision'),
                map_update_stamp_ns=source_stamp,
                known_cells=update.get('known_cells'),
                same_stamp_update_revisions=[u.get('map_revision') for u in same_stamp_updates],
                source_stamp_unique_as_observed=len(same_stamp_updates) == 1)
            candidates = [m for m in self.accepted_snapshots if stamp_ns(m) == source_stamp]
            request['matching_accepted_clouds_in_window'] = len(candidates)
            # Keep the selected message outside the bounded receive window.
            # Later unrelated maps must not turn a completed capture pending
            # again, and repeated execution status messages must not erase it.
            retained_match = (self.failure_map is not None
                and self.failure_map_revision == revision
                and stamp_ns(self.failure_map) == source_stamp)
            if not retained_match and candidates:
                self.failure_map = candidates[-1]
                self.failure_map_revision = revision
                retained_match = True
            request['map_stamp_match'] = retained_match
            request['pending'] = not retained_match
            if not retained_match:
                return
            request['cloud_width'] = int(self.failure_map.width)
            request['known_cells_matches_cloud_width'] = (
                request['known_cells'] == request['cloud_width'])
            # Acquisition-stamp equality selects a candidate; simultaneous
            # sensor inputs can share a stamp, so it does not prove unique
            # identity with this validator's immutable map snapshot.
            request['exact_revision_match'] = False

        def vehicle(self, message):
            self.gazebo = json.loads(message.data)
            require(self.gazebo['physics'] == 'Gazebo Fortress', 'Unexpected physics source')
            self.states.append(self.timed(self.gazebo))
            if args.contacts:
                base = self.gazebo.get('contact_diagnostics', {}).get('parts', {}).get('base', {})
                require(base.get('contact_pairs', 0) == 0, 'Native chassis contact observed')

        def joint(self, message):
            self.joints.append(self.timed({'names': list(message.name),
                'position': list(message.position), 'velocity': list(message.velocity)}))

        def velocity(self, message):
            self.last_command = (message.linear.x, message.angular.z)
            self.commands.append(self.timed({'linear_x': message.linear.x, 'angular_z': message.angular.z}))

        def limited_velocity(self, message):
            self.limited_commands.append(self.timed({'stamp_ns': stamp_ns(message),
                'linear_x': message.twist.linear.x, 'angular_z': message.twist.angular.z}))

        def command_path(self, message):
            self.path_message = message
            self.paths.append(self.timed({'path': [pose_record(p.pose) for p in message.poses]}))

        def status(self, message, name):
            value = json.loads(message.data)
            setattr(self, name, value)
            self.events.append(self.timed({'layer': name, **value}))
            outcome = value.get('terminal_outcome') if name == 'execution' else None
            if outcome:
                key = (self.goal_key(outcome['goal_id']), outcome['generation'])
                previous = self.terminal_outcomes.get(key)
                require(previous is None or previous['outcome'] == outcome,
                        'One input Action acquired conflicting terminal outcomes')
                if previous is None:
                    self.terminal_outcomes[key] = self.timed({'outcome': copy.deepcopy(outcome)})
            failure = value.get('validation_failure') if name == 'execution' else None
            if failure and 'map_revision' in failure:
                self.capture_failure_map(failure['map_revision'])

        def freshness(self):
            now = self.get_clock().now().nanoseconds
            age, receipt = (now-self.odom_stamp)*1.e-9, time.monotonic()-self.odom_received
            receipt_ros = (now-self.odom_received_ros_ns)*1.e-9
            return {'fresh': self.current is not None and -.1 <= age <= .5 and 0 <= receipt_ros <= .5,
                    'command_fresh': self.current is not None and -.1 <= age <= .5 and receipt <= .5,
                    'source_age_s': age, 'receipt_age_s': receipt, 'receipt_ros_age_s': receipt_ros,
                    'clock_domain': 'ROS simulation time'}

        def synchronized_freshness(self):
            """Drain a delayed /clock before judging a just-received future odom."""
            before = value = self.freshness()
            if value['source_age_s'] >= -.1 or value['receipt_age_s'] > .5:
                return value
            # The observer deserializes large map messages in this executor.
            # ROS /clock and odometry are separate subscriptions: a queued
            # odometry can arrive before its corresponding clock callback.
            # Native controllers retain their independent watchdogs. Wait for
            # clock delivery only; do not clamp time or change freshness limits.
            began = time.monotonic()
            while rclpy.ok() and time.monotonic()-began < .5:
                rclpy.spin_once(self, timeout_sec=.005)
                value = self.freshness()
                if value['source_age_s'] >= -.1:
                    break
            summary.setdefault('observer_clock_delivery_waits', []).append(self.timed({
                'before': before, 'after': value, 'wall_elapsed_s': time.monotonic()-began,
                'wait_budget_s': .5, 'recovered': value['fresh']}))
            return value

        def until(self, predicate, label, seconds=8.):
            end = min(deadline, time.monotonic()+seconds)
            while not predicate():
                require(process is None or process.poll() is None, 'Gazebo launch exited; see '+str(log_path))
                if time.monotonic() >= end:
                    raise TimeoutError(label+'; control='+str(self.control))
                rclpy.spin_once(self, timeout_sec=.01)

        def spin_for(self, seconds):
            end = time.monotonic()+seconds
            self.until(lambda: time.monotonic() >= end, 'Observation interval', seconds+.3)
            recent = self.local_metrics[-5:]
            if self.phase == 'automatic_exploration' and len(recent) == 5:
                failures = [m.get('reference_path_diagnostics') for m in recent]
                if (all(m.get('reason') == 'reference_path_unavailable' for m in recent)
                        and all(f == failures[0] for f in failures)):
                    raise RuntimeError('Five local requests hit the same unavailable graph anchor; preserve evidence for diagnosis')

        def call(self, client, request, seconds=8.):
            if client is self.local_trigger:
                # The shared automatic observer uses a Bullet experiment's
                # 0.2 m/s request. This run keeps native configured speed:
                # zero is PCI's existing "no request override" value.
                request.vel_max = 0.
                summary['automatic_trigger_velocity_override'] = 0.
            name = getattr(client, 'srv_name', 'service')
            self.until(client.service_is_ready, name+' discovery', seconds)
            observed = self.timed({'service': name, 'observer_budget_s': seconds, 'response_received': False})
            summary.setdefault('service_observations', []).append(observed)
            began = time.monotonic()
            try:
                future = client.call_async(request)
                self.until(future.done, name+' response', seconds)
                observed['response_received'] = True
                return future.result()
            finally:
                observed['elapsed_s'] = time.monotonic()-began

        def state(self, name):
            # Read-only status may share the native planner's map callback
            # queue. This observer budget never alters a motion watchdog.
            response = self.call(self.state_clients[name], Trigger.Request(), 30.)
            require(response.success, name+': '+response.message)
            return json.loads(response.message)

        def snapshots(self):
            return {name: self.state(name) for name in self.state_clients}

        def read_planner_profile(self, expected, mapper=False, drive=False):
            # A single native get_parameters call verifies the effective node,
            # rather than treating a source-file hash as proof of launch wiring.
            names = list(expected)
            queried = self.timed({})
            request = GetParameters.Request()
            request.names = names
            client = self.drive_parameters if drive else self.mapper_parameters if mapper else self.planner_parameters
            response = self.call(client, request, 30.)
            require(response is not None and len(response.values) == len(names),
                    'Incomplete node parameter readback')
            actual = {}
            for name, parameter in zip(names, response.values):
                value = parameter_value_to_python(parameter)
                actual[name] = value.tolist() if hasattr(value, 'tolist') else value
            return {'query_started': queried, 'query_completed': self.timed({}),
                    'node': 'smb_drive' if drive else 'voxblox_mapper' if mapper else 'gbplanner_local',
                    'expected': expected, 'actual': actual,
                    'mismatches': {name: {'expected': expected[name], 'actual': actual[name]}
                                   for name in names if actual[name] != expected[name]}}

        def accepted_for_revision(self, revision):
            update = next((u for u in reversed(self.map_updates)
                           if u.get('map_revision') == revision), None)
            if update is None:
                return None
            source_stamp = int(update['stamp_sec'])*1_000_000_000+int(update['stamp_nanosec'])
            return next(((message, received) for message, received in reversed(self.accepted_receipts)
                         if stamp_ns(message) == source_stamp), None)

        def capture_background(self, label, requested_revision=None):
            # No spin or service call occurs while these references are frozen.
            # Their source timestamps are independent: this is an observer
            # capture, not an atomic planner/map/odometry transaction.
            captured = self.timed({'unix_time_ns': time.time_ns()})
            accepted = self.accepted_for_revision(requested_revision) if requested_revision is not None else None
            exact_revision = accepted is not None
            if accepted is None and self.accepted_receipts:
                accepted = self.accepted_receipts[-1]
            require(accepted is not None and self.current is not None and self.global_graph is not None
                    and self.state_history is not None,
                    'Observation capture lacks map, measured odometry, graph or visit history')
            cloud, cloud_received = accepted
            pose = copy.deepcopy(self.current)
            odometry_stamp = self.odom_stamp
            odometry_received = {'time_s': self.odom_received-started,
                                 'sim_s': self.odom_received_ros_ns*1.e-9}
            graph, graph_received = self.global_graph, copy.deepcopy(self.global_graph_received)
            history, history_received = self.state_history, copy.deepcopy(self.state_history_received)
            cloud_stamp = stamp_ns(cloud)
            matching_updates = [copy.deepcopy(update) for update in self.map_updates
                if int(update['stamp_sec'])*1_000_000_000+int(update['stamp_nanosec']) == cloud_stamp]
            background_dir.mkdir(exist_ok=True)
            map_path = background_dir/(label+'_map.bin')
            graph_path = background_dir/(label+'_graph.json')
            history_file = background_dir/(label+'_state_history.json')
            record_path = background_dir/(label+'_snapshot.json')
            data = bytes(cloud.data)
            with map_path.open('xb') as stream:
                stream.write(data)
            graph_text = json.dumps(message_to_ordereddict(graph), indent=2, allow_nan=False)+'\n'
            with graph_path.open('x') as stream:
                stream.write(graph_text)
            history_text = json.dumps(history, indent=2, allow_nan=False)+'\n'
            with history_file.open('x') as stream:
                stream.write(history_text)
            record = {'label': label, 'observer_capture': captured,
                'atomic_snapshot': False,
                'association': 'Latest independently received odometry, global graph and visit history at observer capture; '
                    'accepted-map revision association uses map_updates acquisition stamps. '
                    'Two sensors can share an acquisition stamp; a stamp match alone does not prove snapshot identity. '
                    'The planner input odometry stamp and timer input map revision are not reported by those services.',
                'accepted_map': {**cloud_metadata(cloud, 'accepted_voxel_map'),
                    'source': 'Native lidar → Voxblox → planner accepted_voxel_map',
                    'received': cloud_received, 'stamp_ns': cloud_stamp,
                    'data_file': str(map_path), 'sha256': hashlib.sha256(data).hexdigest(),
                    'requested_revision': requested_revision, 'requested_revision_stamp_match': exact_revision,
                    'exact_revision_identity_proven': False,
                    'selection': 'Requested revision stamp match' if exact_revision else 'Latest received accepted map',
                    'matching_map_updates': matching_updates},
                'odometry': {'topic': 'odometry', 'source': 'Gazebo Fortress measured base odometry',
                    'frame_id': 'map', 'child_frame_id': 'base_link', 'stamp_ns': odometry_stamp,
                    'received': odometry_received, 'pose': message_to_ordereddict(pose),
                    'position_and_yaw': pose_record(pose)},
                'global_graph': {'topic': 'global_graph', 'source': 'gbplanner_local global graph publication',
                    'frame_id': graph.header.frame_id, 'stamp_ns': stamp_ns(graph),
                    'received': graph_received, 'vertices': len(graph.vertices), 'edges': len(graph.edges),
                    'data_file': str(graph_path), 'sha256': hashlib.sha256(graph_text.encode()).hexdigest(),
                    'publication_note': 'Unchanged graphs may reuse an earlier durable publication'},
                'state_history': {'topic': 'state_history', 'stamp_ns': history.get('stamp_ns'),
                    'received': history_received, 'revision': history.get('revision'),
                    'total_states': history.get('total_states'), 'saved_states': len(history.get('states', [])),
                    'first_index': history.get('first_index'), 'truncated': history.get('truncated'),
                    'complete': not history.get('truncated', False) and history.get('first_index') == 0 and
                        len(history.get('states', [])) == history.get('total_states'),
                    'data_file': str(history_file), 'sha256': hashlib.sha256(history_text.encode()).hexdigest(),
                    'publication_note': 'Complete received topic payload; source truncation is explicitly retained'},
                'snapshot_file': str(record_path)}
            with record_path.open('x') as stream:
                stream.write(json.dumps(record, indent=2, allow_nan=False)+'\n')
            summary['observation_captures'][label] = record
            return record

        def observe_background(self, expected_profile):
            self.phase = 'background_observation'
            record = {'status': 'failed', 'service': 'gbplanner',
                'local_request_executes_path': False, 'explicit_expand_requests': 0,
                'qualification_field': 'planning_metrics.graph_build_completed',
                'timer_observations': [], 'captures': {}}
            summary['background_observation'] = record
            record['profile_readback'] = self.read_planner_profile(expected_profile)
            require(not record['profile_readback']['mismatches'], 'Native planner profile differs from YAML')
            before = self.snapshots()
            record['before'] = before
            initial = before['expansion_status']
            require(initial.get('timer_enabled') is True and initial.get('budget_mode') == 'time'
                    and initial.get('timer_clock') == 'ros'
                    and initial.get('compute_budget_clock') == 'steady',
                    'Native time-budget ROS timer is not enabled')
            require(initial.get('local_triggered') is False and initial['updates'] == 0,
                    'Background expansion was qualified before the first live local graph')
            path_start, metrics_start = len(self.paths), len(self.local_metrics)
            record['command_path_count_before'] = path_start
            record['captures']['before_request'] = self.capture_background('before_request')
            request = PlannerSrv.Request()
            request.header.frame_id, request.header.stamp = 'map', self.get_clock().now().to_msg()
            request.bound_mode = 3
            # All seven pose scalars stay zero, including quaternion w. This
            # is the native live-odometry sentinel, not a hypothetical root.
            request.root_pose.orientation.w = 0.
            record['request'] = message_to_ordereddict(request)
            record['request_started'] = self.timed({})
            response = self.call(self.local_planner, request, 60.)
            record['response_received'] = self.timed({})
            record['response'] = message_to_ordereddict(response)
            self.until(lambda: len(self.local_metrics) > metrics_start, 'Live local planning metrics')
            metric = copy.deepcopy(self.local_metrics[metrics_start])
            record['planning_metrics'] = metric
            # Preserve the result's map even when graph building or gain
            # evaluation produced no usable route; path success is not a gate.
            revision = metric.get('map_revision')
            require(isinstance(revision, int), 'Local planning metrics lack an accepted-map revision')
            # A long native request may outlive the two-map observer window.
            # Save the actual latest map and declare the missing association.
            record['captures']['local_result'] = self.capture_background('local_result', revision)
            require(metric.get('graph_build_completed') is True, 'Live local graph build did not complete')
            current = self.state('expansion_status')
            record['after_local'] = self.timed(current)
            require(current.get('local_triggered') is True, 'Completed live graph did not qualify background timer')
            period = float(current['timer_period_seconds'])
            require(math.isfinite(period) and period > 0., 'Invalid native background timer period')
            # Read two distinct subsequent counter changes. Nothing calls the
            # explicit expansion service, and graph publication need not change.
            for index in range(1, 3):
                previous = current
                observation_deadline = min(deadline, time.monotonic()+max(15., period*20.))
                while True:
                    require(time.monotonic() < observation_deadline, 'Background timer did not advance twice')
                    self.spin_for(.2)
                    query_started = self.timed({})
                    current = self.state('expansion_status')
                    observed = self.timed({'query_started': query_started, **current})
                    record['timer_observations'].append(observed)
                    require(current.get('timer_enabled') is True and current.get('budget_mode') == 'time'
                            and current.get('local_triggered') is True and current.get('timer_clock') == 'ros'
                            and current.get('compute_budget_clock') == 'steady',
                            'Background timer qualification changed')
                    require(current['updates'] >= previous['updates']
                            and current['total_added'] >= previous['total_added'], 'Background counters regressed')
                    if current['updates'] > previous['updates']:
                        require(current['timer_ticks'] > previous['timer_ticks']
                                and current['last_timer_ros_ns'] > previous['last_timer_ros_ns'],
                                'Background update lacks a new native ROS timer tick')
                        observed['capture_label'] = f'timer_{index}'
                        record['captures'][f'timer_{index}'] = self.capture_background(f'timer_{index}')
                        break
            after = self.snapshots()
            record['after'] = after
            record['command_path_count_after'] = len(self.paths)
            record['updates_observed'] = current['updates']-record['after_local']['updates']
            record['total_added'] = after['expansion_status']['total_added']
            record['added_during_observation'] = record['total_added']-initial['total_added']
            record['graph_growth_observed'] = record['added_during_observation'] > 0
            record['local_metrics_count'] = len(self.local_metrics)-metrics_start
            self.action_counts(before, after, 0)
            require(len(self.paths) == path_start, 'Background observation emitted a command_path')
            require(record['local_metrics_count'] == 1, 'Unexpected additional local planning request')
            record['checks'] = {'profile_loaded': True, 'live_graph_build_completed': True,
                'two_native_timer_updates': True, 'no_command_path': True, 'no_action_requests': True}
            record['status'] = 'passed'
            self.phase = 'preflight'

        def settle(self):
            def stopped():
                return self.freshness()['fresh'] and self.speed < .03 and abs(self.yaw_rate) < .05
            self.until(stopped, 'Physical stop', 8.)
            before = pose_record(self.current)
            sim_end = self.get_clock().now().nanoseconds+350_000_000
            self.until(lambda: self.get_clock().now().nanoseconds >= sim_end, 'Stopped observation', 8.)
            require(stopped() and math.dist(before[:3], pose_record(self.current)[:3]) < .015,
                    'Native vehicle did not remain stopped')

        def validate(self):
            request = ValidatePath.Request()
            request.header.frame_id = 'map'
            request.header.stamp = self.get_clock().now().to_msg()
            request.bound_mode, request.path = 3, [copy.deepcopy(self.current)]
            request.first_pose_is_observation = True
            response = self.call(self.validator, request)
            if not response.valid:
                self.capture_failure_map(response.map_revision)
            return {'valid': response.valid, 'reason': response.reason, 'map_revision': response.map_revision}

        def capture_failure_map(self, revision):
            if (self.failure_map_request is not None
                    and self.failure_map_request['requested_revision'] == revision):
                self.match_failure_map()
                return
            self.failure_map_revision = revision
            self.failure_map = None
            self.failure_map_request = {'requested_revision': revision, 'pending': True,
                'requested_at': self.timed({}), 'map_stamp_match': False,
                'exact_revision_match': False}
            self.match_failure_map()

        def mapping_leg(self, distance):
            origin = pose_record(self.current)
            sign = 1. if distance > 0 else -1.
            command = Twist()
            command.linear.x = sign*.2
            def traveled():
                pose = pose_record(self.current)
                return sign*((pose[0]-origin[0])*math.cos(origin[3])+(pose[1]-origin[1])*math.sin(origin[3]))
            end = min(deadline, time.monotonic()+30.)
            try:
                while traveled() < abs(distance):
                    require(time.monotonic() < end, 'Supervised mapping leg timed out')
                    freshness = self.freshness()
                    if not freshness['command_fresh']:
                        # This is supervised initialization, before any PCI
                        # request. Stop on stale observations, wait for fresh
                        # measured rest, then resume this same audited leg.
                        # The planner/executor watchdogs are not modified.
                        pause = self.timed({'freshness': freshness, 'resumed': False})
                        summary.setdefault('bootstrap_observation_pauses', []).append(pause)
                        self.teleop.publish(Twist())
                        self.until(lambda: self.freshness()['command_fresh'] and self.speed < .03,
                                   'Fresh stopped odometry after supervised pause', 5.)
                        pause.update(resumed=True, resumed_at=self.timed({}),
                                     pose=pose_record(self.current))
                        continue
                    pose = pose_record(self.current)
                    require(abs(wrap(pose[3])) <= .08, 'Initialization heading left audited corridor')
                    command.angular.z = max(-.15, min(.15, -2.*wrap(pose[3])-sign*pose[1]))
                    self.teleop.publish(command)
                    self.spin_for(.04)
            finally:
                self.teleop.publish(Twist())
            self.settle()
            return {'requested_signed_distance_m': distance, 'start': origin,
                    'end': pose_record(self.current), 'measured_progress_m': traveled()}

        def mapping_observation(self):
            return self.timed({'pose': pose_record(self.current), 'odom_stamp_ns': self.odom_stamp,
                'map_revision': self.update.get('map_revision'),
                'map_acquisition_stamp_ns': int(self.update.get('stamp_sec', 0))*1_000_000_000+
                                            int(self.update.get('stamp_nanosec', 0))})

        def mapping_fresh(self):
            freshness = self.freshness()
            if freshness['command_fresh']:
                return True
            pause = self.timed({'freshness': freshness, 'resumed': False})
            summary.setdefault('bootstrap_observation_pauses', []).append(pause)
            self.teleop.publish(Twist())
            self.until(lambda: self.freshness()['command_fresh'] and self.speed < .03 and abs(self.yaw_rate) < .05,
                       'Fresh stopped odometry after supervised pause', 5.)
            pause.update(resumed=True, resumed_at=self.mapping_observation())
            return False

        def finish_mapping_record(self, record, trace_start, maps_start):
            record['end'] = self.mapping_observation()
            poses = [record['start']['pose']] + [row['pose'] for row in self.trace[trace_start:]]
            record.update(measured_distance_m=sum(math.dist(a[:2], b[:2]) for a, b in zip(poses, poses[1:])),
                measured_signed_turn_rad=sum(wrap(b[3]-a[3]) for a, b in zip(poses, poses[1:])),
                measured_absolute_turn_rad=sum(abs(wrap(b[3]-a[3])) for a, b in zip(poses, poses[1:])),
                odometry_history_indices=[trace_start, len(self.trace)],
                map_update_history_indices=[maps_start, len(self.map_updates)],
                observed_map_revisions=list(dict.fromkeys(
                    row['map_revision'] for row in self.map_updates[maps_start:] if 'map_revision' in row)))

        def mapping_turn_step(self, heading, anchor, record):
            self.mapping_segment = (anchor, anchor)
            record.update(status='failed', start=self.mapping_observation(), target_yaw_rad=heading,
                requested_turn_rad=wrap(heading-pose_record(self.current)[3]),
                maximum_angular_speed_rad_s=.15, timeout_s=30., timeout_clock='wall',
                stop_requested=False, exit_reason='in_progress')
            trace_start, maps_start = len(self.trace), len(self.map_updates)
            end = min(deadline, time.monotonic()+30.)
            record['deadline_time_s'] = end-started
            try:
                try:
                    while True:
                        require(time.monotonic() < end, 'Supervised waypoint turn timed out')
                        if not self.mapping_fresh():
                            continue
                        error = wrap(heading-pose_record(self.current)[3])
                        if abs(error) <= .04:
                            break
                        command = Twist()
                        command.angular.z = max(-.15, min(.15, 1.5*error))
                        self.teleop.publish(command)
                        self.spin_for(.04)
                finally:
                    self.teleop.publish(Twist())
                    record['stop_requested'] = True
                self.settle()
                record['final_yaw_error_rad'] = abs(wrap(heading-pose_record(self.current)[3]))
                require(record['final_yaw_error_rad'] <= .08, 'Supervised waypoint turn missed heading')
                record.update(status='passed', settled=True, exit_reason='heading_reached_and_stopped')
            except Exception as error:
                record['exit_reason'] = f'{type(error).__name__}: {error}'
                raise
            finally:
                record['final_yaw_error_rad'] = abs(wrap(heading-pose_record(self.current)[3]))
                self.finish_mapping_record(record, trace_start, maps_start)

        def mapping_turn(self, heading, anchor, record):
            origin = self.mapping_observation()
            change = wrap(heading-origin['pose'][3])
            count = max(1, math.ceil(abs(change)/(math.pi/2)))
            # Freeze the entire list before motion. Progress and observation
            # pauses cannot create another target or restart a step deadline.
            targets = [wrap(origin['pose'][3]+change*(i+1)/count) for i in range(count)]
            targets[-1] = wrap(heading)
            record.update(status='failed', start=origin, target_yaw_rad=heading,
                requested_turn_rad=change, maximum_angular_speed_rad_s=.15,
                maximum_planned_step_rad=math.pi/2, timeout_per_step_s=30.,
                fixed_step_targets_rad=targets, steps=[], stop_requested=False,
                exit_reason='in_progress')
            trace_start, maps_start = len(self.trace), len(self.map_updates)
            try:
                for index, target in enumerate(targets):
                    step = {'index': index}
                    record['steps'].append(step)
                    self.mapping_turn_step(target, anchor, step)
                require(abs(wrap(heading-pose_record(self.current)[3])) <= .08,
                        'Supervised waypoint turn missed final heading')
                record.update(status='passed', settled=True, exit_reason='all_fixed_steps_completed')
            except Exception as error:
                record['exit_reason'] = f'{type(error).__name__}: {error}'
                raise
            finally:
                self.teleop.publish(Twist())
                record['stop_requested'] = True
                record['final_yaw_error_rad'] = abs(wrap(heading-pose_record(self.current)[3]))
                self.finish_mapping_record(record, trace_start, maps_start)

        def mapping_waypoint(self, start, target, direction, index):
            heading = math.atan2(target[1]-start[1], target[0]-start[0])
            record = {'status': 'failed', 'direction': direction, 'index': index,
                      'nominal_start_xy_m': start, 'target_xy_m': target, 'turn': {}, 'drive': {}}
            # Append before motion so failures retain the partial leg evidence.
            summary['physical_initialization']['legs'].append(record)
            self.mapping_turn(heading, start, record['turn'])
            self.mapping_segment = (start, target)
            drive = record['drive']
            drive.update(status='failed', start=self.mapping_observation(), maximum_speed_m_s=.2,
                         maximum_angular_speed_rad_s=.15, timeout_s=30., stop_requested=False)
            trace_start, maps_start = len(self.trace), len(self.map_updates)
            end = min(deadline, time.monotonic()+30.)
            try:
                try:
                    while True:
                        require(time.monotonic() < end, 'Supervised waypoint drive timed out')
                        if not self.mapping_fresh():
                            continue
                        pose = pose_record(self.current)
                        along, cross, length = segment_coordinates(pose, start, target)
                        remaining = length-along
                        if remaining <= .02 and math.dist(pose[:2], target) <= .03:
                            break
                        require(remaining >= -.02, 'Supervised waypoint passed its target without settling')
                        require(abs(wrap(pose[3]-heading)) <= .08,
                                'Initialization heading left the audited segment')
                        # Correct cross-track error while retaining the original
                        # 0.08 rad heading envelope and 0.2/0.15 speed caps.
                        desired = heading-max(-.06, min(.06, cross/.5))
                        command = Twist()
                        command.linear.x = min(.2, max(.03, .8*remaining))
                        command.angular.z = max(-.15, min(.15, 2.*wrap(desired-pose[3])))
                        self.teleop.publish(command)
                        self.spin_for(.04)
                finally:
                    self.teleop.publish(Twist())
                    drive['stop_requested'] = True
                self.settle()
                endpoint = pose_record(self.current)
                drive.update(endpoint_xy_error_m=math.dist(endpoint[:2], target),
                             endpoint_yaw_error_rad=abs(wrap(endpoint[3]-heading)))
                require(drive['endpoint_xy_error_m'] <= .04 and drive['endpoint_yaw_error_rad'] <= .08,
                        'Supervised waypoint endpoint or heading missed')
                drive.update(status='passed', settled=True)
                record['status'] = 'passed'
            finally:
                endpoint = pose_record(self.current)
                drive.update(endpoint_xy_error_m=math.dist(endpoint[:2], target),
                             endpoint_yaw_error_rad=abs(wrap(endpoint[3]-heading)))
                self.finish_mapping_record(drive, trace_start, maps_start)

        def mapping_polyline(self, points):
            home = [.5, 0.]
            outward = [home, *points]
            previous = [0., 0.]
            for index, target in enumerate(outward):
                self.mapping_waypoint(previous, target, 'outbound', index)
                previous = target
            for index, target in enumerate(reversed(outward[:-1])):
                self.mapping_waypoint(previous, target, 'return', index)
                previous = target
            record = summary['physical_initialization']
            record['restore_home_heading'] = {}
            self.mapping_turn(0., home, record['restore_home_heading'])
            endpoint = pose_record(self.current)
            record.update(final_xy_error_m=math.dist(endpoint[:2], home),
                          final_yaw_error_rad=abs(wrap(endpoint[3])),
                          measured_distance_m=sum(leg[part]['measured_distance_m']
                              for leg in record['legs'] for part in ('turn', 'drive'))+
                              record['restore_home_heading']['measured_distance_m'])
            require(record['final_xy_error_m'] <= .04 and record['final_yaw_error_rad'] <= .08,
                    'Supervised polyline did not return to its home pose')

        @staticmethod
        def goal_key(value):
            if isinstance(value, (list, tuple)):
                require(len(value) == 16 and all(type(v) is int and 0 <= v <= 255 for v in value),
                        'Invalid Action UUID bytes')
                return bytes(value).hex()
            require(isinstance(value, str) and re.fullmatch(r'[0-9a-fA-F]{32}', value) is not None,
                    'Missing or invalid Action UUID')
            return value.lower()

        def collect_automatic_actions(self, paths, events, baseline):
            # Bind publication intervals to real accepted input UUIDs. Then
            # join both terminal layers by UUID, never by success ordinal.
            active, terminal, tracking = {}, {}, {}
            for event in events:
                if event['layer'] == 'execution':
                    if event.get('active') and event.get('goal_id') is not None:
                        key = (self.goal_key(event['goal_id']), event['generation'])
                        if event['accepted'] > baseline['execution_state']['accepted']:
                            active.setdefault(key, event)
                    outcome = event.get('terminal_outcome')
                    if outcome:
                        terminal[(self.goal_key(outcome['goal_id']), outcome['generation'])] = event
                elif event['layer'] == 'tracking' and event.get('state') in {'SUCCEEDED', 'CANCELED', 'ABORTED'}:
                    tracking[self.goal_key(event['goal_id'])] = event
            actions, used = [], set()
            for index, path in enumerate(paths):
                limit = paths[index+1]['time_s'] if index+1 < len(paths) else math.inf
                item = {'index': index+1, 'commanded_path': path, 'association': 'pending'}
                matches = [(key, event) for key, event in active.items() if key not in used
                    and path['time_s']-.1 <= event['time_s'] < limit
                    and event.get('endpoint') is not None
                    and math.dist(event['endpoint'][:3], path['path'][-1][:3]) < 1.e-7]
                if len(matches) == 1:
                    key, accepted = matches[0];used.add(key)
                    item.update(goal_id=key[0], generation=key[1], association='input_uuid_and_endpoint',
                                execution_accepted=accepted)
                    modes = [e for e in events if e['layer'] == 'control'
                        and e.get('state') in {'SENDING', 'EXECUTING'}
                        and path['time_s']-.1 <= e['time_s'] < limit]
                    if modes:
                        item['mode'] = min(modes, key=lambda e:abs(e['time_s']-accepted['time_s']))['mode']
                    ended = terminal.get(key)
                    if ended is not None:
                        outcome = ended['terminal_outcome'];item['terminal_outcome'] = outcome
                        item['execution_terminal'] = ended
                        output_id = outcome.get('output_goal_id')
                        tracked = tracking.get(self.goal_key(output_id)) if output_id is not None else None
                        if tracked is not None:
                            item['tracking_terminal'] = tracked
                        if (outcome.get('success') is True and outcome.get('action_status') == 4
                                and outcome.get('output_status') == 4 and tracked is not None
                                and tracked.get('state') == 'SUCCEEDED'):
                            item.update(execution_success=ended, tracking_success=tracked)
                elif len(matches) > 1:
                    item.update(association='ambiguous', candidate_goal_ids=[key[0] for key, _ in matches])
                actions.append(item)
            return actions

        def action_counts(self, before, after, count, allow_recovery=False):
            if allow_recovery:
                require(self.phase == 'automatic_exploration', 'Recovery allowance outside automatic task')
                state = after['control_state']['path_invalidation_recovery']
                proofs, seen, outputs = [], set(), set()
                require(state['pending'] is None and state['consecutive_attempts'] == 0,
                        'Automatic mission ended with incomplete recovery')
                events = [e for e in self.events if e.get('phase') == 'automatic_exploration']
                tracker_cancelled = {self.goal_key(e['goal_id']): e for e in events
                    if e['layer'] == 'tracking' and e.get('state') == 'CANCELED'}
                for recovery in state['events']:
                    key = (self.goal_key(recovery['goal_id']), recovery['generation'])
                    require(key not in seen and key in self.terminal_outcomes,
                            'Recovery has duplicate or missing input terminal UUID')
                    seen.add(key);outcome = self.terminal_outcomes[key]['outcome']
                    require(outcome['action_status'] == 6 and outcome['success'] is False
                        and outcome['stop_confirmed'] is True and outcome['output_type'] == 'kAction'
                        and outcome['reason'] == recovery['reason']
                        and outcome['reason'] in {'path_invalidated:'+r for r in
                            ('path_not_free', 'terrain_support_missing', 'terrain_slope_exceeded', 'path_off_terrain')}
                        and outcome['failure_map_revision'] == recovery['failure_map_revision']
                        and outcome['failure_map_revision'] > outcome['validated_revision']
                        and outcome['stop_stamp_s'] == recovery['confirmed_stop_stamp_s']
                        and outcome['finished_ros_s'] >= outcome['stop_stamp_s']
                        and recovery.get('replan_map_revision', 0) > outcome['failure_map_revision']
                        and recovery.get('replan_map_stamp_s', -1) >= outcome['stop_stamp_s']
                        and recovery.get('replan_odom_stamp_s', -1) >= outcome['stop_stamp_s']
                        and recovery['bound_mode'] == 3,
                        'Recovery lacks matching stopped Action and post-stop observation evidence')
                    proof = {'goal_id': key[0], 'generation': key[1],
                             'recovery': recovery, 'terminal_outcome': outcome}
                    if outcome['dispatched']:
                        output = self.goal_key(outcome['output_goal_id'])
                        require(output not in outputs and output in tracker_cancelled
                            and outcome['output_status'] == 5
                            and outcome['confirmation'] == 'action_terminal_and_stationary_odometry',
                            'Recovered input abort lacks its own confirmed downstream cancellation')
                        outputs.add(output);tracked = tracker_cancelled[output]
                        require(not tracked['active'] and tracked['last_command'] == {'linear_x': 0, 'angular_z': 0},
                                'Recovered downstream Action did not terminate with zero command')
                        proof['tracking_cancelled'] = tracked
                    else:
                        require(outcome['confirmation'] == 'not_dispatched' and outcome['output_goal_id'] is None
                                and outcome['output_status'] is None, 'Undispatched recovery has downstream motion')
                    proofs.append(proof)
                require(state['attempts'] == len(proofs), 'Recovery history is incomplete or truncated')
                failures = {(self.goal_key(e['terminal_outcome']['goal_id']), e['terminal_outcome']['generation'])
                    for e in events if e['layer'] == 'execution' and e.get('terminal_outcome')
                    and e['terminal_outcome']['success'] is False
                    and e['accepted'] > before['execution_state']['accepted']}
                require(failures == seen, 'An automatic failed Action was not explicitly recovered')
                for name, extra, aborted, cancelled in (
                        ('execution_state', len(proofs), len(proofs), 0),
                        ('tracking_state', len(outputs), 0, len(outputs))):
                    require(after[name]['accepted'] == before[name]['accepted']+count+extra
                        and after[name]['succeeded'] == before[name]['succeeded']+count
                        and after[name]['aborted'] == before[name]['aborted']+aborted
                        and after[name]['cancelled'] == before[name]['cancelled']+cancelled
                        and after[name]['rejected'] == before[name]['rejected'] and not after[name]['active'],
                        'Native automatic totals contain an unmatched Action: '+name)
                require(after['execution_state']['output_type'] == 'kAction'
                    and after['execution_state']['action_commands'] == before['execution_state']['action_commands']+count+len(outputs)
                    and after['execution_state']['trajectory_commands'] == before['execution_state']['trajectory_commands'],
                    'Automatic execution bypassed downstream Actions')
                return proofs
            for name in ('execution_state', 'tracking_state'):
                require(all(after[name][k] == before[name][k]+count for k in ('accepted', 'succeeded'))
                    and all(after[name][k] == before[name][k] for k in ('rejected', 'aborted', 'cancelled'))
                    and not after[name]['active'], 'Native double-Action totals differ: '+name)
            require(after['execution_state']['output_type'] == 'kAction'
                and after['execution_state']['action_commands'] == before['execution_state']['action_commands']+count
                and after['execution_state']['trajectory_commands'] == before['execution_state']['trajectory_commands'],
                'Execution did not use downstream Actions')

    try:
        mesh = (root/'src/planner_gazebo_sim/collision/edgar_mine.triangles' if args.scene == 'edgar'
                else root/'src/planner_gazebo_sim/models/pittsburgh_mine/pittsburgh_mine.dae')
        if args.bootstrap:
            if args.scene == 'pittsburgh':
                from pittsburgh_mesh.audit_bootstrap import audit_bootstrap
                summary['bootstrap_geometry'] = audit_bootstrap(root, args.bootstrap_forward, body_z)
            elif args.bootstrap_waypoints is None:
                from check_edgar_bootstrap_geometry import audit_geometry
                summary['bootstrap_geometry'] = audit_geometry(mesh,
                    corridor=((-0.7, args.bootstrap_forward+.7), (-.75, .75)), body_z=body_z)
            else:
                summary['bootstrap_geometry'] = audit_waypoint_bootstrap(mesh, args.bootstrap_waypoints, body_z)
            require(summary['bootstrap_geometry']['status'] == 'passed', 'Original-mesh corridor audit failed')
        source_configs = {'exploration': (root/'src/gbplanner_smb/config/edgar_exploration.yaml'
                          if args.scene == 'edgar' else root/'src/gbplanner_gazebo/config/pittsburgh_exploration.yaml'),
                          'tracking': root/'src/gbplanner_tracking/config/smb.yaml',
                          'drive': root/'src/gbplanner_gazebo/config/controllers.yaml'}
        configurations = {name: yaml.safe_load(path.read_text()) for name, path in source_configs.items()}
        drive_readback = copy.deepcopy(configurations['drive']['smb_drive']['ros__parameters'])
        drive_readback['wheel_separation_multiplier'] = args.wheel_separation_multiplier
        mapper = copy.deepcopy(configurations['exploration']['voxblox_mapper']['ros__parameters'])
        planner = copy.deepcopy(configurations['exploration']['gbplanner_local']['ros__parameters'])
        profile = {}
        if args.planner_profile is not None:
            profile_document = yaml.safe_load(args.planner_profile.read_text())
            require(isinstance(profile_document, dict)
                    and isinstance(profile_document.get('gbplanner_local'), dict),
                    'Planner profile must contain gbplanner_local.ros__parameters')
            profile = profile_document['gbplanner_local'].get('ros__parameters')
            require(isinstance(profile, dict) and profile, 'Planner profile has no parameter mapping')
            planner.update(profile)
            source_configs['planner_profile'] = args.planner_profile
        # Match the actual launch order: exploration -> optional planner
        # overlay -> sensor overlay -> explicit launch overrides.
        if args.sensor_profile == 'upstream':
            source_configs['sensor_profile'] = root/'src/gbplanner_smb/config/upstream_sensors.yaml'
            sensors = yaml.safe_load(source_configs['sensor_profile'].read_text())
            planner.update(sensors['gbplanner_local']['ros__parameters'])
            mapper.update(sensors['voxblox_mapper']['ros__parameters'])
        launch_overrides = {'global.auto_home': False, 'use_sim_time': True,
                            'diagnostics.map_trace_file': str(map_trace_path)}
        launch_overrides['diagnostics.home_snapshot_directory'] = str(home_snapshot_dir) if home_snapshot_dir else ''
        if args.sensor_profile == 'compact':
            launch_overrides['sensor.smb_lidar.vertical_fov'] = 0.5235987755982988
        planner.update(launch_overrides)
        mapper.update(use_sim_time=True, update_mesh_every_n_sec=0.)
        mapper['diagnostics.snapshot_trace_file'] = str(snapshot_trace_path)
        mapper['diagnostics.voxel_capture_directory'] = str(voxel_capture_dir) if voxel_capture_dir else ''
        mapper['diagnostics.voxel_capture_log_all_changes'] = not args.trace_transitions_only
        if args.trace_voxels:
            mapper['diagnostics.voxel_capture_indices'] = [x for v in args.trace_voxels for x in v]
        # Parameters for a disabled named sensor are not declared by the
        # native planner; the effective sensor.names list owns those models.
        planner_readback = {name: value for name, value in planner.items()
            if not (name.startswith('sensor.') and len(name.split('.')) > 2 and
                    name.split('.')[1] not in planner.get('sensor.names', []))}
        mapper_readback = {name: value for name, value in mapper.items()
            if not (name.startswith('input.') and len(name.split('.')) > 2 and
                    name.split('.')[1] not in mapper.get('input_sensors', []))}
        if args.bootstrap_waypoints is not None:
            size, extension, offset = (planner[key] for key in
                ('robot.size', 'robot.size_extension', 'robot.center_offset'))
            radius = .5*math.hypot(size[0]+extension[0], size[1]+extension[1])+math.hypot(*offset[:2])
            require(radius+.1 <= 1., 'Configured robot exceeds the audited polyline envelope')
            require(.38+offset[2]-.5*(size[2]+extension[2]) >= .27-1.e-9 and
                    .44+offset[2]+.5*(size[2]+extension[2]) <= body_z[1]+1.e-9,
                    'Configured robot exceeds the audited body-height range')
        require(not mapper['clear_unknown_box_on_start'] and not mapper['esdf.clear_sphere_for_planning']
            and not mapper['use_freespace_pointcloud'] and not planner['planning.freespace_cloud_enable'],
            'Synthetic unknown-space clearing enabled')
        summary['configuration'] = {'max_cycles_override': max(1, args.auto_cycles),
            'ground_maneuvers': args.ground_maneuvers,
            'wheel_separation_multiplier': args.wheel_separation_multiplier,
            'effective_drive_parameters': drive_readback,
            'search_target_forward_m': .65, 'search_target_radius': planner['search.target_radius'],
            'tracking_desired_vel': configurations['tracking']['gbplanner_pure_pursuit']['ros__parameters']['desired_vel'],
            'planner_profile': {'file': str(args.planner_profile), 'parameters': profile,
                'sha256': hashlib.sha256(args.planner_profile.read_bytes()).hexdigest()}
                if args.planner_profile is not None else None,
            'sensor_profile': args.sensor_profile, 'bootstrap_body_z_m': list(body_z),
            'planner_launch_overrides': launch_overrides,
            'effective_planner_parameters': planner_readback, 'effective_mapper_parameters': mapper_readback,
            'source_sha256': {
            str(path.relative_to(root) if path.is_relative_to(root) else path):
                hashlib.sha256(path.read_bytes()).hexdigest()
            for path in [*source_configs.values(), mesh, Path(__file__),
                         root/'tools/edgar_native_exploration.py',
                         root/'tools/edgar_global_exploration.py',
                         root/'tools/fortress_homing.py',
                         root/'src/gbplanner_smb/scripts/velocity_mux_node.py']}}
        rclpy.init(args=[])
        node = Probe(collect_background=planner_readback.get('global.background_enable', False))
        log = log_path.open('w')
        command = ['ros2', 'launch', 'gbplanner_gazebo', args.scene+'_fortress.launch.py',
            'mapping:=true', 'planning:=true', 'auto_home:=false',
            'ground_maneuvers:='+str(args.ground_maneuvers).lower(),
            'sensor_profile:='+args.sensor_profile,
            f'contacts:={str(args.contacts).lower()}',
            f'lateral_slip_compliance:={args.lateral_slip_compliance}',
            f'wheel_separation_multiplier:={args.wheel_separation_multiplier:.17g}',
            'map_trace_file:='+str(map_trace_path),
            'snapshot_trace_file:='+str(snapshot_trace_path),
            'max_cycles:='+str(max(1, args.auto_cycles))]
        # Humble's launch CLI rejects name:= with an empty value. Omission
        # selects the launch file's empty default when capture is disabled.
        if home_snapshot_dir is not None:
            command.append('home_snapshot_directory:='+str(home_snapshot_dir))
        if args.planner_profile is not None:
            command.append('planner_profile:='+str(args.planner_profile))
        if args.renderer_prefix is not None:
            command.extend(['renderer:=project', 'renderer_prefix:='+str(args.renderer_prefix),
                            'gpu_rays_min_cubemap_size:='+str(args.gpu_rays_min_cubemap_size)])
        if voxel_capture_dir:
            command.extend(['voxel_capture_directory:='+str(voxel_capture_dir),
                            'voxel_capture_indices:='+json.dumps(mapper['diagnostics.voxel_capture_indices']),
                            'voxel_capture_log_all_changes:='+str(not args.trace_transitions_only).lower()])
        summary['launch_command'] = command
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=dict(os.environ))
        node.until(lambda: node.current is not None and node.gazebo and node.control.get('state') == 'IDLE'
            and len(node.accepted_stamps) >= 3 and len(node.cloud_stamps) >= 3
            and all(len(stamps) >= 3 for stamps in node.sensor_stamps.values())
            and node.global_graph is not None and node.state_history is not None
            and all(c.service_is_ready() for c in node.state_clients.values())
            and node.validator.service_is_ready() and node.pci.service_is_ready(), 'Full native startup', 40.)
        node.settle()
        initial = pose_record(node.current)
        require(max(map(abs, initial[:2])) < .03 and abs(initial[3]) < .03 and .38 < initial[2] < .44,
                'Unexpected native mine initial pose')
        if args.scene == 'pittsburgh':
            # A previous static-scene run does not establish support in this
            # newly created physical world. Use fresh native wheel contacts.
            diagnostics = node.gazebo.get('contact_diagnostics', {})
            parts = diagnostics.get('parts', {})
            require(diagnostics.get('enabled') is True, 'Pittsburgh contact source disabled')
            for part in ('lf', 'rf', 'lh', 'rh'):
                contact = parts.get(part, {})
                pairs = contact.get('last_pairs', [])
                age = (node.odom_stamp-contact.get('last_stamp_ns', 0))*1.e-9
                require(contact.get('contact_points', 0) > 0 and pairs and -.1 <= age <= .5 and
                        all(any('pittsburgh_mine' in name for name in pair) for pair in pairs),
                        'Missing fresh Pittsburgh wheel support: '+part)
            summary['initial_native_wheel_contacts'] = copy.deepcopy(diagnostics)
        summary['initial_states'] = node.snapshots()
        summary['initial_pose'] = initial
        summary['configuration_readback'] = {
            'planner': node.read_planner_profile(planner_readback),
            'mapper': node.read_planner_profile(mapper_readback, mapper=True),
            'drive': node.read_planner_profile(drive_readback, drive=True)}
        require(all(not readback['mismatches'] for readback in summary['configuration_readback'].values()),
                'Native effective planner/mapper/drive parameters differ from launch overlays')
        summary['initial_sensor_clouds'] = {name: {kind: describe_cloud(node.clouds[topic], topic)
            for kind, topic in pair.items()} for name, pair in sensor_topics.items()}
        if args.sensor_profile == 'compact':
            summary['initial_cloud'] = summary['initial_sensor_clouds']['smb_lidar']['adapted']
        summary['checks']['root_before_initialization'] = node.validate()
        if args.bootstrap:
            if args.bootstrap_waypoints is not None:
                node.mapping_segment = ([0., 0.], [.5, 0.])
            node.phase = 'supervised_mapping'
            record = {'autonomous': False, 'mesh_only_used_for_offline_corridor_audit': True,
                'method': ('Physical wheel commands, odometry waypoint turns/advances and retraced return'
                    if args.bootstrap_waypoints is not None else
                    'Physical wheel commands, measured-odometry stop and corridor heading hold'),
                'observation_association': 'Latest independently received odometry and accepted-map update; not an atomic snapshot',
                'injected_map_cells': 0, 'legs': [],
                'requested_return_xy_m': [args.bootstrap_return_x, 0.] if args.bootstrap_waypoints is None else [.5, 0.]}
            summary['physical_initialization'] = record
            if args.bootstrap_waypoints is None:
                for distance in (args.bootstrap_forward, args.bootstrap_return_x-args.bootstrap_forward):
                    record['legs'].append(node.mapping_leg(distance))
            else:
                node.mapping_polyline(args.bootstrap_waypoints)
            # Three republished revisions can still contain scans acquired
            # during the drive. Gate autonomy on a real post-stop observation.
            # This startup observer wait is separate from PCI's 30 s request
            # deadline and never changes a planning or execution watchdog.
            boundary = node.mapping_observation()
            observation = node.update.get('observation_revision', 0)
            minimum_stamp = node.get_clock().now().nanoseconds
            wait = record['post_stop_observation'] = {
                'status': 'waiting', 'boundary': boundary,
                'minimum_acquisition_stamp_ns': minimum_stamp,
                'initial_observation_revision': observation, 'wait_budget_s': 30.}
            def observed_after_stop():
                stamp = int(node.update.get('stamp_sec', 0))*1_000_000_000+int(node.update.get('stamp_nanosec', 0))
                return (node.update.get('map_revision', 0) > boundary['map_revision'] and
                        node.update.get('observation_revision', 0) > observation and
                        stamp >= minimum_stamp and node.odom_stamp >= minimum_stamp and
                        node.freshness()['fresh'])
            try:
                node.until(observed_after_stop, 'Map acquired after bootstrap stop', 30.)
                wait['status'] = 'passed'
            finally:
                wait['last_observation'] = node.mapping_observation()
                wait['elapsed_s'] = wait['last_observation']['time_s']-boundary['time_s']
                if wait['status'] == 'waiting': wait['status'] = 'failed'
            node.until(lambda: node.state('velocity_mux_state')['gate'] == 'waiting_for_goal', 'Teleop release', 8.)
            record['final_states'] = node.snapshots()
        node.phase = 'preflight'
        node.capture_background('before_home')
        summary['checks']['root_before_autonomy'] = root_check = node.validate()
        require(root_check['valid'], 'Observed TSDF rejects current robot body/support: '+str(root_check))
        # A rejected unknown startup is expected before supervised mapping. It
        # must not be labelled as the exact input of a later motion failure.
        node.failure_map = node.failure_map_revision = None
        node.failure_map_request = None
        # Supervised mapping is not part of the autonomous mission. Choose its
        # home here through the public PCI service, once current support is
        # observed, instead of seeding graph home halfway through that sweep.
        # PCI owns its unchanged 30 s operation deadline. Leave one second
        # for delivering that result rather than preempting it after 8 s.
        require(node.call(node.home_setter, PciSetHomingPos.Request(), 31.).success,
                'PCI refused to initialize mission home at the observed start')
        node.until(lambda: node.global_graph is not None and any(v.id == 0 for v in node.global_graph.vertices),
                   'Explicit mission home publication')
        summary['mission_home'] = {'set_after_supervised_mapping': True,
            'service': 'pci_set_homing_pos', 'physical_pose': pose_record(node.current),
            'graph_pose': pose_record(next(v for v in node.global_graph.vertices if v.id == 0).pose)}
        if args.observe_background:
            node.observe_background(planner_readback)
        if args.global_first:
            from edgar_global_exploration import run_global_frontier
            before = node.snapshots()
            run_global_frontier(node, summary, pose_record, require)
        else:
            before = node.snapshots()
            origin = pose_record(node.current)
            target = copy.deepcopy(node.current)
            target.position.x += .65*math.cos(origin[3])
            target.position.y += .65*math.sin(origin[3])
            stamped = PoseStamped()
            stamped.header.frame_id, stamped.header.stamp, stamped.pose = 'map', node.get_clock().now().to_msg(), target
            node.until(lambda: node.target.get_subscription_count() > 0, 'Search target subscription')
            node.target.publish(stamped)
            node.spin_for(.15)
            node.phase = 'autonomous_search'
            mission = {'start': origin, 'target': pose_record(target), 'before': before}
            summary['autonomous_search'] = mission
            path_start, event_start = len(node.paths), len(node.events)
            request = PciSearch.Request()
            request.header.frame_id, request.header.stamp = 'map', node.get_clock().now().to_msg()
            request.use_current_state, request.not_exe_path, request.bound_mode = True, False, 3
            summary['autonomous_requests'] += 1
            response = node.call(node.pci, request, 30.)
            mission['request_accepted'], mission['planner_path'] = response.success, [pose_record(p) for p in response.path]
            if not response.success and node.metrics:
                node.capture_failure_map(node.metrics[-1].get('map_revision'))
            require(response.success and len(response.path) >= 2, 'PCI search did not return a motion route')
            node.until(lambda: len(node.paths) > path_start, 'Native route publication')
            mission['commanded_path'] = node.paths[-1]
            node.until(lambda: any(e['layer'] == 'control' and e.get('state') in {'DONE', 'ERROR', 'STOPPED'}
                                  for e in node.events[event_start:]), 'PCI search completion', 50.)
            require(node.control.get('state') == 'DONE' and node.control.get('mode') == 'search',
                    'PCI search did not finish: '+str(node.control))
            node.settle()
            after = node.snapshots()
            end = pose_record(node.current)
            endpoint = mission['commanded_path']['path'][-1]
            mission.update(after=after, end=end, endpoint_xy_error_m=math.dist(end[:2], endpoint[:2]),
                endpoint_yaw_error_rad=abs(wrap(end[3]-endpoint[3])),
                displacement_m=math.dist(origin[:2], end[:2]), events=node.events[event_start:])
            node.action_counts(before, after, 1)
            if args.observe_background:
                require(len(node.paths) == path_start+1, 'Search-only phase emitted extra command_path messages')
                summary['checks']['search_only_no_extra_command_paths'] = True
            require(mission['endpoint_xy_error_m'] <= .04 and mission['endpoint_yaw_error_rad'] <= .1
                and mission['displacement_m'] > .2 and node.last_command == (0., 0.), 'Search endpoint or stop missed')
            summary['checks']['search_double_action_completed'] = True
            mission['status'] = 'passed'
        if args.homing_after_search:
            from fortress_homing import run_explicit_homing
            run_explicit_homing(node, summary, pose_record, require)
        if args.auto_cycles:
            # The existing helper observes native PCI policy without changing
            # planner-selected paths or the controller's own homing decision.
            from edgar_native_exploration import run_native_exploration
            try:
                run_native_exploration(node, summary, args.auto_cycles, 120., pose_record, require)
            finally:
                # The shared observer predates the Fortress adapter; retain
                # its measurement with the actual source's name in this run.
                failure = summary.get('automatic_freshness_failure', {})
                if 'last_smb_status_elapsed_s' in failure:
                    failure['last_gazebo_status_elapsed_s'] = failure.pop('last_smb_status_elapsed_s')
        final = node.snapshots()
        summary['final_states'] = final
        if 'expansion_status' in final:
            initial_background = summary['initial_states']['expansion_status']
            final_background = final['expansion_status']
            summary['background_runtime'] = {
                'source': 'Planner expansion_status counters; no explicit expand service calls',
                'timer_enabled': final_background['timer_enabled'],
                'local_triggered': final_background['local_triggered'],
                'updates_delta': final_background['updates']-initial_background['updates'],
                'vertices_added_delta': final_background['total_added']-initial_background['total_added'],
                'initial': initial_background, 'final': final_background,
                'scope': 'Whole observed run, including the optional nonexecuting local request; '
                         'counts exclude vertices inserted by foreground local/global requests'}
        summary['final_pose'] = pose_record(node.current)
        require(final['mapper_status']['freespace']['unknown_box_operations'] == 0, 'Unknown-space clearing occurred')
        require(final['mapper_status']['integrated_scans'] > before['mapper_status']['integrated_scans']
            and final['map_status']['map_revision'] > before['map_status']['map_revision'], 'Native observed map did not advance')
        matching = node.cloud_stamps & node.raw_stamps & node.accepted_stamps
        require(len(matching) >= 5, 'Scan acquisition stamps did not pass through both map stages')
        summary['sensor_pipeline'] = {}
        for name, pair in sensor_topics.items():
            shared = node.sensor_stamps[pair['raw']] & node.sensor_stamps[pair['adapted']]
            integrated = final['mapper_status']['input_sources'][name]
            require(len(shared) >= 5 and integrated['integrated_scans'] > 0,
                    'Native sensor raw/adapter/mapper observations missing: '+name)
            summary['sensor_pipeline'][name] = {'raw_adapted_matching_stamps': len(shared),
                'map_stage_matching_stamps': len(shared & node.raw_stamps & node.accepted_stamps),
                'integrated_source': integrated,
                'association': 'Acquisition stamp intersection only; simultaneous sensors may share a stamp'}
        motion_phase = 'autonomous_global' if args.global_first else 'autonomous_search'
        driven = {name for row in node.joints if row['phase'] == motion_phase
                  for name, v in zip(row['names'], row['velocity']) if abs(v) > .1}
        require({'LF_wheel_joint', 'RF_wheel_joint', 'LH_wheel_joint', 'RH_wheel_joint'} <= driven,
                'Autonomous motion lacks all four rotating wheel observations')
        summary['checks'].update(native_observation_pipeline=True, four_driven_wheels=True,
                                 matching_scan_tsdf_accepted_stamps=len(matching), unknown_box_operations=0)
        summary['status'] = 'passed'
    except Exception as exc:
        summary['error'] = f'{type(exc).__name__}: {exc}'
        import traceback
        summary['traceback'] = traceback.format_exc()
        if node is not None:
            summary['last_state'] = {'phase': node.phase, 'gazebo': node.gazebo, 'control': node.control,
                'execution': node.execution, 'tracking': node.tracking, 'map': node.update,
                'freshness': node.freshness()}
            if (args.observe_background and node.phase == 'background_observation'
                    and node.accepted_receipts and node.current is not None and node.global_graph is not None):
                try:
                    summary['background_observation']['failure_observations'] = node.capture_background('failure')
                except Exception as capture_error:
                    summary['background_observation']['failure_capture_error'] = (
                        f'{type(capture_error).__name__}: {capture_error}')
    finally:
        try:
            if node is not None:
                if rclpy.ok():
                    node.teleop.publish(Twist())
                wait_started = time.monotonic()
                wait_deadline = wait_started+4.
                while (node.failure_map_request is not None
                       and node.failure_map_request.get('pending') and rclpy.ok()
                       and time.monotonic() < wait_deadline):
                    node.match_failure_map()
                    if not node.failure_map_request.get('pending'):
                        break
                    rclpy.spin_once(node, timeout_sec=max(0., min(.05, wait_deadline-time.monotonic())))
                if node.failure_map_request is not None:
                    summary['failure_map_wait'] = {
                        'requested_revision': node.failure_map_request['requested_revision'],
                        'waited_s': time.monotonic()-wait_started,
                        'limit_s': 4., 'pending_at_timeout': node.failure_map_request.get('pending', False)}
                if node.accepted_receipts and node.current is not None and node.global_graph is not None and node.state_history is not None:
                    if node.local_metrics and 'local_result' not in summary['observation_captures']:
                        node.capture_background('after_local_requests', node.local_metrics[-1].get('map_revision'))
                    node.capture_background('final')
                history_path.write_text(json.dumps({'odometry': node.trace, 'commands': node.commands,
                    'controller_limited_commands': node.limited_commands,
                    'gazebo': node.states, 'joints': node.joints, 'events': node.events,
                    'map_updates': node.map_updates, 'paths': node.paths, 'search_metrics': node.metrics,
                    'planning_metrics': node.local_metrics, 'global_metrics': node.global_metrics,
                    'execution_terminal_outcomes': list(node.terminal_outcomes.values()),
                    'state_history_publications': node.state_history_publications}, indent=2)+'\n')
                summary['history_file'] = str(history_path)
                summary['contact_observations'] = node.gazebo.get('contact_diagnostics', {})
                summary['wheel_contact_model'] = node.gazebo.get('wheel_contact_model', {})
                summary['latest_clouds'] = {name: cloud_metadata(m, name) for name, m in node.clouds.items()}
                if node.capacity_map_capture is not None:
                    message = node.capacity_map_capture
                    payload = bytes(message.data)
                    with capture_map_path.open('xb') as stream:
                        stream.write(payload)
                    summary['capacity_map_capture'] = {
                        **cloud_metadata(message, 'voxel_map'), 'data_file': str(capture_map_path),
                        'sha256': hashlib.sha256(payload).hexdigest(),
                        'selection': 'First raw mapper voxel_map at or above requested cell count',
                        'planner_acceptance_is_separate': True}
                if 'points' in node.clouds:
                    summary['final_cloud'] = describe_cloud(node.clouds['points'], 'points')
                summary['sensor_observations'] = {name: {kind: {
                    'topic': topic, 'messages': node.sensor_counts[topic],
                    'first_stamp_ns': min(node.sensor_stamps[topic]) if node.sensor_stamps[topic] else None,
                    'last_stamp_ns': max(node.sensor_stamps[topic]) if node.sensor_stamps[topic] else None,
                    'last_received': node.sensor_receipts.get(topic),
                    'latest_cloud': describe_cloud(node.clouds[topic], topic) if topic in node.clouds else None}
                    for kind, topic in pair.items()} for name, pair in sensor_topics.items()}
                if summary['status'] != 'passed' and 'accepted_voxel_map' in node.clouds:
                    failure_map = node.failure_map or node.clouds['accepted_voxel_map']
                    failure_map_request = node.failure_map_request or {}
                    requested_stamp = failure_map_request.get('map_update_stamp_ns')
                    selected_stamp = stamp_ns(failure_map)
                    requested_known = failure_map_request.get('known_cells')
                    map_path = args.output.with_name(args.output.stem+'_failure_map.bin')
                    map_path.write_bytes(bytes(failure_map.data))
                    summary['failure_map'] = {**cloud_metadata(failure_map, 'accepted_voxel_map'),
                        'data_file': str(map_path),
                        'requested_revision': failure_map_request.get(
                            'requested_revision', node.failure_map_revision),
                        'map_update_revision': failure_map_request.get('map_update_revision'),
                        'map_update_stamp_ns': requested_stamp,
                        'selected_cloud_stamp_ns': selected_stamp,
                        'map_stamp_match': requested_stamp is not None and selected_stamp == requested_stamp,
                        'revision_stamp_match': requested_stamp is not None and selected_stamp == requested_stamp,
                        'known_cells_at_requested_revision': requested_known,
                        'cloud_width': int(failure_map.width),
                        'known_cells_matches_cloud_width': (
                            requested_known is not None and int(failure_map.width) == requested_known),
                        'source_stamp_unique_as_observed': failure_map_request.get(
                            'source_stamp_unique_as_observed'),
                        'same_stamp_update_revisions': failure_map_request.get(
                            'same_stamp_update_revisions', []),
                        'matching_accepted_clouds_in_window': failure_map_request.get(
                            'matching_accepted_clouds_in_window', 0),
                        'exact_revision_match': False,
                        'association': 'Source acquisition stamps are not unique map revisions, especially with simultaneous sensors',
                        'fallback_note': (
                            'An accepted cloud matched the revision update stamp, but ValidatePath does not expose '
                            'an immutable map identity; exact revision is not proven.'
                            if node.failure_map is not None else
                            'No accepted cloud matched the requested revision stamp within the 4 s wait; '
                            'latest accepted map is retained as a non-exact fallback.')}
        except Exception as evidence_error:
            # Preserve the original failure and still stop the native process
            # tree when evidence serialization or disk writes fail.
            summary['status'] = 'failed'
            summary['evidence_error'] = f'{type(evidence_error).__name__}: {evidence_error}'
            summary.setdefault('error', summary['evidence_error'])
        if process is not None:
            try:
                # Signal launch only: it owns child signal delivery. Sending
                # to the group duplicates SIGINT in native C++ executables.
                process.send_signal(signal.SIGINT)
                process.wait(timeout=7.)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass
        summary['cleanup'] = stop_process_tree(process)
        if log:
            log.close()
        summary['status_before_cleanup_check'] = summary['status']
        failures, escalations = shutdown_log_events(log_path.read_text() if log_path.exists() else '')
        summary['node_exit_failures'] = failures
        summary['cleanup']['launch_signal_escalations'] = escalations
        summary['checks']['clean_shutdown'] = (not failures and not escalations
            and not summary['cleanup'].get('forced_kill', False)
            and summary['cleanup'].get('launch_returncode') == 0)
        if not summary['checks']['clean_shutdown']:
            summary['status'] = 'failed'
            summary.setdefault('error', 'Native process did not exit cleanly')
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        summary['wall_elapsed_s'] = time.monotonic()-started
        summary['log_file'] = str(log_path)
        args.output.write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
        print(json.dumps({'status': summary['status'], 'output': str(args.output), 'error': summary.get('error')}))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

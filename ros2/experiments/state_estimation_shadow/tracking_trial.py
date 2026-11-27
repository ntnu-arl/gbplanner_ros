"""Run the unchanged SMB tracker against one isolated odometry source.

The caller supplies /shadow/tracking_input, spins its ROS node and owns the
process lifetime. This helper neither transforms odometry nor publishes motor
commands. In particular, the caller must stop its open-loop command publisher
before dispatching the tracking action.
"""
from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import time

SHARP_PATH = [[0.0, 0.0], [1.2, 0.0], [2.4, 1.0], [2.4, 2.2], [3.5, 2.2]]
GENTLE_PATH = [[0.0, 0.0], [1.2, 0.0], [2.4, 0.4], [3.2, 0.9], [4.2, 1.2]]
HELDOUT_PATH = [[0.0, 0.0], [1.0, 0.0], [2.2, -0.45], [3.2, -1.0],
                [4.6, -1.0], [5.4, -0.6]]
VALIDATION_PATH = [[0.0, 0.0], [0.9, 0.0], [1.9, 0.6], [2.9, 0.6],
                   [4.0, -0.1], [5.1, 0.3]]
ROUTE_PROFILES = {'gentle': GENTLE_PATH, 'heldout': HELDOUT_PATH,
                  'validation': VALIDATION_PATH}
# These candidates are frozen before the follow-up runs. The precise profile
# uses a one-second nominal lookahead and reserves more of the original B-point
# position tolerance for estimation error; it is not fitted to trial truth.
CENTER_ACCURACY_PROFILES = {
    'baseline': {'goal_reaching_threshold': 0.02, 'goal_yaw_threshold': 0.07},
    'precise': {'look_ahead_distance': 0.30,
                'goal_reaching_threshold': 0.01, 'goal_yaw_threshold': 0.03},
}
path_points = GENTLE_PATH
tolerance_xy = 0.04
tolerance_yaw = 0.1


def file_sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def sample_path(root, points, follower_params):
    """Use the production executor's ground-path interpolation unchanged.

    The downstream tracker normally receives these dense samples, rather than
    the planner's sparse vertices. The ground executor keeps assigned headings
    and takes the smaller linear/yaw fraction, retaining exact segment ends.
    """
    module_path = Path(root) / 'src/gbplanner_execution/scripts/trajectory_tools.py'
    spec = importlib.util.spec_from_file_location('shadow_trajectory_tools', module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    vertices = []
    for index, (x, y) in enumerate(points):
        yaw = (math.atan2(points[index + 1][1] - y, points[index + 1][0] - x)
               if index + 1 < len(points) else 0.0)
        vertices.append((x, y, 0.24, yaw))
    speed = float(follower_params['speed'])
    yaw_rate = float(follower_params['yaw_rate'])
    dt = float(follower_params['sample_period'])
    samples = module.interpolate(vertices, speed, yaw_rate, dt,
                                 int(follower_params.get('max_points', 20000)))
    # A source vertex appears at both adjoining segment boundaries; retain
    # that upstream representation and record the first occurrence of each.
    vertex_indices = []
    for vertex in vertices:
        index = next(i for i, sample in enumerate(samples)
                     if sample[:3] == vertex[:3] and abs(module.wrap(sample[3] - vertex[3])) < 1e-12)
        vertex_indices.append(index)
    metadata = {
        'method': 'production trajectory_tools.interpolate; ground/manual path, no yaw reallocation',
        'module': str(module_path),
        'module_sha256': file_sha256(module_path),
        'speed_mps': speed,
        'yaw_rate_rad_s': yaw_rate,
        'sample_period_s': dt,
        'maximum_xyz_step_m': max(math.dist(a[:3], b[:3]) for a, b in zip(samples, samples[1:])),
        'retained_vertex_indices': vertex_indices,
        'retains_exact_endpoints_and_duplicate_segment_starts': True,
        'submitted_path_sha256': hashlib.sha256(json.dumps(samples).encode()).hexdigest(),
    }
    return samples, metadata


class TrackingTrial:
    """A single fixed PathFollower goal using the production pure-pursuit node."""

    path_points = path_points
    tolerance_xy = tolerance_xy
    tolerance_yaw = tolerance_yaw

    def __init__(self, node, out, root, source, launch_process_callback,
                 control_point='base', offset_m=0.2159,
                 accuracy_profile='baseline', route_profile='gentle', path_profile='legacy',
                 command_topic='/cmd_vel'):
        if control_point not in ('base', 'center'):
            raise ValueError('control_point must be base or center')
        if accuracy_profile not in CENTER_ACCURACY_PROFILES:
            raise ValueError('accuracy_profile must be baseline or precise')
        if accuracy_profile == 'precise' and control_point != 'center':
            raise ValueError('The precise profile requires center control')
        if route_profile not in ROUTE_PROFILES:
            raise ValueError('Unknown route_profile')
        if path_profile not in ('legacy', 'kinematic'):
            raise ValueError('path_profile must be legacy or kinematic')
        if path_profile == 'kinematic' and control_point != 'center':
            raise ValueError('Kinematic reference headings require center control')
        if not math.isfinite(offset_m) or offset_m < 0.0:
            raise ValueError('offset_m must be finite and nonnegative')
        self.node = node
        self.out = Path(out)
        self.root = Path(root)
        self.source = str(source)
        self.control_point = control_point
        self.offset_m = float(offset_m)
        self.accuracy_profile = accuracy_profile
        self.route_profile = route_profile
        self.path_profile = path_profile
        self.command_topic = command_topic
        self.launch_process = launch_process_callback
        self.path_points = copy.deepcopy(ROUTE_PROFILES[route_profile])
        self.process = None
        self.client = None
        self.state_client = None
        self.status_subscription = None
        self.goal_future = None
        self.goal_handle = None
        self.result_future = None
        self.reply = None
        self.accepted = None
        self.error = None
        self.feedback_messages = 0
        self.last_feedback = None
        self.last_tracking_state = None
        self.metadata = {}
        self.samples = []
        self.speed = 0.3

    def start(self):
        """Write only a trial configuration copy and start the existing binary."""
        if self.process is not None:
            raise RuntimeError('TrackingTrial.start() can only be called once')
        import yaml
        from ament_index_python.packages import get_package_prefix
        from planner_msgs.action import PathFollower
        from rclpy.action import ActionClient
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        source = self.root / 'src/gbplanner_tracking/config/smb.yaml'
        tracking_prefix = Path(get_package_prefix('gbplanner_tracking'))
        executable = tracking_prefix / 'lib/gbplanner_tracking/pure_pursuit_node'
        library = tracking_prefix / 'lib/libpure_pursuit.so'
        original = yaml.safe_load(source.read_text(encoding='utf-8'))
        self.samples, sampling = sample_path(
            self.root, self.path_points, original['gbplanner_path_follower']['ros__parameters'])
        if self.path_profile == 'kinematic':
            from consistent_path import consistent_base_poses
            # B follows the same position polyline. Recompute only intermediate
            # headings so its forward virtual point C has no lateral velocity.
            # The endpoint yaw is a boundary condition, not a fitted parameter.
            self.samples, consistency = consistent_base_poses(self.samples, self.offset_m)
            sampling['kinematic_heading'] = consistency
            sampling['submitted_path_sha256'] = hashlib.sha256(
                json.dumps(self.samples).encode()).hexdigest()
            sampling['retained_vertex_heading_policy'] = 'XYZ retained; intermediate yaw recomputed from kinematics'
        self.speed = sampling['speed_mps']
        config = {'gbplanner_pure_pursuit': copy.deepcopy(original['gbplanner_pure_pursuit'])}
        params = config['gbplanner_pure_pursuit']['ros__parameters']
        if (params['goal_reaching_threshold'] != tolerance_xy or
                params['goal_yaw_threshold'] != tolerance_yaw):
            raise ValueError('Production terminal tolerances changed; review the paired comparison')
        overrides = {'use_sim_time': True, 'fixed_frame': 'map', 'robot_frame': 'base_link'}
        reference_transform = {'method': 'identity; controller reference is base_link'}
        if self.control_point == 'center':
            # Convert every already-sampled B pose, not just sparse vertices:
            # C = B + R(yaw) [ell, 0]. This retains the desired base trajectory
            # and terminal heading while controlling the hypothesized no-slip
            # reference point. The caller must provide odometry at the same C.
            sampling['base_reference_sampled_path_sha256'] = sampling['submitted_path_sha256']
            self.samples = [(x + self.offset_m * math.cos(yaw),
                             y + self.offset_m * math.sin(yaw), z, yaw)
                            for x, y, z, yaw in self.samples]
            sampling['submitted_path_sha256'] = hashlib.sha256(
                json.dumps(self.samples).encode()).hexdigest()
            sampling['maximum_xyz_step_m'] = max(
                math.dist(a[:3], b[:3]) for a, b in zip(self.samples, self.samples[1:]))
            overrides.update(robot_frame='shadow_kinematic_center')
            overrides.update(CENTER_ACCURACY_PROFILES[self.accuracy_profile])
            center_xy = overrides['goal_reaching_threshold']
            center_yaw = overrides['goal_yaw_threshold']
            reference_transform = {
                'method': 'C = B + offset_m * [cos(yaw), sin(yaw)]; applied after production sampling',
                'offset_m': self.offset_m,
                'offset_status': 'fixed effective turning-center hypothesis, not a physical extrinsic correction',
                'original_base_vertices_retained_for_scoring': True,
                'first_controller_reference_xy_m': list(self.samples[0][:2]),
                'last_controller_reference_xy_m': list(self.samples[-1][:2]),
                'base_position_bound_at_controller_tolerances_m':
                    center_xy + 2.0 * self.offset_m * math.sin(center_yaw / 2.0),
                'tolerance_rationale': f'Declared before the run: {center_xy:g} m center error '
                    f'plus the lever-arm error at {center_yaw:g} rad must fit the original '
                    '0.04 m base tolerance; this geometric bound excludes state-estimation error.',
            }
        params.update(overrides)
        config_path = self.out / 'tracking.yaml'
        with config_path.open('x', encoding='utf-8') as stream:
            yaml.safe_dump(config, stream, sort_keys=False)

        self.client = ActionClient(self.node, PathFollower, 'pci_output_path')
        # Keep both request and response endpoints alive during tracker startup.
        # A freshly discovered service can precede FastDDS response matching.
        self.state_client = self.node.create_client(Trigger, 'tracking_state')
        self.status_subscription = self.node.create_subscription(
            String, 'tracking_status', self._status,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        command = [str(executable), '--ros-args', '--params-file', str(config_path),
                   '-r', 'odometry:=/shadow/tracking_input']
        if self.command_topic != '/cmd_vel':
            command.extend(['-r', 'cmd_vel:='+self.command_topic])
        self.metadata = {
            'odometry_source': self.source,
            'control_point': self.control_point,
            'accuracy_profile': self.accuracy_profile,
            'path_profile': self.path_profile,
            'accuracy_profile_selection': 'Preset before this follow-up; no evaluation-truth parameter fitting',
            'lookahead_rationale': ('0.30 m is one second at the nominal 0.30 m/s requested speed'
                                    if self.accuracy_profile == 'precise'
                                    else 'Existing production lookahead retained'),
            'tracker': 'production gbplanner_tracking/pure_pursuit_node',
            'configuration_source': str(source),
            'configuration_copy': str(config_path),
            'source_sha256': {
                'smb.yaml': file_sha256(source),
                'pure_pursuit_node': file_sha256(executable),
                'libpure_pursuit.so': file_sha256(library),
                'tracking_trial.py': file_sha256(__file__),
            },
            'overrides': overrides,
            'odometry_topic': '/shadow/tracking_input',
            'command_topic': self.command_topic,
            'route_profile': self.route_profile,
            'route_comparison_scope': f'Planar fixed-path comparison on the {self.route_profile} route; '
                'the earlier dense sharp-route truth baseline did not complete within 100 simulation seconds.',
            'previous_sharp_route': {'path_xy_m': SHARP_PATH, 'truth_baseline_completed': False,
                                     'reference_run': 'run4b_tracking_truth', 'time_limit_s': 100.0},
            'path_xy_m': self.path_points,
            'path_z_m': 0.24,
            'final_yaw_rad': 0.0,
            'vel_max_mps': self.speed,
            'submitted_waypoints': len(self.samples),
            'sampling': sampling,
            'reference_transform': reference_transform,
            'tolerance_xy_m': params['goal_reaching_threshold'],
            'tolerance_yaw_rad': params['goal_yaw_threshold'],
            'base_scoring_tolerance_xy_m': tolerance_xy,
            'base_scoring_tolerance_yaw_rad': tolerance_yaw,
            'scope': 'Fixed-path execution only; no planner or mapper',
        }
        (self.out / 'tracking_setup.json').write_text(
            json.dumps(self.metadata, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        self.process = self.launch_process('tracker', command)
        return self.process

    def ready(self):
        return self.client is not None and self.client.server_is_ready()

    def confirm_ready(self):
        """Observe the live tracker and command connection before dispatch."""
        from std_srvs.srv import Trigger
        client = self.state_client
        discovery_end = time.monotonic() + 5.
        connected = lambda: (client.service_is_ready() and
                             self.node.count_publishers(self.command_topic) > 0)
        self.node.spin_until(lambda: connected() or time.monotonic() >= discovery_end,
                             'tracking state and command discovery')
        if not connected():
            raise TimeoutError('Tracker readiness endpoints not discovered within 5 wall seconds')
        attempts = []
        response = None
        for index in range(3):
            started = time.monotonic()
            future = client.call_async(Trigger.Request())
            self.node.spin_until(lambda: future.done() or time.monotonic()-started >= 2.,
                                 'live tracker readiness response')
            attempts.append({'attempt': index+1, 'response_received': future.done(),
                             'wall_seconds': time.monotonic()-started})
            (self.out/'tracking_readiness_attempts.json').write_text(json.dumps(attempts,indent=2)+'\n')
            if future.done():
                response = future.result()
                break
            # This retry is exclusively a read-only state request. Never
            # resend a movement Action when its acceptance reply is missing.
            client.remove_pending_request(future)
            future.cancel()
        if response is None:
            raise TimeoutError('Tracker state response missing after three read-only requests')
        if not response.success:
            raise RuntimeError('Tracker readiness service failed')
        state = json.loads(response.message)
        # The installed tracker exposes its observation stamp and
        # steady-receipt age. Do not require newer source-only diagnostics.
        observation_stamp = state.get('odometry_stamp_s')
        source_age = (self.node.get_clock().now().nanoseconds*1.e-9 - observation_stamp
                      if observation_stamp is not None else None)
        ages = [source_age, state.get('odom_age')]
        if (state.get('active') or state.get('state') != 'IDLE' or
                any(v is None or not -.1 <= v <= .5 for v in ages)):
            raise RuntimeError('Tracker is not idle with fresh estimated odometry: '+response.message)
        state['command_publishers'] = self.node.count_publishers(self.command_topic)
        state['command_subscribers'] = self.node.count_subscribers(self.command_topic)
        state['readiness_source_age_s'] = source_age
        state['readiness_attempts'] = attempts
        if state['command_subscribers'] < 1:
            raise RuntimeError('Tracker command output has no consumer')
        (self.out/'tracking_ready.json').write_text(json.dumps(state,indent=2)+'\n')
        return state

    def send_goal(self):
        """Dispatch the fixed route; the caller must continue spinning its node."""
        if self.goal_future is not None:
            raise RuntimeError('This trial already has a tracking goal')
        if not self.ready():
            raise RuntimeError('Tracking action server is not ready')
        from geometry_msgs.msg import PoseStamped
        from planner_msgs.action import PathFollower

        goal = PathFollower.Goal()
        goal.manual_path = True
        goal.vel_max = self.speed
        for x, y, z, yaw in self.samples:
            point = PoseStamped()
            point.header.frame_id = 'map'
            point.header.stamp = self.node.get_clock().now().to_msg()
            point.pose.position.x = x
            point.pose.position.y = y
            point.pose.position.z = z
            point.pose.orientation.z = math.sin(yaw / 2.0)
            point.pose.orientation.w = math.cos(yaw / 2.0)
            goal.path.append(point)
        self.goal_future = self.client.send_goal_async(goal, feedback_callback=self._feedback)
        return self.goal_future

    def done(self):
        """Advance action-response bookkeeping without blocking or spinning."""
        if self.error is not None or self.accepted is False or self.reply is not None:
            return True
        if self.goal_future is None or not self.goal_future.done():
            return False
        try:
            if self.goal_handle is None:
                self.goal_handle = self.goal_future.result()
                self.accepted = bool(self.goal_handle.accepted)
                if not self.accepted:
                    return True
                self.result_future = self.goal_handle.get_result_async()
            if self.result_future.done():
                self.reply = self.result_future.result()
                return True
        except Exception as error:
            self.error = repr(error)
            return True
        return False

    def result(self):
        finished = self.done()
        return {
            'done': finished,
            'accepted': self.accepted,
            'action_status': int(self.reply.status) if self.reply is not None else None,
            'success': bool(self.reply.result.success) if self.reply is not None else False,
            'error': self.error,
            'feedback_messages': self.feedback_messages,
            'submitted_waypoints': len(self.samples),
            'last_feedback': self.last_feedback,
            'last_tracking_state': self.last_tracking_state,
        }

    def _feedback(self, message):
        feedback = message.feedback
        self.feedback_messages += 1
        self.last_feedback = {
            'remaining_waypoints': int(feedback.remaining_waypoints),
            'dist_to_goal': float(feedback.dist_to_goal),
            'estimated_time_remaining': float(feedback.estimated_time_remaining),
        }

    def _status(self, message):
        self.last_tracking_state = json.loads(message.data)

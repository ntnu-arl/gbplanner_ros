#!/usr/bin/env python3
"""Isolated follow-up: held-out route and native-controller comparisons.

Only an explicitly selected source drives the isolated tracking trial. Truth
is used for evaluation and for the declared truth-controller baseline only.
Production launch/configuration files are not modified.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback
from turning_center import TurningCenterAdapter

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PROFILES = {
    'asymmetric': [(4., .25, 0.), (5., .20, .30), (3., .24, 0.),
                   (2.5, .17, -.22), (3.5, .20, .16), (4., .25, 0.), (3., 0., 0.)],
    'tracking': [],
}


def stamp(message):
    return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec


def vec(v):
    return [v.x, v.y, v.z]


def odom_row(m):
    return {'stamp_ns': stamp(m), 'frame': m.header.frame_id, 'child': m.child_frame_id,
            'position': vec(m.pose.pose.position),
            'quaternion': vec(m.pose.pose.orientation) + [m.pose.pose.orientation.w],
            'linear': vec(m.twist.twist.linear), 'angular': vec(m.twist.twist.angular),
            'pose_covariance': list(m.pose.covariance), 'twist_covariance': list(m.twist.covariance)}


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if norm < .99 or norm > 1.01:
        raise ValueError('Unexpected sensor mounting quaternion')
    x, y, z, w = (v/norm for v in (x,y,z,w))
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def source_hashes():
    files = ['src/gbplanner_gazebo/launch/edgar_fortress.launch.py',
             'src/gbplanner_gazebo/config/controllers.yaml',
             'src/gbplanner_gazebo/config/smb_imu.yaml',
             'src/gbplanner_gazebo/scripts/fortress_state_node.py',
             'src/gbplanner_smb/launch/exploration.launch.py',
             'src/gbplanner_smb/config/exploration.yaml',
             'src/gbplanner_tracking/config/smb.yaml']
    return {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in files}


def filter_config(name, fused):
    wheel_mask = [False]*15
    for i in (6, 7, 11):
        wheel_mask[i] = True
    imu_mask = [False]*15
    imu_mask[11] = True
    cfg = {'use_sim_time': True, 'frequency': 50.0, 'sensor_timeout': .2,
           'two_d_mode': True, 'publish_tf': False, 'publish_acceleration': False,
           'print_diagnostics': False, 'debug': False, 'use_control': False,
           'transform_timeout': 0.0, 'transform_time_offset': 0.0,
           'map_frame': 'shadow_map', 'odom_frame': 'shadow_odom',
           'world_frame': 'shadow_odom', 'base_link_frame': 'base_link',
           'odom0': '/shadow/wheel_input', 'odom0_config': wheel_mask,
           'odom0_queue_size': 50, 'odom0_differential': False, 'odom0_relative': False,
           'permit_corrected_publication': False}
    if fused:
        cfg.update(imu0='/shadow/imu_input', imu0_config=imu_mask,
                   imu0_queue_size=100, imu0_differential=False, imu0_relative=False,
                   imu0_remove_gravitational_acceleration=False)
    return {name: {'ros__parameters': cfg}}


def stop_owned(process):
    result = {'pid': process.pid, 'escalated': False}
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            result['escalated'] = True
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
    result['returncode'] = process.returncode
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True, type=Path)
    parser.add_argument('--runtime', type=Path, default=Path('/tmp/gbplanner-estimation-runtime'))
    parser.add_argument('--wall-budget', type=float, default=180.)
    parser.add_argument('--profile', choices=PROFILES, default='asymmetric')
    parser.add_argument('--controller-source', choices=('open_loop', 'truth', 'ekf_fused', 'ekf_slip'), default='open_loop')
    parser.add_argument('--offset-m', type=float, default=.2159)
    parser.add_argument('--control-point', choices=('base', 'center'), default='base')
    parser.add_argument('--wheel-window', type=int, choices=(1, 10), default=10)
    parser.add_argument('--calibration-duration', type=float, default=3.)
    parser.add_argument('--accuracy-profile', choices=('baseline', 'precise'), default='baseline')
    parser.add_argument('--path-profile', choices=('legacy', 'kinematic'), default='legacy')
    parser.add_argument('--route-profile', choices=('gentle', 'heldout', 'validation'), default='gentle')
    parser.add_argument('--imu-seed', type=int, default=1)
    parser.add_argument('--mapping-trial', action='store_true',
                        help='Independent native dual-lidar mapping and real planner probe')
    parser.add_argument('--icp-shadow', action='store_true',
                        help='Evaluate a separate native ICP mapper on the same lidar/estimate streams; no control feedback')
    parser.add_argument('--execute-planned-path', action='store_true',
                        help='After estimated-map search, align, replan and execute with the existing tracker')
    parser.add_argument('--planned-target', nargs=2, type=float, default=[2.4, .4],
                        metavar=('X', 'Y'), help='Fixed map-coordinate goal for the independent planner trial')
    parser.add_argument('--additional-planned-target', nargs=2, type=float, action='append', default=[],
                        metavar=('X', 'Y'), help='Continue from rest with another real search and execution')
    parser.add_argument('--search-policy', choices=('baseline', 'tree_first_target', 'tree_first_target_refined'), default='baseline',
                        help='Independent search graph/resource policy; maximum budgets stay unchanged')
    parser.add_argument('--additional-path-geometry', choices=('original', 'center_dubins'), default='original',
                        help='Explicit experimental connector for additional goals only; original first-goal execution unchanged')
    parser.add_argument('--alignment-yaw-accel', type=float, default=0.,
                        help='Optional isolated preturn yaw slew limit in rad/s^2; zero disables the relay')
    parser.add_argument('--tracking-timeout', type=float, default=100.)
    args = parser.parse_args()
    if not 60 <= args.wall_budget <= 300:
        parser.error('wall budget must be between 60 and 300 seconds')
    if (args.profile == 'tracking') != (args.controller_source != 'open_loop'):
        parser.error('tracking profile requires an explicit truth/fused/slip controller source')
    if not 3. <= args.calibration_duration <= 15.:
        parser.error('calibration-duration must be between 3 and 15 seconds')
    if args.accuracy_profile == 'precise' and args.control_point != 'center':
        parser.error('precise profile is defined only at the center control point')
    if args.path_profile == 'kinematic' and args.control_point != 'center':
        parser.error('kinematic reference headings require center control')
    if not 0 <= args.imu_seed < 2**32:
        parser.error('imu-seed must be a uint32')
    if args.mapping_trial and (args.profile != 'tracking' or args.controller_source != 'ekf_slip'
                              or args.route_profile != 'gentle'):
        parser.error('mapping-trial requires ekf_slip tracking on the room-compatible gentle route')
    if args.icp_shadow and not args.mapping_trial:
        parser.error('icp-shadow requires the estimated mapping trial')
    if args.execute_planned_path and (not args.mapping_trial or args.control_point != 'center'
                                     or args.path_profile != 'kinematic'):
        parser.error('execute-planned-path requires mapping-trial, center control and kinematic reference')
    if not all(math.isfinite(v) for v in args.planned_target):
        parser.error('planned-target coordinates must be finite')
    if args.planned_target != [2.4, .4] and not args.mapping_trial:
        parser.error('planned-target requires mapping-trial')
    if args.search_policy != 'baseline' and not args.mapping_trial:
        parser.error('search-policy requires mapping-trial')
    if args.additional_planned_target and not args.execute_planned_path:
        parser.error('additional-planned-target requires execute-planned-path')
    if args.additional_path_geometry != 'original' and not args.additional_planned_target:
        parser.error('additional-path-geometry requires an additional planned target')
    if not all(math.isfinite(v) for target in args.additional_planned_target for v in target):
        parser.error('additional-planned-target coordinates must be finite')
    if not math.isfinite(args.alignment_yaw_accel) or args.alignment_yaw_accel < 0:
        parser.error('alignment-yaw-accel must be finite and nonnegative')
    if args.alignment_yaw_accel and not args.execute_planned_path:
        parser.error('alignment-yaw-accel is defined only for planned path execution')
    profile = PROFILES[args.profile]
    out = args.run_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    os.environ.setdefault('ROS_DOMAIN_ID', '168' if args.mapping_trial else '167')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['IGN_PARTITION'] = 'gbplanner_shadow_' + os.environ['ROS_DOMAIN_ID']
    os.environ['ROS_LOG_DIR'] = str(out/'ros_logs')
    runtime = args.runtime.resolve()
    prefix = runtime/'root/opt/ros/humble'
    ekf = prefix/'lib/robot_localization/ekf_node'
    if not ekf.exists():
        raise FileNotFoundError('Run prepare_runtime.sh first')
    env = dict(os.environ)
    env['LD_LIBRARY_PATH'] = str(prefix/'lib')+':'+str(runtime/'root/usr/lib/aarch64-linux-gnu')+':'+env.get('LD_LIBRARY_PATH','')
    env['AMENT_PREFIX_PATH'] = str(prefix)+':'+env.get('AMENT_PREFIX_PATH','')
    summary = {'status': 'running', 'scope': 'Isolated flat-ground sensor and native-controller comparison',
               'profile': args.profile, 'controller_source': args.controller_source,
               'control_point': args.control_point,
               'wheel_velocity_window': args.wheel_window,
               'calibration_duration_s': args.calibration_duration,
               'accuracy_profile': args.accuracy_profile, 'route_profile': args.route_profile,
               'path_profile': args.path_profile,
               'imu_seed': args.imu_seed,
               'turning_center_offset_m': args.offset_m,
               'offset_provenance': 'Frozen front-axle hypothesis selected after exploratory run2; not an extrinsic correction or online slip identification',
               'domain': os.environ['ROS_DOMAIN_ID'], 'physics_partition': os.environ['IGN_PARTITION'],
               'production_before': source_hashes(), 'runtime': json.loads((runtime/'manifest.json').read_text()),
               'source_sha256': {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.glob('*.py')},
               'ekf_publish_tf': False, 'planner_mapper_started': args.mapping_trial,
               'mapping_trial': args.mapping_trial, 'execute_planned_path': args.execute_planned_path,
               'icp_shadow_enabled': args.icp_shadow,
               'icp_feedback_to_controller_or_main_mapper': False,
               'planned_target_xy_m': args.planned_target,
               'additional_planned_targets_xy_m': args.additional_planned_target,
               'search_policy': args.search_policy,
               'additional_path_geometry': args.additional_path_geometry,
               'alignment_yaw_accel_rad_s2': args.alignment_yaw_accel,
               'tracker_started': args.profile == 'tracking',
               'truth_used_by_filter': False, 'truth_used_by_controller': args.controller_source == 'truth',
               'motion_profile': profile,
               'wheel_twist_variance_prior': [.0004,.01,.01,.01,.01,.04],
               'uncertainty_note': 'Declared engineering priors, not tuned on this trial. Both EKFs use the same wheel data and parameters; fused adds gyro only.',
               'limitations': ['Single flat-ground trial; not a full planning or mapping evaluation.',
                               'Both EKFs use a soft zero lateral-velocity observation; skid steering can violate this assumption.',
                               'Wheel yaw variance is deliberately conservative; filter priors have not been empirically calibrated.',
                               'Only XY and yaw are evaluated; no 3-D state-estimation claim.']}
    (out/'run.json').write_text(json.dumps(summary,indent=2))
    snapshot = out/'source_snapshot'
    snapshot.mkdir()
    for source in HERE.iterdir():
        if source.is_file() and source.suffix in ('.py', '.sh', '.sdf'):
            (snapshot/source.name).write_bytes(source.read_bytes())
    if args.mapping_trial:
        mapping_dir=HERE.parent/'estimated_mapping'
        sys.path.insert(0,str(mapping_dir))
        for source in mapping_dir.iterdir():
            if source.is_file() and source.suffix in ('.py','.sdf'):
                (snapshot/source.name).write_bytes(source.read_bytes())
                summary['source_sha256'][source.name]=hashlib.sha256(source.read_bytes()).hexdigest()
    events = (out/'events.jsonl').open('w', buffering=1)
    processes, logs = [], []
    started = time.monotonic()
    deadline = started + args.wall_budget - 20
    node = None
    initialized = False
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformListener

    def launch_process(label, command, process_env=None):
        log = (out/(label+'.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(command, env=process_env or os.environ.copy(), stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((label,process))
        return process

    class Recorder(Node):
        def __init__(self):
            super().__init__('shadow_trial_recorder', parameter_overrides=[Parameter('use_sim_time',value=True)])
            self.clock_ns = 0
            self.phase = 'startup'
            self.counts = {}
            self.latest = {}
            self.raw_frames = set()
            self.mount = None
            self.bias = None
            self.variance = None
            self.calibration = []
            self.calibration_wheel_max = 0.
            self.last_command_wall = 0.
            self.command = (0.,0.)
            self.relay_yaw = 0.
            self.relay_stamp_ns = None
            self.dynamic_tfs = set()
            self.shadow_tf_messages = 0
            self.input_enabled = False
            self.command_enabled = True
            self.tracking_origin = None
            self.mapping_stage = None
            self.icp_shadow = None
            self.center_adapter = TurningCenterAdapter(args.offset_m)
            self.slip_pub = self.create_publisher(Odometry, '/shadow/ekf_slip', 50)
            self.tracking_pub = self.create_publisher(Odometry, '/shadow/tracking_input', 50)
            self.create_subscription(Twist, '/cmd_vel', self.observe_command, 50)
            self.tf_buffer = Buffer(node=self)
            self.tf_listener = TransformListener(self.tf_buffer,self)
            self.velocity_pub = self.create_publisher(Twist,'/cmd_vel',10)
            if args.alignment_yaw_accel:
                self.create_subscription(Twist,'/shadow/tracker_command',self.relay_tracker_command,50)
            self.wheel_pub = self.create_publisher(Odometry,'/shadow/wheel_input',50)
            self.imu_pub = self.create_publisher(Imu,'/shadow/imu_input',100)
            self.create_subscription(Clock,'/clock',self.clock,qos_profile_sensor_data)
            self.create_subscription(Imu,'/os1_cloud_node/imu',self.imu,qos_profile_sensor_data)
            for topic,kind in [('/shadow/ground_truth','truth'),('/smb_drive/odom','wheel_raw'),
                               ('/shadow/ekf_wheel','ekf_wheel'),('/shadow/ekf_fused','ekf_fused')]:
                self.create_subscription(Odometry,topic,lambda m,k=kind:self.odom(m,k),qos_profile_sensor_data)
            self.create_subscription(TFMessage,'/tf',self.tf_observe,qos_profile_sensor_data)
            for name in ('ekf_wheel','ekf_fused'):
                self.create_subscription(TFMessage,'/shadow/'+name+'/tf',self.shadow_tf,qos_profile_sensor_data)

        def record(self,kind,**row):
            self.counts[kind] = self.counts.get(kind,0)+1
            events.write(json.dumps({'kind':kind,'receive_sim_ns':self.clock_ns,**row},allow_nan=False)+'\n')

        def clock(self,m):
            self.clock_ns=m.clock.sec*1000000000+m.clock.nanosec

        def tf_observe(self,m):
            for t in m.transforms:
                self.dynamic_tfs.add((t.header.frame_id,t.child_frame_id))

        def shadow_tf(self,m):
            if m.transforms:
                self.shadow_tf_messages += 1
                raise RuntimeError('publish_tf=false estimator emitted a TF message')

        def odom(self,m,kind):
            self.latest[kind]=m
            self.record(kind,**odom_row(m))
            if kind == 'ekf_fused':
                converted = self.center_adapter.convert(odom_row(m))
                candidate = copy.deepcopy(m)
                candidate.pose.pose.position.x, candidate.pose.pose.position.y, candidate.pose.pose.position.z = converted['position']
                candidate.twist.twist.linear.x, candidate.twist.twist.linear.y, candidate.twist.twist.linear.z = converted['linear']
                candidate.pose.covariance = converted['pose_covariance']
                candidate.twist.covariance = converted['twist_covariance']
                self.latest['ekf_slip'] = candidate
                self.record('ekf_slip', **odom_row(candidate))
                self.slip_pub.publish(candidate)
                self.publish_tracking_input(candidate, 'ekf_slip')
            self.publish_tracking_input(m, kind)
            if kind!='wheel_raw':
                return
            if m.child_frame_id != 'base_link':
                raise ValueError('Wheel odometry twist must be in base_link')
            if self.phase=='calibration':
                self.calibration_wheel_max=max(self.calibration_wheel_max,abs(m.twist.twist.linear.x),abs(m.twist.twist.angular.z))
            if self.input_enabled:
                obs=copy.deepcopy(m)
                # Both filters consume velocities only. v_y=0 is a soft planar
                # prior with variance .01, not a measured absence of slip.
                obs.twist.twist.linear.y=0.
                cov=np.zeros((6,6));np.fill_diagonal(cov,[.0004,.01,.01,.01,.01,.04])
                obs.twist.covariance=cov.reshape(-1).tolist()
                self.wheel_pub.publish(obs)
                self.record('wheel_input',**odom_row(obs))

        def imu(self,m):
            self.latest['imu_raw']=m
            self.raw_frames.add(m.header.frame_id)
            self.record('imu_raw',stamp_ns=stamp(m),frame=m.header.frame_id,angular=vec(m.angular_velocity),
                        angular_covariance=list(m.angular_velocity_covariance))
            if self.mount is None:
                try:
                    self.mount=rotation(self.tf_buffer.lookup_transform('base_link',m.header.frame_id,Time()).transform.rotation)
                except Exception:
                    return
            rate=self.mount@np.asarray(vec(m.angular_velocity))
            if self.phase=='calibration':
                self.calibration.append(rate)
            if not self.input_enabled:
                return
            obs=Imu()
            obs.header=m.header
            obs.header.frame_id='base_link'
            # Intentionally never read the simulator's orientation or exact bias.
            obs.orientation_covariance[0]=-1.
            obs.linear_acceleration_covariance[0]=-1.
            corrected=rate-self.bias
            obs.angular_velocity.x,obs.angular_velocity.y,obs.angular_velocity.z=map(float,corrected)
            obs.angular_velocity_covariance=np.diag(self.variance).reshape(-1).tolist()
            self.imu_pub.publish(obs)
            self.record('imu_input',stamp_ns=stamp(obs),frame=obs.header.frame_id,angular=vec(obs.angular_velocity),
                        angular_covariance=list(obs.angular_velocity_covariance))

        def observe_command(self, m):
            self.record('observed_command', stamp_ns=self.clock_ns,
                        linear_x=m.linear.x, angular_z=m.angular.z)

        def relay_tracker_command(self, message):
            # This candidate limits acceleration only during the new preturn.
            # A zero command bypasses the limiter so stop/cancel never coasts.
            # Outside this phase the original controller command is unchanged.
            before = message.angular.z
            limited = (self.phase == 'planned_alignment' and
                       (message.linear.x != 0. or before != 0.))
            forwarded = copy.deepcopy(message)
            if limited:
                dt = 0. if self.relay_stamp_ns is None else max(
                    0., min(.1, (self.clock_ns-self.relay_stamp_ns)*1.e-9))
                step = args.alignment_yaw_accel*dt
                forwarded.angular.z = max(self.relay_yaw-step, min(self.relay_yaw+step, before))
            self.relay_yaw = forwarded.angular.z
            self.relay_stamp_ns = self.clock_ns
            self.velocity_pub.publish(forwarded)
            self.record('tracker_command_relay', stamp_ns=self.clock_ns,
                        phase=self.phase, limit_active=limited,
                        input_linear_x=message.linear.x, input_angular_z=before,
                        output_linear_x=forwarded.linear.x, output_angular_z=forwarded.angular.z)

        def publish_tracking_input(self, m, kind):
            if args.profile != 'tracking' or kind != args.controller_source:
                return
            if kind != 'truth' and self.tracking_origin is None:
                return
            obs = copy.deepcopy(m)
            obs.header.frame_id = 'map'
            if kind != 'truth':
                # Declared starting map pose is (0,0,0), known from trial setup.
                # No simulator truth is read to initialize an estimator.
                x0, y0, yaw0 = self.tracking_origin
                x, y = m.pose.pose.position.x-x0, m.pose.pose.position.y-y0
                c, s = math.cos(yaw0), math.sin(yaw0)
                obs.pose.pose.position.x = c*x+s*y
                obs.pose.pose.position.y = -s*x+c*y
                obs.pose.pose.position.z = .24  # fixed level-ground geometry only
                q = m.pose.pose.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))-yaw0
                obs.pose.pose.orientation.x=0.; obs.pose.pose.orientation.y=0.
                obs.pose.pose.orientation.z=math.sin(yaw/2); obs.pose.pose.orientation.w=math.cos(yaw/2)
                jac = np.eye(6); jac[:2,:2] = [[c,s],[-s,c]]
                cov = np.array(m.pose.covariance).reshape(6,6)
                obs.pose.covariance = (jac@cov@jac.T).reshape(-1).tolist()
            if self.mapping_stage is not None:
                self.mapping_stage.publish_pose(obs)
                self.record('mapping_input', **odom_row(obs))
            if args.control_point == 'center':
                # Transform both pose and twist to C, the hypothesized point
                # with zero lateral velocity. TrackingTrial transforms the
                # desired B poses to this same reference point independently.
                q = obs.pose.pose.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                ell = args.offset_m
                obs.pose.pose.position.x += ell*math.cos(yaw)
                obs.pose.pose.position.y += ell*math.sin(yaw)
                obs.twist.twist.linear.y += ell*obs.twist.twist.angular.z
                pose_jac = np.eye(6)
                pose_jac[0,5] = -ell*math.sin(yaw)
                pose_jac[1,5] = ell*math.cos(yaw)
                twist_jac = np.eye(6); twist_jac[1,5] = ell
                pose_cov = np.array(obs.pose.covariance).reshape(6,6)
                twist_cov = np.array(obs.twist.covariance).reshape(6,6)
                obs.pose.covariance = (pose_jac@pose_cov@pose_jac.T).reshape(-1).tolist()
                obs.twist.covariance = (twist_jac@twist_cov@twist_jac.T).reshape(-1).tolist()
                obs.child_frame_id = 'shadow_kinematic_center'
            self.tracking_pub.publish(obs)
            self.record('tracking_input', **odom_row(obs))

        def set_phase(self,name):
            self.phase=name
            if name == 'planned_alignment':
                self.relay_yaw = 0.
                self.relay_stamp_ns = self.clock_ns
            self.record('phase',stamp_ns=self.clock_ns,name=name)
            print('Phase:',name,'sim=',self.clock_ns*1e-9,flush=True)

        def publish_command(self):
            m=Twist();m.linear.x,m.angular.z=self.command
            self.velocity_pub.publish(m)
            self.last_command_wall=time.monotonic()
            self.record('command',stamp_ns=self.clock_ns,linear_x=m.linear.x,angular_z=m.angular.z)

        def spin_until(self,predicate,label):
            while not predicate():
                if time.monotonic()>=deadline:
                    raise TimeoutError('Wall budget exhausted: '+label)
                for process_label,p in processes:
                    if p.poll() is not None:
                        raise RuntimeError(process_label+' exited early: '+str(p.returncode))
                rclpy.spin_once(self,timeout_sec=.01)
                if self.command_enabled and time.monotonic()-self.last_command_wall>.04:
                    self.publish_command()

        def duration(self,seconds):
            target=self.clock_ns+int(seconds*1e9)
            self.spin_until(lambda:self.clock_ns>=target,self.phase)

    try:
        rclpy.init(args=[]);initialized=True
        node=Recorder()
        sensor_launch=(HERE.parent/'estimated_mapping/mapping_trial.launch.py'
                       if args.mapping_trial else HERE/'sensor_trial.launch.py')
        launch_process('sensors',['ros2','launch',str(sensor_launch),
                                 'output_dir:='+str(out/'generated'),
                                 'velocity_window:='+str(args.wheel_window),
                                 'imu_seed:='+str(args.imu_seed)])
        node.spin_until(lambda: all(k in node.latest for k in ('truth','wheel_raw','imu_raw')) and node.mount is not None,
                        'wheel/IMU/truth and static mounting TF discovery')
        node.set_phase('settle');node.duration(2.)
        node.set_phase('calibration');node.duration(args.calibration_duration)
        samples=np.asarray(node.calibration)
        if len(samples)<200 or node.calibration_wheel_max>.02:
            raise RuntimeError('Insufficient stationary IMU calibration data')
        node.bias=np.mean(samples,axis=0)
        node.variance=np.maximum(np.var(samples,axis=0,ddof=1),2.5e-5)*(1+1/len(samples))
        summary['calibration']={'samples':len(samples),'measured_gyro_bias_body_rad_s':node.bias.tolist(),
                                'gyro_measurement_variance':node.variance.tolist(),
                                'wheel_max_abs_speed_or_rate':node.calibration_wheel_max,
                                'sensor_to_body_rotation':node.mount.tolist(),
                                'method':'Mean of stationary measured gyro; no simulator bias topic/service'}
        for name,fused in [('ekf_wheel',False),('ekf_fused',True)]:
            config=out/(name+'.yaml')
            config.write_text(yaml.safe_dump(filter_config(name,fused),sort_keys=False))
            launch_process(name,[str(ekf),'--ros-args','-r','__node:='+name,'--params-file',str(config),
                                 '-r','odometry/filtered:=/shadow/'+name,
                                 '-r','/tf:=/shadow/'+name+'/tf',
                                 '-r','/set_pose:=/shadow/'+name+'/set_pose'],env)
        node.input_enabled=True
        node.set_phase('warmup')
        node.spin_until(lambda:all(k in node.latest for k in ('ekf_wheel','ekf_fused')),'both shadow outputs')
        node.duration(2.)
        # Verify the live graph before motion: filters consume only the explicit
        # sensor inputs, publish no TF, and have no downstream planner consumer.
        summary['graph_before_motion']={
          'nodes':sorted(node.get_node_names()),
          'filter_subscriptions':{name:[{'topic':t,'types':types} for t,types in node.get_subscriber_names_and_types_by_node(name,'/')]
                                  for name in ('ekf_wheel','ekf_fused')},
          'filter_publishers':{name:[{'topic':t,'types':types} for t,types in node.get_publisher_names_and_types_by_node(name,'/')]
                               for name in ('ekf_wheel','ekf_fused')},
          'output_subscribers':{topic:[info.node_name for info in node.get_subscriptions_info_by_topic(topic)]
                                for topic in ('/shadow/ekf_wheel','/shadow/ekf_fused')}}
        for name in ('ekf_wheel','ekf_fused'):
            pubs=summary['graph_before_motion']['filter_publishers'][name]
            if any(p['topic'] in ('/tf','/cmd_vel','/odometry') for p in pubs):
                raise RuntimeError('Shadow filter has a production publisher')
        if args.profile == 'tracking':
            from tracking_trial import TrackingTrial
            reference = node.latest[args.controller_source]
            q = reference.pose.pose.orientation
            node.tracking_origin = (reference.pose.pose.position.x, reference.pose.pose.position.y,
                                    math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            summary['declared_initial_map_pose'] = [0., 0., 0.]
            summary['tracking_estimator_origin'] = list(node.tracking_origin) if args.controller_source != 'truth' else None
            if args.mapping_trial:
                from mapping_stage import MappingStage
                node.mapping_stage=MappingStage(node,out,ROOT,launch_process)
                node.publish_tracking_input(reference,'ekf_slip')
                node.duration(.3)
                node.mapping_stage.start()
                node.set_phase('mapping_bootstrap')
                node.mapping_stage.wait_initial_map()
                if args.icp_shadow:
                    from icp_shadow import IcpShadow
                    node.icp_shadow = IcpShadow(node, out, ROOT, launch_process)
                    node.icp_shadow.start()
                    node.set_phase('icp_shadow_bootstrap')
                    node.icp_shadow.wait_ready()
            trial = TrackingTrial(node, out, ROOT, args.controller_source, launch_process,
                                  control_point=args.control_point, offset_m=args.offset_m,
                                  accuracy_profile=args.accuracy_profile, route_profile=args.route_profile,
                                  path_profile=args.path_profile,
                                  command_topic='/shadow/tracker_command' if args.alignment_yaw_accel else '/cmd_vel')
            trial.start()
            summary.update(path_points=trial.path_points, path_target_xy=trial.path_points[-1],
                           path_frame_id='map', path_target_yaw=0.,
                           goal_tolerance_xy=.04, goal_tolerance_yaw=.1,
                           settled_tolerance_xy=.07, settled_tolerance_yaw=.15)
            node.spin_until(trial.ready, 'native path-tracking action server')
            node.duration(.5)
            if args.execute_planned_path:
                summary['tracker_ready'] = trial.confirm_ready()
            node.set_phase('motion')
            motion_start = node.clock_ns
            node.command_enabled = False  # tracker is the sole command publisher during motion
            trial.send_goal()
            node.spin_until(lambda: trial.goal_future.done() or node.clock_ns-motion_start > 5.e9,
                            'native path-tracking Action acceptance')
            if not trial.goal_future.done():
                summary['action_result'] = trial.result()
                raise TimeoutError('Native tracker Action acceptance response missing after 5 simulation seconds')
            node.spin_until(lambda: trial.done() or node.clock_ns-motion_start > args.tracking_timeout*1e9,
                            'native path-tracking action')
            if not trial.done():
                raise TimeoutError('Native tracker did not finish inside the simulation-time budget')
            summary['action_result'] = trial.result()
            summary['active_command_duration_s'] = (node.clock_ns-motion_start)*1e-9
            node.record('action_result', stamp_ns=node.clock_ns, result=summary['action_result'])
            node.command_enabled = True
        else:
            summary['active_command_duration_s'] = sum(seconds for seconds,v,w in profile if v != 0 or w != 0)
            node.set_phase('motion')
            for seconds,v,w in profile:
                node.command=(v,w);node.publish_command();node.duration(seconds)
        node.command=(0.,0.);node.publish_command()
        node.set_phase('stopped');node.duration(3.)
        # Preserve the original fixed-route scoring endpoint before any new
        # planned motion. Multi-stage analysis must not score the return goal
        # against the first outbound target or absorb it into a settling tail.
        summary['fixed_route_evaluation_end_ns'] = node.clock_ns
        if args.mapping_trial:
            from planner_probe import run_planner_probe
            node.set_phase('mapping_snapshot')
            summary['mapping']=node.mapping_stage.snapshot()
            node.set_phase('planning_probe')
            summary['planner_probe']=run_planner_probe(node,ROOT,out,launch_process,
                node.spin_until,lambda:node.mapping_stage.latest_odometry,
                target_xy=args.planned_target, search_policy=args.search_policy)
            summary['mapping_after_planning']=node.mapping_stage.status()
            summary['mapping_planning_acceptance']={
                'passed': bool(summary['planner_probe'].get('success')),
                'planner_status': summary['planner_probe'].get('status'),
                'reason': summary['planner_probe'].get('reason'),
                'scope': 'Estimated-pose mapping and real search/path validation; no planned-path execution'}
            if not summary['mapping_planning_acceptance']['passed']:
                raise RuntimeError('Mapping/planning stage did not pass: '+
                                   str(summary['mapping_planning_acceptance']['reason']))
            if args.execute_planned_path:
                if not (summary['action_result'].get('success') and
                        summary['action_result'].get('action_status') == 4):
                    raise RuntimeError('Fixed outbound action failed; do not continue with planned execution')
                from planned_execution import run_planned_execution
                summary['planned_execution'] = run_planned_execution(
                    node, ROOT, out, trial, summary['planner_probe'], launch_process,
                    args.offset_m, args.tracking_timeout)
                summary['mapping_after_execution'] = node.mapping_stage.status()
                if not summary['planned_execution'].get('success'):
                    raise RuntimeError('Planned execution did not pass: '+
                                       str(summary['planned_execution'].get('reason')))
                summary['mapping_after_execution_snapshot'] = node.mapping_stage.snapshot('after_execution')
                # Additional goals share the same live estimate, map and tracker.
                # Neither estimator origin nor map is reset between rounds.
                summary['additional_planned_executions'] = []
                for ordinal, target in enumerate(args.additional_planned_target, start=2):
                    round_dir = out/('round'+str(ordinal))
                    entry = {'ordinal': ordinal, 'target_xy_m': target,
                             'execution_relative_path': 'round'+str(ordinal)+'/planned_execution.json',
                             'start_sim_ns': node.clock_ns}
                    summary['additional_planned_executions'].append(entry)
                    node.set_phase('planned_round_'+str(ordinal)+'_search')
                    probe = run_planner_probe(node, ROOT, round_dir, launch_process,
                        node.spin_until, lambda: node.mapping_stage.latest_odometry,
                        start_planner=False, target_xy=target, search_policy=args.search_policy)
                    entry['planner_probe'] = probe
                    if not probe.get('success'):
                        raise RuntimeError('Round '+str(ordinal)+' search failed: '+str(probe.get('reason')))
                    entry['execution'] = run_planned_execution(
                        node, ROOT, round_dir, trial, probe, launch_process,
                        args.offset_m, args.tracking_timeout, geometry=args.additional_path_geometry)
                    if not entry['execution'].get('success'):
                        raise RuntimeError('Round '+str(ordinal)+' execution failed: '+
                                           str(entry['execution'].get('error')))
                    entry['end_sim_ns'] = node.clock_ns
                    entry['mapping_snapshot'] = node.mapping_stage.snapshot('after_round'+str(ordinal))
        if node.icp_shadow is not None:
            summary['icp_shadow_final_status'] = node.icp_shadow.finish()
        summary['status']='completed'
    except Exception as exc:
        summary['status']='failed'
        summary['error']=repr(exc)
        summary['traceback']=traceback.format_exc()
        print(summary['traceback'],flush=True)
    finally:
        if node is not None:
            node.command=(0.,0.)
            if rclpy.ok():
                node.publish_command()
            summary['counts']=node.counts
            summary['imu_frames']=sorted(node.raw_frames)
            summary['shadow_tf_messages']=node.shadow_tf_messages
            summary['observed_dynamic_tf_edges']=sorted([list(edge) for edge in node.dynamic_tfs])
            summary['last_sim_seconds']=node.clock_ns*1e-9
            if node.mapping_stage is not None:
                node.mapping_stage.save()
            if node.icp_shadow is not None:
                node.icp_shadow.save()
        summary['shutdown']={label:stop_owned(p) for label,p in reversed(processes)}
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        for log in logs:
            log.close()
        events.close()
        summary['wall_seconds']=time.monotonic()-started
        summary['production_after']=source_hashes()
        summary['production_files_unchanged']=summary['production_after']==summary['production_before']
        if not summary['production_files_unchanged']:
            summary['status']='failed'
        summary['events_sha256']=hashlib.sha256((out/'events.jsonl').read_bytes()).hexdigest()
        (out/'run.json').write_text(json.dumps(summary,indent=2,allow_nan=False))
        print(json.dumps({'status':summary['status'],'output':str(out),'counts':summary.get('counts'),
                          'production_files_unchanged':summary['production_files_unchanged']},indent=2),flush=True)
    return 0 if summary['status']=='completed' else 1

if __name__=='__main__':
    raise SystemExit(main())

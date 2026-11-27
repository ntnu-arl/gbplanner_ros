#!/usr/bin/env python3
"""Bounded, isolated sensor experiment. EKF outputs never drive the robot.

The same wheel observations feed both filters; only the fused filter receives
calibrated gyro samples. Simulator truth is recorded exclusively for scoring.
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
import time
import traceback

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PROFILE = [(4.0, .25, 0.0), (6.0, .20, .30), (3.0, .25, 0.0),
           (6.0, .20, -.30), (4.0, .25, 0.0), (3.0, 0.0, 0.0)]


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
    args = parser.parse_args()
    if not 60 <= args.wall_budget <= 300:
        parser.error('wall budget must be between 60 and 300 seconds')
    out = args.run_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    os.environ.setdefault('ROS_DOMAIN_ID', '167')
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
    summary = {'status': 'running', 'scope': 'Open-loop, flat-ground Gazebo SMB sensor trial; two shadow EKFs',
               'domain': os.environ['ROS_DOMAIN_ID'], 'physics_partition': os.environ['IGN_PARTITION'],
               'production_before': source_hashes(), 'runtime': json.loads((runtime/'manifest.json').read_text()),
               'source_sha256': {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.glob('*.py')},
               'ekf_publish_tf': False, 'planner_mapper_tracker_started': False,
               'truth_used_by_filter_or_commands': False, 'motion_profile': PROFILE,
               'wheel_twist_variance_prior': [.0004,.01,.01,.01,.01,.04],
               'uncertainty_note': 'Declared engineering priors, not tuned on this trial. Both EKFs use the same wheel data and parameters; fused adds gyro only.',
               'limitations': ['One trajectory and one IMU noise seed; no closed-loop GBPlanner result.',
                               'Both EKFs use a soft zero lateral-velocity observation; skid steering can violate this assumption.',
                               'Wheel yaw variance is deliberately conservative; filter priors have not been empirically calibrated.',
                               'Only XY and yaw are evaluated; no 3-D state-estimation claim.']}
    (out/'run.json').write_text(json.dumps(summary,indent=2))
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
            self.dynamic_tfs = set()
            self.shadow_tf_messages = 0
            self.input_enabled = False
            self.tf_buffer = Buffer(node=self)
            self.tf_listener = TransformListener(self.tf_buffer,self)
            self.velocity_pub = self.create_publisher(Twist,'/cmd_vel',10)
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

        def set_phase(self,name):
            self.phase=name
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
                if time.monotonic()-self.last_command_wall>.04:
                    self.publish_command()

        def duration(self,seconds):
            target=self.clock_ns+int(seconds*1e9)
            self.spin_until(lambda:self.clock_ns>=target,self.phase)

    try:
        rclpy.init(args=[]);initialized=True
        node=Recorder()
        launch_process('sensors',['ros2','launch',str(HERE/'sensor_trial.launch.py'),'output_dir:='+str(out/'generated')])
        node.spin_until(lambda: all(k in node.latest for k in ('truth','wheel_raw','imu_raw')) and node.mount is not None,
                        'wheel/IMU/truth and static mounting TF discovery')
        node.set_phase('settle');node.duration(2.)
        node.set_phase('calibration');node.duration(3.)
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
        node.set_phase('motion')
        for seconds,v,w in PROFILE:
            node.command=(v,w);node.publish_command();node.duration(seconds)
        node.command=(0.,0.);node.publish_command()
        node.set_phase('stopped');node.duration(2.)
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

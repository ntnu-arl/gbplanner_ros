#!/usr/bin/env python3
"""Exercise migrated Hector services on a native OS0 or OS1 IMU stream.

The simulator is paused only while checking exact service state transitions.
After resuming, recorded native/raw, adapted and bias messages are compared
at identical acquisition stamps. No odometry, motion or planner commands are
published. This is an IMU interface check, not an exploration mission.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback

sys.dont_write_bytecode = True
from check_fortress_smb_sensors import shutdown_launch
from check_fortress_autonomy import shutdown_log_events


def require(value, reason):
    if not value:
        raise AssertionError(reason)


def quaternion(value):
    return [value.w, value.x, value.y, value.z]


def multiply(a, b):
    w,x,y,z = a
    v,i,j,k = b
    return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]


def normalized(q):
    norm = math.sqrt(sum(x*x for x in q))
    return [x/norm for x in q]


def values(v):
    return [v.x, v.y, v.z]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--platform', choices=('smb','rmf'), required=True)
    parser.add_argument('--bridge', choices=('system','managed'), default='managed')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    log_path = output.with_suffix('.log')
    if output.exists() or log_path.exists():
        parser.error('Choose a new output; existing results are retained')
    output.parent.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[1]
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    env = dict(os.environ)
    env['IGN_PARTITION'] = 'gbplanner_fortress_'+env['ROS_DOMAIN_ID']
    stream = 'smb_os1' if args.platform == 'smb' else 'os0'
    topic = '/os1_cloud_node/imu' if args.platform == 'smb' else '/os0_cloud_node/imu'
    raw_topic = '/smb/os1/imu/raw' if args.platform == 'smb' else '/rmf/imu/os0/raw'
    status_service = '/smb/imu_state' if args.platform == 'smb' else '/rmf/imu_state'
    started = time.monotonic()
    deadline = started+110.
    summary = {'status':'failed', 'functional_status':'failed', 'scope':__doc__,
               'platform':args.platform, 'bridge':args.bridge, 'stream':stream,
               'topic':topic, 'raw_topic':raw_topic, 'domain':env['ROS_DOMAIN_ID'],
               'checks':{}, 'states':{}, 'world_control':[],
               'source_sha256':{}}
    for relative in ['tools/check_native_imu_services.py',
                     'src/gbplanner_gazebo/src/rmf_imu_adapter_node.cpp',
                     'src/gbplanner_gazebo/src/managed_bridge_node.cpp',
                     'src/hector_gazebo_plugins/srv/SetBias.srv']:
        p = root/relative
        if p.exists():summary['source_sha256'][relative] = hashlib.sha256(p.read_bytes()).hexdigest()
    node = process = log = None
    initialized = paused = False

    def world_pause(value):
        command = ['ign','service','-s','/world/edgar/control',
                   '--reqtype','ignition.msgs.WorldControl', '--reptype','ignition.msgs.Boolean',
                   '--timeout','2000','--req','pause: '+str(value).lower()]
        completed = subprocess.run(command, env=env, capture_output=True, text=True, timeout=4.)
        summary['world_control'].append({'pause':value,'exit_code':completed.returncode,
                                        'response':completed.stdout.strip(), 'stderr':completed.stderr.strip()})
        require(completed.returncode == 0 and 'data: true' in completed.stdout,
                'Native world pause/resume was not acknowledged')

    try:
        import rclpy
        from hector_gazebo_plugins.srv import SetBias
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rosidl_runtime_py.convert import message_to_ordereddict
        from sensor_msgs.msg import Imu
        from std_srvs.srv import Empty, Trigger

        rclpy.init(args=[])
        initialized = True
        node = Node('native_imu_services_probe')
        samples = {kind:{} for kind in ('raw','output','bias')}
        def receive(kind, message):
            stamp = int(message.header.stamp.sec)*1_000_000_000+message.header.stamp.nanosec
            samples[kind][stamp] = message
            if len(samples[kind]) > 500:
                del samples[kind][next(iter(samples[kind]))]
        for kind,name in [('raw',raw_topic),('output',topic),('bias',topic+'/bias')]:
            node.create_subscription(Imu,name,lambda m,k=kind:receive(k,m),qos_profile_sensor_data)
        clients = {'state':node.create_client(Trigger,status_service),
                   'calibrate':node.create_client(Empty,topic+'/calibrate'),
                   'accel':node.create_client(SetBias,topic+'/set_accel_bias'),
                   'rate':node.create_client(SetBias,topic+'/set_rate_bias')}
        def until(predicate, label):
            while not predicate():
                require(process is not None and process.poll() is None, 'Native launch exited: '+label)
                if time.monotonic() > deadline:raise TimeoutError(label)
                rclpy.spin_once(node,timeout_sec=.01)
        def spin_for(seconds):
            end = time.monotonic()+seconds
            until(lambda:time.monotonic() >= end, 'Timed observation')
        def call(kind, request):
            until(clients[kind].service_is_ready, kind+' service discovery')
            future = clients[kind].call_async(request)
            until(future.done, kind+' service response')
            return future.result()
        def state():
            response = call('state',Trigger.Request())
            require(response.success,'IMU state service failed')
            return json.loads(response.message)['streams'][stream]
        def set_bias(kind, vector):
            request = SetBias.Request()
            request.bias.x,request.bias.y,request.bias.z = vector
            call(kind,request)
        def matched(after=0):
            return sorted(t for t in set(samples['raw']) & set(samples['output']) & set(samples['bias']) if t > after)

        launch_file = 'edgar_fortress.launch.py' if args.platform == 'smb' else 'rmf_fortress.launch.py'
        command = ['ros2','launch','gbplanner_gazebo',launch_file,
                   'mapping:=false','planning:=false','bridge:='+args.bridge]
        if args.platform == 'smb':
            command += ['sensor_profile:=upstream','renderer:=project',
                        'renderer_prefix:='+str(root/'install_renderer')]
        summary['launch_command'] = command
        log = log_path.open('x')
        process = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,
                                   start_new_session=True,env=env)
        until(lambda:len(matched()) >= 10 and all(c.service_is_ready() for c in clients.values()),
              'Native IMU and migrated interfaces')
        summary['states']['before'] = state()
        world_pause(True)
        paused = True
        spin_for(.3)
        first_pause = state()
        spin_for(.15)
        before = state()
        require(first_pause['published'] == before['published'], 'Raw IMU stream did not settle while paused')
        summary['states']['paused'] = before
        accel = [.15,-.08,.03]
        rate = [.02,-.03,.04]
        set_bias('accel',accel)
        set_bias('rate',rate)
        set_state = state()
        summary['states']['set_bias'] = set_state
        h = set_state['hector']
        require(h['accel_bias'] == accel and h['rate_bias'] == rate, 'SetBias did not set the dynamic drift')
        require(set_state['source_stamp_s'] == before['source_stamp_s'] and
                set_state['published'] == before['published'], 'Service synthesized an IMU sample or reset time')
        call('calibrate',Empty.Request())
        calibrated = state()
        summary['states']['calibrated'] = calibrated
        require(calibrated['hector']['rate_bias'] == [0.,0.,0.] and
                calibrated['hector']['accel_bias'] == accel and
                calibrated['hector']['yaw_bias'] == h['yaw_bias'], 'Calibration changed the wrong noise state')
        require(calibrated['source_stamp_s'] == before['source_stamp_s'] and
                calibrated['published'] == before['published'], 'Calibration reset source time or published synthetic data')
        require(calibrated['hector']['calibration_calls'] == before['hector']['calibration_calls']+1 and
                calibrated['hector']['set_accel_bias_calls'] == before['hector']['set_accel_bias_calls']+1 and
                calibrated['hector']['set_rate_bias_calls'] == before['hector']['set_rate_bias_calls']+1,
                'Service counters do not match requests')
        # Leave both vectors nonzero so the resumed data path demonstrates
        # that each service changed the state actually consumed by the sensor.
        set_bias('rate',rate)
        resume_after = max(samples['raw'])
        world_pause(False)
        paused = False
        until(lambda:len(matched(resume_after)) >= 30, 'Thirty post-service native IMU samples')
        stamps = matched(resume_after)[:30]
        max_orientation = max_accel_residual = max_rate_residual = max_bias_deviation = 0.
        records = []
        for stamp in stamps:
            raw,out,bias = [samples[k][stamp] for k in ('raw','output','bias')]
            require(out.header == bias.header, 'Bias and measurement headers differ')
            expected = normalized(multiply(quaternion(bias.orientation),normalized(quaternion(raw.orientation))))
            actual = quaternion(out.orientation)
            error = min(math.sqrt(sum((a-b)**2 for a,b in zip(expected,actual))),
                        math.sqrt(sum((a+b)**2 for a,b in zip(expected,actual))))
            max_orientation = max(max_orientation,error)
            accel_residual = [o-r-b for o,r,b in zip(values(out.linear_acceleration),values(raw.linear_acceleration),values(bias.linear_acceleration))]
            rate_residual = [o-r-b for o,r,b in zip(values(out.angular_velocity),values(raw.angular_velocity),values(bias.angular_velocity))]
            max_accel_residual = max(max_accel_residual,*map(abs,accel_residual))
            max_rate_residual = max(max_rate_residual,*map(abs,rate_residual))
            max_bias_deviation = max(max_bias_deviation,
                *[abs(a-b) for a,b in zip(values(bias.linear_acceleration),accel)],
                *[abs(a-b) for a,b in zip(values(bias.angular_velocity),rate)])
            require(all(v == 0. for v in [*bias.orientation_covariance,*bias.angular_velocity_covariance,
                                          *bias.linear_acceleration_covariance]), 'Bias covariance is not the source default')
            records.append({'stamp_ns':stamp,'raw':message_to_ordereddict(raw),
                            'output':message_to_ordereddict(out),'bias':message_to_ordereddict(bias)})
        summary['comparison'] = {'matched_samples':len(stamps),'first_stamp_ns':stamps[0],
            'last_stamp_ns':stamps[-1],'max_orientation_composition_error':max_orientation,
            'max_accel_white_noise_residual':max_accel_residual,'max_rate_white_noise_residual':max_rate_residual,
            'max_dynamic_bias_change_after_resume':max_bias_deviation,
            'white_noise_check_scope':'A 0.05 bound checks these retained samples against the configured 0.005 sigma; it is not a noise distribution estimate'}
        summary['samples'] = records
        require(max_orientation < 1.e-10,'Bias topic contains the wrong orientation error')
        require(max_accel_residual < .05 and max_rate_residual < .05,'Post-service measurement lacks the expected bias/noise decomposition')
        require(max_bias_deviation < .001,'Requested drift was not retained after native samples resumed')
        final = state()
        summary['states']['final'] = final
        require(final['hector']['bias_published'] == final['published'], 'Bias was not published for each accepted native sample')
        summary['checks'] = {'native_input_observed':True,'set_bias_exact_while_paused':True,
            'calibration_only_zeros_rate_bias':True,'services_preserve_timestamp_and_sample_count':True,
            'same_stamp_bias_measurement_pairs':True,'orientation_error_composes_with_native_pose':True,
            'resumed_native_measurements_consume_requested_bias':True,'bias_covariance_preserved':True}
        summary['functional_status'] = 'passed'
    except Exception as error:
        summary['error'] = type(error).__name__+': '+str(error)
        summary['traceback'] = traceback.format_exc()
    finally:
        if paused and process is not None and process.poll() is None:
            try:world_pause(False)
            except Exception as error:summary['resume_cleanup_error'] = str(error)
        summary['cleanup'] = shutdown_launch(process)
        if log is not None:log.close()
        log_text = log_path.read_text() if log_path.exists() else ''
        failures,escalations = shutdown_log_events(log_text)
        summary['node_exit_failures'],summary['launch_signal_escalations'] = failures,escalations
        cleanup = summary['cleanup']
        clean = (cleanup.get('launch_returncode') == 0 and not cleanup.get('forced_kill') and
                 not cleanup.get('emergency_sigterm') and not cleanup.get('process_group_remaining') and
                 not failures and not escalations and 'Traceback (most recent call last)' not in log_text)
        summary['checks']['clean_shutdown'] = clean
        summary['status'] = 'passed' if clean and summary['functional_status'] == 'passed' else 'failed'
        if node is not None:node.destroy_node()
        if initialized:
            import rclpy
            if rclpy.ok():rclpy.shutdown()
        summary['wall_elapsed_s'] = time.monotonic()-started
        summary['log_file'] = str(log_path)
        output.write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'status':summary['status'],'functional_status':summary['functional_status'],
                      'output':str(output),'elapsed_s':summary['wall_elapsed_s'],'error':summary.get('error')}),flush=True)
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

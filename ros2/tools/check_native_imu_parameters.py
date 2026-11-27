#!/usr/bin/env python3
"""Change Hector model parameters and check same-stamp native measurement equations.

The world pauses for exact state transitions, then resumes with zero accel/rate
white noise and fixed drift so scale/offset equations can be checked directly.
No motion or planning is requested, and no noise-distribution claim is made.
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
from check_native_imu_services import multiply, normalized, quaternion, values


def require(value, message):
    if not value:
        raise AssertionError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--platform', choices=('smb', 'rmf'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    log_path = output.with_suffix('.log')
    require(not output.exists() and not log_path.exists(), 'Use a new output path')
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))
    env = dict(os.environ)
    env['IGN_PARTITION'] = 'gbplanner_fortress_' + env['ROS_DOMAIN_ID']
    stream = 'smb_os1' if args.platform == 'smb' else 'os0'
    topic = '/os1_cloud_node/imu' if args.platform == 'smb' else '/os0_cloud_node/imu'
    raw_topic = '/smb/os1/imu/raw' if args.platform == 'smb' else '/rmf/imu/os0/raw'
    status_service = '/smb/imu_state' if args.platform == 'smb' else '/rmf/imu_state'
    started = time.monotonic()
    deadline = started + 100.
    summary = {'status': 'failed', 'platform': args.platform, 'stream': stream,
               'scope': __doc__, 'states': {}, 'requests': [], 'source_sha256': {}}
    for relative in ('tools/check_native_imu_parameters.py',
                     'src/gbplanner_gazebo/src/rmf_imu_adapter_node.cpp',
                     'src/gbplanner_gazebo/src/managed_bridge_node.cpp'):
        summary['source_sha256'][relative] = hashlib.sha256((root / relative).read_bytes()).hexdigest()
    process = node = log = None
    initialized = paused = False

    def world_pause(value):
        result = subprocess.run(['ign', 'service', '-s', '/world/edgar/control',
            '--reqtype', 'ignition.msgs.WorldControl', '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '2000', '--req', 'pause: ' + str(value).lower()],
            env=env, capture_output=True, text=True, timeout=4.)
        require(result.returncode == 0 and 'data: true' in result.stdout, 'World pause/resume failed')

    try:
        import rclpy
        from hector_gazebo_plugins.srv import SetBias
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rcl_interfaces.srv import GetParameters, SetParametersAtomically
        from rclpy.qos import qos_profile_sensor_data
        from rosidl_runtime_py.convert import message_to_ordereddict
        from sensor_msgs.msg import Imu
        from std_srvs.srv import Empty, Trigger
        rclpy.init(args=[])
        initialized = True
        node = Node('native_imu_parameters_probe')
        parameter_set = node.create_client(SetParametersAtomically,
            '/gbplanner_rmf_imu_adapter/set_parameters_atomically')
        parameter_get = node.create_client(GetParameters,
            '/gbplanner_rmf_imu_adapter/get_parameters')
        clients = {'state': node.create_client(Trigger, status_service),
                   'accel': node.create_client(SetBias, topic + '/set_accel_bias'),
                   'rate': node.create_client(SetBias, topic + '/set_rate_bias'),
                   'calibrate': node.create_client(Empty, topic + '/calibrate')}
        samples = {kind: {} for kind in ('raw', 'output', 'bias')}
        def receive(kind, message):
            stamp = int(message.header.stamp.sec) * 1_000_000_000 + message.header.stamp.nanosec
            samples[kind][stamp] = message
            if len(samples[kind]) > 500:
                del samples[kind][next(iter(samples[kind]))]
        for kind, name in [('raw', raw_topic), ('output', topic), ('bias', topic + '/bias')]:
            node.create_subscription(Imu, name, lambda m, k=kind: receive(k, m), qos_profile_sensor_data)

        def until(predicate, description):
            while not predicate():
                require(process.poll() is None, 'Native launch exited: ' + description)
                if time.monotonic() > deadline:
                    raise TimeoutError(description)
                rclpy.spin_once(node, timeout_sec=.01)
        def spin_for(seconds):
            end = time.monotonic() + seconds
            until(lambda: time.monotonic() >= end, 'Timed observation')
        def finish(future):
            until(future.done, 'Service response')
            return future.result()
        def state():
            response = finish(clients['state'].call_async(Trigger.Request()))
            require(response.success, 'IMU state service rejected request')
            return json.loads(response.message)['streams'][stream]
        def set_parameters(entries, expected):
            request = SetParametersAtomically.Request()
            request.parameters = [Parameter(name, value=value).to_parameter_msg()
                                  for name, value in entries.items()]
            response = finish(parameter_set.call_async(request))
            summary['requests'].append({'parameters': entries,
                'successful': response.result.successful, 'reason': response.result.reason})
            require(response.result.successful == expected, 'Unexpected atomic parameter result')
        def matched(after=0):
            return sorted(stamp for stamp in set(samples['raw']) & set(samples['output']) &
                          set(samples['bias']) if stamp > after)

        launch = 'edgar_fortress.launch.py' if args.platform == 'smb' else 'rmf_fortress.launch.py'
        command = ['ros2', 'launch', 'gbplanner_gazebo', launch,
                   'mapping:=false', 'planning:=false', 'bridge:=managed']
        if args.platform == 'smb':
            command += ['sensor_profile:=upstream', 'renderer:=project',
                        'renderer_prefix:=' + str(root / 'install_renderer')]
        summary['launch_command'] = command
        log = log_path.open('x')
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   env=env, start_new_session=True)
        until(lambda: len(matched()) >= 10 and parameter_set.service_is_ready() and parameter_get.service_is_ready() and
              all(c.service_is_ready() for c in clients.values()), 'Native streams and parameter services')
        initial_stamp = matched()[-1]
        initial_out = samples['output'][initial_stamp]
        initial_covariances = {'accel': list(initial_out.linear_acceleration_covariance),
                               'rate': list(initial_out.angular_velocity_covariance)}
        world_pause(True)
        paused = True
        spin_for(.3)
        before = state()
        spin_for(.15)
        require(before['published'] == state()['published'], 'Paused world did not settle')
        summary['states']['before'] = before
        values_by_model = {
            'accel': {'Offset': [.1, -.2, .05], 'ScaleError': [1.02, .98, 1.01],
                      'Drift': [0., 0., 0.], 'GaussianNoise': [0., 0., 0.], 'DriftFrequency': [0., 0., 0.]},
            'rate': {'Offset': [.01, -.02, .03], 'ScaleError': [1.1, .9, 1.],
                     'Drift': [0., 0., 0.], 'GaussianNoise': [0., 0., 0.], 'DriftFrequency': [0., 0., 0.]},
            'yaw': {'Offset': .05, 'ScaleError': 1.4, 'Drift': 0., 'GaussianNoise': .02, 'DriftFrequency': 0.}}
        entries = {f'{stream}.{model}.{field}': value for model, fields in values_by_model.items()
                   for field, value in fields.items()}
        set_parameters(entries, True)
        updated = state()
        summary['states']['updated'] = updated
        require(updated['hector']['parameters'] == values_by_model, 'Model did not consume all parameter fields')
        for name in ('accel_drift', 'rate_drift', 'yaw_drift'):
            require(updated['hector'][name] == before['hector'][name], 'Parameter change reset drift: ' + name)
        require(updated['published'] == before['published'] and
                updated['source_stamp_s'] == before['source_stamp_s'], 'Parameter change synthesized a sample/time')
        require(updated['hector']['parameter_updates'] == before['hector']['parameter_updates'] + 1,
                'Atomic model update count mismatch')
        set_parameters({f'{stream}.accel.Offset': [.8, .8, .8],
                        f'{stream}.rate.GaussianNoise': [-1., 0., 0.]}, False)
        set_parameters({f'{stream}.accel.Offset': [.8, .8, .8], 'use_sim_time': False}, False)
        set_parameters({'gravity_magnitude': 9.7}, False)
        rejected = state()
        require(rejected['hector']['parameters'] == updated['hector']['parameters'] and
                rejected['hector']['parameter_updates'] == updated['hector']['parameter_updates'],
                'Rejected transaction partially modified the model')
        read_request = GetParameters.Request()
        read_request.names = list(entries)
        readback = finish(parameter_get.call_async(read_request))
        require([list(v.double_array_value) if v.type == Parameter.Type.DOUBLE_ARRAY.value else v.double_value
                 for v in readback.values] == list(entries.values()),
                'Model and ROS parameter server disagree')
        for name, vector in [('accel', [.03, -.04, .05]), ('rate', [.006, -.007, .008])]:
            request = SetBias.Request()
            request.bias.x, request.bias.y, request.bias.z = vector
            finish(clients[name].call_async(request))
        finish(clients['calibrate'].call_async(Empty.Request()))
        calibrated = state()
        summary['states']['calibrated'] = calibrated
        require(calibrated['hector']['rate_drift'] == [0., 0., 0.] and
                calibrated['hector']['rate_bias'] == values_by_model['rate']['Offset'],
                'Calibration removed fixed rate offset or failed to clear dynamic drift')
        require(calibrated['hector']['accel_drift'] == [.03, -.04, .05], 'SetBias/calibration changed accel drift')
        resume_after = max(samples['raw'])
        world_pause(False)
        paused = False
        until(lambda: len(matched(resume_after)) >= 30, 'Thirty post-update native samples')
        records = []
        maximum = {'accel': 0., 'rate': 0., 'orientation': 0., 'bias': 0.}
        for stamp in matched(resume_after)[:30]:
            raw, out, bias = [samples[k][stamp] for k in ('raw', 'output', 'bias')]
            require(out.header == bias.header, 'Bias and measurement headers differ')
            for name, field in [('accel', 'linear_acceleration'), ('rate', 'angular_velocity')]:
                source = values(getattr(raw, field))
                measured = values(getattr(out, field))
                total_bias = values(getattr(bias, field))
                expected_bias = calibrated['hector'][name + '_bias']
                maximum['bias'] = max(maximum['bias'], *(abs(a-b) for a,b in zip(total_bias, expected_bias)))
                scale = values_by_model[name]['ScaleError']
                maximum[name] = max(maximum[name], *(abs(v - (r*s+b)) for v,r,s,b in
                                                    zip(measured, source, scale, total_bias)))
            # Source orientation uses total accel/yaw bias, never scale_error.
            q = normalized(quaternion(raw.orientation))
            vector_q = [0., *values(bias.linear_acceleration)]
            world_bias = multiply(multiply(q, vector_q), [q[0], -q[1], -q[2], -q[3]])[1:]
            yaw = calibrated['hector']['yaw_bias']
            error = normalized(multiply([math.cos(yaw/2.), 0., 0., math.sin(yaw/2.)],
                [1., .5*world_bias[1]/9.81, -.5*world_bias[0]/9.81, 0.]))
            expected_q = normalized(multiply(error, q))
            actual_q = quaternion(out.orientation)
            maximum['orientation'] = max(maximum['orientation'],
                min(math.dist(expected_q, actual_q), math.dist(expected_q, [-x for x in actual_q])))
            require(list(out.linear_acceleration_covariance) == initial_covariances['accel'] and
                    list(out.angular_velocity_covariance) == initial_covariances['rate'],
                    'Hector startup accel/rate covariances changed at runtime')
            require(out.orientation_covariance[0] == 0. and out.orientation_covariance[4] == 0. and
                    abs(out.orientation_covariance[8] - .02**2) < 1.e-15,
                    'Orientation covariance did not follow current noise parameters')
            records.append({'stamp_ns': stamp, 'raw': message_to_ordereddict(raw),
                            'output': message_to_ordereddict(out), 'bias': message_to_ordereddict(bias)})
        summary['samples'] = records
        summary['maximum_equation_errors'] = maximum
        require(all(value < 1.e-9 for value in maximum.values()), 'Native sample equations do not match Hector')
        summary['functional_status'] = 'passed'
    except Exception as error:
        summary['error'] = str(error)
        summary['traceback'] = traceback.format_exc()
    finally:
        if paused and process is not None and process.poll() is None:
            try:
                world_pause(False)
            except Exception as error:
                summary['resume_error'] = str(error)
        summary['cleanup'] = shutdown_launch(process)
        if log is not None:
            log.close()
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()
        log_text = log_path.read_text(errors='replace') if log_path.exists() else ''
        summary['node_exit_failures'], summary['signal_escalations'] = shutdown_log_events(log_text)
        cleanup = summary['cleanup']
        if (summary.get('functional_status') == 'passed' and cleanup.get('launch_returncode') == 0
                and not any(cleanup.get(k) for k in ('forced_kill', 'emergency_sigterm', 'process_group_remaining'))
                and not summary['node_exit_failures'] and not summary['signal_escalations']):
            summary['status'] = 'passed'
        summary['wall_elapsed_s'] = time.monotonic() - started
        output.write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps({k: summary.get(k) for k in ('status', 'platform', 'error',
              'maximum_equation_errors', 'wall_elapsed_s', 'node_exit_failures', 'signal_escalations')}, indent=2))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

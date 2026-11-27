#!/usr/bin/env python3
"""One native ROS workflow for sphere bounds, radius sampling, and gain rays."""
import argparse
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import time


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=workspace/'results/gain_sphere/summary.json')
    args = parser.parse_args()
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))
    namespace = f'/gain_sphere_{os.getpid()}'
    started = time.monotonic()
    process = node = None
    initialized = False
    summary = {'status': 'failed', 'domain': os.environ['ROS_DOMAIN_ID'],
               'namespace': namespace, 'checks': {}}

    def require(condition, message):
        if not condition:
            raise AssertionError(message)

    try:
        import rclpy
        from geometry_msgs.msg import Pose
        from nav_msgs.msg import Odometry
        from planner_msgs.srv import PlannerSetGlobalBound, PlannerSrv, ValidatePath
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2, PointField
        from std_srvs.srv import Trigger
        from visualization_msgs.msg import Marker

        rclpy.init(args=[])
        initialized = True
        node = Node('gain_sphere_probe', namespace=namespace)
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        maps = node.create_publisher(PointCloud2, 'voxel_map', durable)
        odometry = node.create_publisher(Odometry, 'odometry', 10)
        accepted, bounds, sampling = [], [], []
        node.create_subscription(PointCloud2, 'accepted_voxel_map', accepted.append, durable)
        node.create_subscription(Marker, 'global_bound_marker', bounds.append, durable)
        node.create_subscription(Marker, 'sampling_bound_marker', sampling.append, durable)
        validator = node.create_client(ValidatePath, 'validate_path')
        set_bound = node.create_client(PlannerSetGlobalBound, 'gbplanner/set_global_bound')
        bound_status = node.create_client(Trigger, 'global_bound')
        sensor_gain = node.create_client(Trigger, 'sensor_gain')
        planner = node.create_client(PlannerSrv, 'gbplanner')

        def pose(x=0., y=0., z=0.):
            value = Pose()
            value.position.x, value.position.y, value.position.z = float(x), float(y), float(z)
            value.orientation.w = 1.
            return value

        def publish_odometry():
            message = Odometry()
            message.header.frame_id, message.child_frame_id = 'map', 'base_link'
            message.header.stamp = node.get_clock().now().to_msg()
            message.pose.pose = pose()
            odometry.publish(message)

        node.create_timer(.05, publish_odometry)
        options = {
            'map.resolution': '0.5', 'map.origin': '[-7.0, -7.0, -3.0]',
            'map.dimensions': '[28, 28, 12]', 'global.enable': 'false',
            'global.background_enable': 'false', 'robot.size': '[0.4, 0.4, 0.4]',
            'planning.global_type': 'kSphere', 'planning.global_center': '[0.0, 0.0, 0.0]',
            'planning.global_radius': '2.0', 'sampling.space_type': 'kSphere',
            'sampling.radius': '1.0', 'sampling.radius_extension': '0.5', 'sampling.seed': '42',
            'planning.max_vertices': '24', 'planning.max_attempts': '2000',
            'sensor.range': '6.0', 'sensor.horizontal_fov': '1.2', 'sensor.vertical_fov': '0.5',
            'sensor.horizontal_resolution': '0.1', 'sensor.vertical_resolution': '0.125',
            'planning.nonuniform_ray_cast': 'true', 'planning.ray_cast_step_size_multiplier': '1.0',
            'planning.max_gain_ray_samples': '1000000', 'gain.free': '0.05',
        }
        # Deliberately omit ray_cast_method: the native default must be iterative.
        launch = ['ros2', 'run', 'gbplanner_local', 'local_planner_node', '--ros-args',
                  '-r', '__ns:='+namespace]
        for key, value in options.items():
            launch += ['-p', key+':='+value]
        summary['launch'] = launch
        log_path = output.with_suffix('.log')
        with log_path.open('w') as log:
            process = subprocess.Popen(launch, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True, env=dict(os.environ))
        deadline = time.monotonic()+30.

        def until(predicate, label):
            while not predicate():
                if process.poll() is not None:
                    raise RuntimeError(f'Planner exited with {process.returncode}; see {log_path}')
                if time.monotonic() > deadline:
                    raise TimeoutError(label)
                rclpy.spin_once(node, timeout_sec=.03)

        def call(client, request):
            until(client.service_is_ready, 'ROS service discovery')
            pending = client.call_async(request)
            until(pending.done, 'ROS service response')
            return pending.result()

        def status():
            response = call(bound_status, Trigger.Request())
            require(response.success, response.message)
            return json.loads(response.message)

        def validate(end):
            request = ValidatePath.Request()
            request.header.frame_id, request.bound_mode = 'map', 3
            request.path = [pose(), pose(*end)]
            response = call(validator, request)
            return {'valid': response.valid, 'reason': response.reason, 'map_revision': response.map_revision}

        def marker_record(expected_type, first=0):
            until(lambda: any(marker.type == expected_type for marker in bounds[first:]), 'Matching bound Marker')
            marker = next(marker for marker in reversed(bounds[first:]) if marker.type == expected_type)
            return {'type': marker.type, 'center': [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z],
                    'scale': [marker.scale.x, marker.scale.y, marker.scale.z]}

        until(lambda: validator.service_is_ready() and maps.get_subscription_count() > 0
              and odometry.get_subscription_count() > 0, 'Planner/map/odometry discovery')
        cloud = PointCloud2()
        cloud.header.frame_id, cloud.header.stamp = 'map', node.get_clock().now().to_msg()
        cloud.height, cloud.width, cloud.point_step = 1, 28*28*12, 16
        cloud.row_step, cloud.is_dense = cloud.width*cloud.point_step, True
        cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z'))]
        cloud.fields.append(PointField(name='occupancy', offset=12, datatype=PointField.INT8, count=1))
        data = bytearray(cloud.row_step)
        for z in range(12):
            for y in range(28):
                for x in range(28):
                    offset = ((z*28+y)*28+x)*cloud.point_step
                    struct.pack_into('<fffbxxx', data, offset, -7.+(x+.5)*.5,
                                     -7.+(y+.5)*.5, -3.+(z+.5)*.5, 0)
        cloud.data = bytes(data)
        publish_odometry()
        maps.publish(cloud)
        until(lambda: bool(accepted), 'Free map acceptance')
        require(bytes(accepted[-1].data) == bytes(cloud.data), 'Accepted map changed')

        initial = status()
        initial_marker = marker_record(Marker.SPHERE)
        inside, crossing = validate((1., 0., 0.)), validate((1.7, .7, 0.))
        require(initial['type'] == 'kSphere' and initial['radius'] == 2. and
                initial['center'] == [0., 0., 0.] and initial_marker['scale'] == [4., 4., 4.],
                'Initial sphere status and Marker disagree')
        require(inside['valid'] and not crossing['valid'], 'Sphere did not constrain the complete robot body')
        summary['initial'] = {'bound': initial, 'marker': initial_marker,
                              'inside_path': inside, 'crossing_body': crossing}

        first = len(bounds)
        change = PlannerSetGlobalBound.Request()
        change.bound.use_z_val = True
        change.bound.min_val.x = change.bound.min_val.y = change.bound.min_val.z = -2.
        change.bound.max_val.x = change.bound.max_val.y = change.bound.max_val.z = 2.
        require(call(set_bound, change).success, 'Cuboid service change was rejected')
        cube, cube_marker = status(), marker_record(Marker.CUBE, first)
        cube_crossing = validate((1.7, .7, 0.))
        require(cube['type'] == 'kCuboid' and cube['radius'] == 0. and cube_crossing['valid'],
                'Cuboid request did not change the actual boundary')
        summary['cuboid'] = {'bound': cube, 'marker': cube_marker, 'same_path': cube_crossing}

        first = len(bounds)
        reset = PlannerSetGlobalBound.Request()
        reset.reset_to_default = True
        require(call(set_bound, reset).success, 'Default-bound reset was rejected')
        restored, restored_marker = status(), marker_record(Marker.SPHERE, first)
        restored_crossing = validate((1.7, .7, 0.))
        require(all(restored[key] == initial[key] for key in ('type', 'radius', 'center', 'half_size'))
                and not restored_crossing['valid'], 'Reset failed to restore the spherical geometry')
        summary['restored'] = {'bound': restored, 'marker': restored_marker, 'same_path': restored_crossing}

        gain_response = call(sensor_gain, Trigger.Request())
        require(gain_response.success, gain_response.message)
        gain = json.loads(gain_response.message)
        require(len(gain['sensors']) == 1, 'Unexpected gain sensor count')
        sensor = gain['sensors'][0]
        require(sensor['ray_method'] == 'iterative' and sensor['gain_model'] == 'ray'
                and sensor['range'] > 4. and sensor['ray_step_increases'] > 0
                and sensor['weighted_ray_samples'] >= sensor['ray_queries'] > 0,
                'Native gain did not execute iterative nonuniform ray casting')
        summary['sensor_gain'] = gain

        request = PlannerSrv.Request()
        request.header.frame_id, request.bound_mode = 'map', 3
        response = call(planner, request)
        require(response.status >= 0 and len(response.path) > 1, 'Local radius-configured plan failed')
        until(lambda: bool(sampling), 'Local sampling bound Marker')
        sample_marker = sampling[-1]
        sample_scale = [sample_marker.scale.x, sample_marker.scale.y, sample_marker.scale.z]
        path = [[point.position.x, point.position.y, point.position.z] for point in response.path]
        require(sample_marker.type == Marker.CUBE and all(abs(value-2.) < 1e-9 for value in sample_scale)
                and all(abs(value) <= 1.+1e-9 for point in path for value in point),
                'Local sphere radius did not retain the original sampling AABB')
        summary['local_radius_sampling'] = {'radius': 1., 'radius_extension': .5,
                                            'marker_type': sample_marker.type, 'scale': sample_scale, 'path': path}
        summary['checks'] = {'sphere_body_constraint': True, 'cuboid_switch_and_sphere_reset': True,
                             'markers_match_shape': True, 'native_iterative_nonuniform_gain': True,
                             'local_radius_keeps_sampling_aabb': True}
        summary['status'] = 'passed'
    except Exception as error:
        summary['error'] = f'{type(error).__name__}: {error}'
    finally:
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=1.)
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        summary['elapsed_s'] = round(time.monotonic()-started, 3)
        summary['processes_stopped'] = process is None or process.poll() is not None
        output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(summary, ensure_ascii=False))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

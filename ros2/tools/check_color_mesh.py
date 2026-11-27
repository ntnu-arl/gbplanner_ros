#!/usr/bin/env python3
"""Short native Voxblox color/periodic-mesh check in a sourced ROS 2 workspace."""
import argparse
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 80))
    import rclpy
    from gbplanner_voxblox.srv import FilePath
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, PointField
    from std_srvs.srv import Trigger
    from visualization_msgs.msg import Marker

    args.output.parent.mkdir(parents=True, exist_ok=True)
    result = {'status': 'failed', 'cases': [], 'checks': {}}
    log_path = args.output.with_suffix('.log')
    process = None
    started = time.monotonic()
    deadline = started + 18.
    rclpy.init(args=[])
    probe = Node('color_mesh_probe')
    durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    meshes, snapshots = [], []
    probe.create_subscription(Marker, 'surface_mesh', meshes.append, durable)
    probe.create_subscription(PointCloud2, 'voxel_map', snapshots.append, durable)
    publisher = probe.create_publisher(PointCloud2, 'points', qos_profile_sensor_data)
    reset = probe.create_client(Trigger, 'reset_map')
    status = probe.create_client(Trigger, 'mapper_status')
    export = probe.create_client(FilePath, 'export_mesh')

    def spin_until(predicate, label):
        while not predicate():
            if process is not None and process.poll() is not None:
                raise RuntimeError(f'Mapper exited ({process.returncode}): {log_path}')
            if time.monotonic() >= deadline:
                raise TimeoutError(label)
            rclpy.spin_once(probe, timeout_sec=.03)

    def call(client, request):
        spin_until(client.service_is_ready, 'Service discovery')
        future = client.call_async(request)
        spin_until(future.done, 'Service response')
        reply = future.result()
        if not reply.success:
            raise RuntimeError(reply.message)
        return reply

    def make_cloud(field_name, datatype, big_endian, color):
        cloud = PointCloud2()
        cloud.header.frame_id = 'map'
        cloud.header.stamp = probe.get_clock().now().to_msg()
        cloud.width = cloud.height = 31
        cloud.is_bigendian = big_endian
        cloud.is_dense = True
        cloud.point_step = 16 if field_name else 12
        cloud.row_step = cloud.width * cloud.point_step + 8
        for i, name in enumerate(('x', 'y', 'z')):
            cloud.fields.append(PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1))
        if field_name:
            cloud.fields.append(PointField(name=field_name, offset=12, datatype=datatype, count=1))
        data = bytearray(cloud.row_step * cloud.height)
        endian = '>' if big_endian else '<'
        packed = (0xff << 24) | (color[0] << 16) | (color[1] << 8) | color[2]
        for row in range(cloud.height):
            for col in range(cloud.width):
                offset = row*cloud.row_step + col*cloud.point_step
                # Dense planar returns produce a real marching-cubes surface.
                struct.pack_into(endian+'fff', data, offset, 2.1, (col-15)*.08, (row-15)*.08)
                if field_name:
                    # FLOAT32 color fields carry these bits, not a float value.
                    struct.pack_into(endian+'I', data, offset+12, packed)
        cloud.data = bytes(data)
        return cloud

    try:
        with log_path.open('w') as log:
            process = subprocess.Popen([
                'ros2', 'run', 'gbplanner_voxblox', 'voxblox_mapper_node', '--ros-args',
                '-p', 'sensor_frame:=map', '-p', 'map.resolution:=0.2',
                '-p', 'map.origin:=[-4.0,-4.0,-2.0]', '-p', 'map.dimensions:=[40,40,20]',
                '-p', 'tsdf.truncation_distance:=0.6', '-p', 'tsdf.method:=simple',
                '-p', 'color_mode:=colors', '-p', 'update_mesh_every_n_sec:=0.15',
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=dict(os.environ))
        spin_until(lambda: publisher.get_subscription_count() > 0 and status.service_is_ready(),
                   'Mapper discovery')
        layouts = [
            ('rgb_float32_little', 'rgb', PointField.FLOAT32, False, (240, 40, 20)),
            ('rgba_uint32_big', 'rgba', PointField.UINT32, True, (20, 230, 40)),
            ('rgb_int32_little', 'rgb', PointField.INT32, False, (20, 40, 240)),
            ('xyz_big_gray', '', None, True, (160, 160, 160)),
        ]
        for label, field_name, datatype, big_endian, color in layouts:
            reset_start = len(meshes)
            call(reset, Trigger.Request())
            spin_until(lambda: any(m.action == Marker.DELETE for m in meshes[reset_start:]),
                       'Reset must clear the old surface')
            mesh_start = len(meshes)
            cloud = make_cloud(field_name, datatype, big_endian, color)
            publisher.publish(cloud)
            spin_until(lambda: any(m.header.stamp == cloud.header.stamp for m in snapshots),
                       f'{label}: cloud not integrated')
            spin_until(lambda: any(m.action == Marker.ADD and m.points for m in meshes[mesh_start:]),
                       f'{label}: no periodic surface')
            marker = next(m for m in meshes[mesh_start:] if m.action == Marker.ADD and m.points)
            values = [(c.r*255, c.g*255, c.b*255) for c in marker.colors]
            average = [sum(c[i] for c in values)/len(values) for i in range(3)] if values else []
            mapper = json.loads(call(status, Trigger.Request()).message)
            item = {
                'layout': label, 'points_integrated': mapper['accepted_points'],
                'mesh_vertices': len(marker.points), 'mesh_colors': len(marker.colors),
                'mean_rgb': average, 'expected_rgb': color,
                'passed': len(marker.colors) == len(marker.points) > 0 and
                          all(abs(average[i]-color[i]) < 3. for i in range(3)) and
                          mapper['integrated_scans'] == 1 and mapper['rejected_scans'] == 0,
            }
            result['cases'].append(item)
            if label == 'rgba_uint32_big':
                ply = args.output.parent/'colored_surface.ply'
                call(export, FilePath.Request(file_path=str(ply)))
                text = ply.read_text()
                header, vertices = text.split('end_header\n', 1)
                vertex_count = int(next(line.split()[-1] for line in header.splitlines()
                                        if line.startswith('element vertex ')))
                properties = [line.split()[-1] for line in header.splitlines()
                              if line.startswith('property ') and not line.startswith('property list')]
                rows = vertices.splitlines()[:vertex_count]
                colors = [tuple(float(row.split()[properties.index(name)]) for name in ('red', 'green', 'blue'))
                          for row in rows]
                result['ply'] = {'file': str(ply), 'vertices': vertex_count,
                                 'has_rgb': all(name in properties for name in ('red', 'green', 'blue')),
                                 'green_vertices': sum(g > 200 and r < 50 and b < 60 for r, g, b in colors)}
        final = json.loads(call(status, Trigger.Request()).message)
        result['checks'] = {
            'all_color_layouts_and_gray': all(item['passed'] for item in result['cases']),
            'periodic_mesh_multiple_map_updates': final['mesh_updates'] >= len(layouts)+1,
            'colored_ply_export': result['ply']['green_vertices'] == result['ply']['vertices'] > 0,
            'row_padding_supported': all(item['points_integrated'] == 31*31 for item in result['cases']),
        }
        result['mesh_updates'] = final['mesh_updates']
        result['status'] = 'passed' if all(result['checks'].values()) else 'failed'
    except Exception as error:
        result['error'] = f'{type(error).__name__}: {error}'
    finally:
        result['elapsed_seconds'] = round(time.monotonic()-started, 3)
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=1.)
        probe.destroy_node()
        rclpy.shutdown()
        args.output.write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2))
    return 0 if result['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

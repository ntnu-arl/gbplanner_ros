#!/usr/bin/env python3
"""Exercise a usable mapping workflow: save, restart, continue, mesh and ESDF plan."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from gbplanner_voxblox.srv import FilePath
sys.path.insert(0,str(Path(get_package_prefix('gbplanner_local'))/'lib/gbplanner_local'))
from local_demo import Demo,xyz
from mapping_demo import lidar_cloud,inspect_snapshot,verify_path

class MapSession(Demo):
    def __init__(self):
        super().__init__()
        self.outputs={};self.mesh=None
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic in ('voxel_map','tsdf_voxels','esdf_voxels'):
            self.create_subscription(PointCloud2,topic,lambda message,t=topic:self.outputs.update({t:message}),qos)
        self.create_subscription(Marker,'surface_mesh',lambda message:setattr(self,'mesh',message),qos)
        self.clouds=self.create_publisher(PointCloud2,'points',qos_profile_sensor_data)
        self.tf=TransformBroadcaster(self)
        self.mapper=self.create_client(Trigger,'mapper_status')
        self.files={name:self.create_client(FilePath,name) for name in ('save_map','load_map','export_mesh')}
        for client in [self.mapper,*self.files.values()]:
            assert client.wait_for_service(timeout_sec=15)
    def status(self):return json.loads(self.call(self.mapper,Trigger.Request()).message)
    def file(self,operation,path):
        request=FilePath.Request();request.file_path=str(path)
        result=self.call(self.files[operation],request)
        assert result.success,(operation,result.message)
        return result.message
    def observe(self,origin,yaw=math.pi/2):
        self.spin_until(lambda:self.clouds.get_subscription_count()>0)
        stamp=self.get_clock().now().to_msg()
        tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='lidar';tf.header.stamp=stamp
        tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=origin
        tf.transform.rotation.z=math.sin(yaw/2);tf.transform.rotation.w=math.cos(yaw/2)
        self.tf.sendTransform(tf)
        deadline=time.monotonic()+.25
        while time.monotonic()<deadline:rclpy.spin_once(self,timeout_sec=.02)
        before=self.status()['integrated_scans']
        self.clouds.publish(lidar_cloud(stamp,origin=origin,yaw=yaw))
        self.spin_until(lambda:self.status()['integrated_scans']>before)
        self.spin_until(lambda:all(topic in self.outputs and self.outputs[topic].header.stamp==stamp
                                  for topic in ('voxel_map','tsdf_voxels','esdf_voxels')))
        self.spin_until(lambda:self.revision()['known_cells']==self.outputs['voxel_map'].width)
    def plan_at(self,origin):
        self.spin_until(lambda:self.odometry.get_subscription_count()>0)
        odom=Odometry();odom.header.frame_id='map';odom.header.stamp=self.get_clock().now().to_msg()
        odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=origin
        odom.pose.pose.orientation.w=1.
        self.odometry.publish(odom)
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            response,metrics=self.plan()
            if metrics['reason']!='odometry_missing_or_invalid':break
            rclpy.spin_once(self,timeout_sec=.05)
        assert metrics['success'],metrics
        path=xyz(response.path);verify_path(path,inspect_snapshot(self.outputs['voxel_map']))
        return path,metrics

def rows(message):
    return [struct.unpack_from('<'+'f'*(message.point_step//4),message.data,i*message.point_step)
            for i in range(message.width)]

def start(output,load=None,distance='tsdf'):
    command=['ros2','launch','gbplanner_voxblox','mapping.launch.py','esdf:=true',
             'distance_field:='+distance]
    if load:command.append('load_map_file:='+str(load))
    process=subprocess.Popen(command,start_new_session=True)
    rclpy.init();node=MapSession()
    return process,node

def stop(process,node):
    node.destroy_node();rclpy.shutdown();os.killpg(process.pid,signal.SIGINT)
    try:process.wait(timeout=8)
    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
    assert process.returncode==0,process.returncode

def export_csv(path,message,headers):
    with path.open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(headers);writer.writerows(rows(message))

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=Path('results/map_products'))
    args=parser.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    process=node=None;report={'status':'running'}
    try:
        process,node=start(out)
        node.observe((-2.,0.,0.))
        initial_path,initial_metrics=node.plan_at((-2.,0.,0.))
        original_tsdf=bytes(node.outputs['tsdf_voxels'].data)
        initial_known=node.status()['known_cells']
        node.file('save_map',out/'first_scan.vxblx')
        first_mesh=json.loads(node.file('export_mesh',out/'first_scan.ply'))
        node.spin_until(lambda:node.mesh is not None and len(node.mesh.points)==first_mesh['triangles']*3)
        assert first_mesh['triangles']>0
        stop(process,node);process=node=None
        # Restart both mapper and planner. No laser data is sent before planning.
        process,node=start(out,out/'first_scan.vxblx')
        node.spin_until(lambda:all(k in node.outputs for k in ('voxel_map','tsdf_voxels','esdf_voxels')))
        node.spin_until(lambda:node.revision()['known_cells']==initial_known)
        assert node.status()['integrated_scans']==0
        assert bytes(node.outputs['tsdf_voxels'].data)==original_tsdf
        restored_path,restored_metrics=node.plan_at((-2.,0.,0.))
        assert restored_path==initial_path
        # Exercise the explicit service too, then continue integrating fresh scans.
        node.file('load_map',out/'first_scan.vxblx')
        node.observe((2.,0.,0.))
        continued_status=node.status();assert continued_status['known_cells']>initial_known
        node.file('save_map',out/'room.vxblx')
        final_mesh=json.loads(node.file('export_mesh',out/'room.ply'))
        node.spin_until(lambda:node.mesh is not None and len(node.mesh.points)==final_mesh['triangles']*3)
        export_csv(out/'tsdf.csv',node.outputs['tsdf_voxels'],['x','y','z','distance','weight'])
        export_csv(out/'esdf.csv',node.outputs['esdf_voxels'],['x','y','z','distance'])
        tsdf={row[:3]:row[3] for row in rows(node.outputs['tsdf_voxels'])}
        esdf={row[:3]:row[3] for row in rows(node.outputs['esdf_voxels'])}
        propagated=[position for position,distance in esdf.items()
                    if distance>.85 and position in tsdf and tsdf[position]<=.750001]
        assert propagated,'ESDF must extend distances beyond the TSDF truncation band'
        stop(process,node);process=node=None
        # Use the saved scene through the ESDF planning branch, without a scan.
        process,node=start(out,out/'room.vxblx','esdf')
        node.spin_until(lambda:all(k in node.outputs for k in ('voxel_map','tsdf_voxels','esdf_voxels')))
        node.spin_until(lambda:node.revision()['known_cells']>0)
        esdf_path,esdf_metrics=node.plan_at((-2.,0.,0.))
        assert node.status()['distance_field']=='esdf' and node.status()['integrated_scans']==0
        distances={tuple(math.floor((p-o)/.25) for p,o in zip(row[:3],(-5.,-5.,-1.5))):row[3]
                   for row in rows(node.outputs['esdf_voxels'])}
        for index,state in inspect_snapshot(node.outputs['voxel_map']).items():
            assert state==(100 if distances[index]<=.25+1e-6 else 0)
        report={'status':'passed','scope':'Original map I/O, surface extraction and ESDF on synthetic scans; restarted processes',
                'initial_known_cells':initial_known,'continued_known_cells':continued_status['known_cells'],
                'first_mesh':first_mesh,'room_mesh':final_mesh,
                'saved_tsdf_bytes':(out/'room.vxblx').stat().st_size,
                'saved_mesh_bytes':(out/'room.ply').stat().st_size,
                'initial_path':initial_path,'restored_path':restored_path,'esdf_path':esdf_path,
                'initial_planning':initial_metrics,'restored_planning':restored_metrics,'esdf_planning':esdf_metrics,
                'esdf_maximum_distance_m':max(esdf.values()),'esdf_beyond_tsdf_truncation_cells':len(propagated),
                'checks':{'restart_preserves_tsdf_distance_and_weight_exactly':True,
                          'restored_map_plans_without_new_scan':True,'restored_tsdf_path_identical':True,
                          'explicit_load_service':True,'continued_scan_expands_map':True,
                          'original_marching_cubes_mesh_published_and_saved':True,
                          'esdf_propagates_beyond_tsdf_band':True,'esdf_classification_and_planning':True}}
    finally:
        if node is not None:stop(process,node)
    (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()

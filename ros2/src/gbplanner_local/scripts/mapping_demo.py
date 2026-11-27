#!/usr/bin/env python3
"""Drive the mapper and planner with synthetic lidar returns and timestamped TF."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from local_demo import Demo, xyz

ORIGIN=(-2.0,0.0,0.0)
YAW=math.pi/2

def ray_box(origin,direction,lo,hi):
    lower,upper=0.0,float('inf')
    for i in range(3):
        if abs(direction[i])<1e-12:
            if origin[i]<lo[i] or origin[i]>hi[i]:return None
        else:
            a=(lo[i]-origin[i])/direction[i];b=(hi[i]-origin[i])/direction[i]
            lower=max(lower,min(a,b));upper=min(upper,max(a,b))
            if upper<lower:return None
    return lower if upper>=0 else None

def lidar_cloud(stamp,frame='lidar',origin=ORIGIN,yaw=YAW):
    message=PointCloud2();message.header.frame_id=frame;message.header.stamp=stamp
    message.height=1;message.point_step=12;message.is_dense=True
    message.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)
                    for i,n in enumerate(('x','y','z'))]
    data=bytearray()
    # Voxel-aligned walls and room boundaries; 6,480 ideal returns, no map supplied.
    boxes=[((-.25,-5,-1.5),(.25,-1,1.5)),((-.25,1,-1.5),(.25,5,1.5)),
           ((-5,-5,-1.5),(-4.75,5,1.5)),((4.75,-5,-1.5),(5,5,1.5)),
           ((-5,-5,-1.5),(5,-4.75,1.5)),((-5,4.75,-1.5),(5,5,1.5)),
           ((-5,-5,-1.5),(5,5,-1.25)),((-5,-5,1.25),(5,5,1.5))]
    for elevation in range(-70,71,4):
        v=math.radians(elevation)
        for azimuth in range(-180,180,2):
            h=math.radians(azimuth)
            local=(math.cos(v)*math.cos(h),math.cos(v)*math.sin(h),math.sin(v))
            direction=(math.cos(yaw)*local[0]-math.sin(yaw)*local[1],
                       math.sin(yaw)*local[0]+math.cos(yaw)*local[1],local[2])
            hits=[distance for lo,hi in boxes if (distance:=ray_box(origin,direction,lo,hi)) is not None]
            distance=min(hits,default=7.0)
            # Move just inside the struck voxel; farther-than-range returns are truncated.
            distance=distance+1e-4 if distance<=6.0 else 7.0
            data.extend(struct.pack('<fff',*(distance*c for c in local)))
    message.width=len(data)//12;message.row_step=len(data);message.data=bytes(data)
    return message

def inspect_snapshot(message):
    lookup={}
    fields={f.name:f.offset for f in message.fields}
    for i in range(message.width):
        offset=i*message.point_step
        p=[struct.unpack_from('<f',message.data,offset+fields[name])[0] for name in ('x','y','z')]
        index=tuple(math.floor((p[k]-(-5,-5,-1.5)[k])/.25) for k in range(3))
        lookup[index]=struct.unpack_from('<b',message.data,offset+fields['occupancy'])[0]
    return lookup

def verify_path(path,snapshot):
    for a,b in zip(path,path[1:]):
        steps=max(1,math.ceil(math.dist(a,b)/.025))
        for step in range(steps+1):
            center=[a[i]+(b[i]-a[i])*step/steps for i in range(3)]
            for dx in [-.2,-.1,0,.1,.2]:
                for dy in [-.2,-.1,0,.1,.2]:
                    for dz in [-.2,-.1,0,.1,.2]:
                        p=[center[0]+dx,center[1]+dy,center[2]+dz]
                        key=tuple(math.floor((p[i]-(-5,-5,-1.5)[i])/.25) for i in range(3))
                        assert snapshot.get(key,-1)==0, ('path left observed free space',p)
                        assert not(abs(p[0])<.25 and abs(p[1])>1),('path crosses wall',p)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--start-node',action='store_true')
    parser.add_argument('--backend',choices=['occupancy','voxblox'],default='occupancy')
    parser.add_argument('--output',default='results/mapping_integration.json');args=parser.parse_args()
    process=None;node=None
    try:
        if args.start_node:
            process=subprocess.Popen(['ros2','launch','gbplanner_voxblox' if args.backend=='voxblox' else 'gbplanner_local','mapping.launch.py'],start_new_session=True)
        rclpy.init();node=Demo()
        mapper=node.create_client(Trigger,'mapper_status');reset=node.create_client(Trigger,'reset_map')
        assert mapper.wait_for_service(timeout_sec=15) and reset.wait_for_service(timeout_sec=15)
        clouds=node.create_publisher(PointCloud2,'points',qos_profile_sensor_data)
        snapshots=[];tsdf_snapshots=[]
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(PointCloud2,'voxel_map',snapshots.append,qos)
        if args.backend=='voxblox':node.create_subscription(PointCloud2,'tsdf_voxels',tsdf_snapshots.append,qos)
        read=lambda:json.loads(node.call(mapper,Trigger.Request()).message)
        node.spin_until(lambda:clouds.get_subscription_count()>0)
        before=read()
        cloud=lidar_cloud(node.get_clock().now().to_msg())
        clouds.publish(cloud)
        node.spin_until(lambda:read()['rejected_scans']>before['rejected_scans'])
        assert read()['integrated_scans']==0
        broadcaster=StaticTransformBroadcaster(node)
        transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='lidar'
        transform.header.stamp=node.get_clock().now().to_msg()
        transform.transform.translation.x=ORIGIN[0]
        transform.transform.rotation.z=math.sin(YAW/2);transform.transform.rotation.w=math.cos(YAW/2)
        broadcaster.sendTransform(transform)
        deadline=time.monotonic()+10
        while read()['integrated_scans']==0:
            assert time.monotonic()<deadline,'Mapper did not obtain static transform'
            clouds.publish(cloud)
            rclpy.spin_once(node,timeout_sec=.1)
        node.spin_until(lambda:node.revision()['map_revision']>0 and snapshots)
        snapshot=inspect_snapshot(snapshots[-1]);initial=read()
        assert initial['known_cells']>1000 and initial['accepted_points']==cloud.width
        assert any(v==100 for v in snapshot.values()) and any(v==0 for v in snapshot.values())
        tsdf_summary=None
        if args.backend=='voxblox':
            node.spin_until(lambda:bool(tsdf_snapshots))
            raw=tsdf_snapshots[-1];assert raw.width==len(snapshot)
            assert initial['backend']=='voxblox_tsdf' and initial['allocated_blocks']>0
            rows=[]
            for i in range(raw.width):
                x,y,z,distance,weight=struct.unpack_from('<fffff',raw.data,i*raw.point_step)
                assert math.isfinite(distance) and math.isfinite(weight) and weight>=1e-6
                index=tuple(math.floor((v-o)/.25) for v,o in zip((x,y,z),(-5.,-5.,-1.5)))
                assert snapshot[index]==(100 if distance<=.25+1e-6 else 0)
                rows.append((x,y,z,distance,weight))
            import csv
            raw_path=Path(args.output).with_suffix('.csv');raw_path.parent.mkdir(parents=True,exist_ok=True)
            with raw_path.open('w',newline='') as stream:
                writer=csv.writer(stream);writer.writerow(['x','y','z','tsdf_distance_m','weight']);writer.writerows(rows)
            tsdf_summary={'observed_voxels':len(rows),'minimum_distance_m':min(r[3] for r in rows),
                          'maximum_distance_m':max(r[3] for r in rows),
                          'minimum_weight':min(r[4] for r in rows),'maximum_weight':max(r[4] for r in rows),
                          'all_occupancy_classes_match_original_threshold':True}

        odom=Odometry();odom.header.frame_id='map';odom.pose.pose.position.x=ORIGIN[0];odom.pose.pose.orientation.w=1.0
        node.spin_until(lambda:node.odometry.get_subscription_count()>0)
        deadline=time.monotonic()+5
        while True:
            node.odometry.publish(odom);response,metrics=node.plan()
            if metrics['reason']!='odometry_missing_or_invalid':break
            assert time.monotonic()<deadline
        assert metrics['success'],metrics
        coordinates=xyz(response.path);verify_path(coordinates,snapshot)
        first_metrics=metrics
        for bad in ['frame','data','fields','zero_stamp']:
            malformed=copy.deepcopy(cloud)
            if bad=='frame':malformed.header.frame_id='unconfigured_lidar'
            elif bad=='data':malformed.data=bytes(malformed.data)[:-1]
            elif bad=='fields':malformed.fields[1].offset=0
            else:malformed.header.stamp.sec=0;malformed.header.stamp.nanosec=0
            before=read();clouds.publish(malformed)
            node.spin_until(lambda:read()['rejected_scans']>before['rejected_scans'])
            assert read()['integrated_scans']==before['integrated_scans']
        before_revision=node.revision()['map_revision']
        assert node.call(reset,Trigger.Request()).success
        node.spin_until(lambda:node.revision()['map_revision']>before_revision)
        response,metrics=node.plan();assert not response.path and metrics['reason']=='root_not_free'
        assert read()['known_cells']==0
        clouds.publish(cloud);node.spin_until(lambda:read()['integrated_scans']>0)
        node.spin_until(lambda:node.revision()['known_cells']>0)
        response,metrics=node.plan();assert metrics['success'] and xyz(response.path)==coordinates
        output={'status':'passed','scope':'synthetic lidar -> timestamped TF -> '+args.backend+' mapper -> local RRG', 'backend':args.backend,
                'sensor_origin_map':ORIGIN,'sensor_yaw_rad':YAW,'input_returns':cloud.width,
                'mapper':initial,'tsdf':tsdf_summary,'planning':first_metrics,'path':coordinates,
                'checks':{'missing_tf_rejected':True,'rotated_sensor_frame':True,
                          'raw_xyz_cloud_integration':True,'path_in_observed_free_space':True,
                          'independent_wall_check':True,'malformed_clouds_rejected':True,'zero_timestamp_rejected':True,
                          'reset_clears_map':True,'reintegrating_same_scan_repeats_path':True}}
        path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True)
        temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(output,indent=2)+'\n');temporary.replace(path)
        print(json.dumps(output,indent=2))
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if process:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)

if __name__=='__main__':main()

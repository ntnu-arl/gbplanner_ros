#!/usr/bin/env python3
"""Synthetic occupancy snapshot integration test; no robot dynamics or SLAM."""
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
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger
from planner_msgs.srv import PlannerSrv,ValidatePath

ORIGIN=(-5.0,-5.0,-1.5)
DIM=(40,40,12)
RES=0.25

def status_at(p,known=False,wall=True):
    if any(p[i]<ORIGIN[i] or p[i]>=ORIGIN[i]+DIM[i]*RES for i in range(3)):
        return -1
    index=[math.floor((p[i]-ORIGIN[i])/RES) for i in range(3)]
    center=[ORIGIN[i]+(index[i]+0.5)*RES for i in range(3)]
    if wall and abs(center[0])<0.25 and abs(center[1])>1.0:
        return 100
    return -1 if not known and center[0]>=1.5 else 0

def make_map(known=False,wall=True):
    message=PointCloud2()
    message.header.frame_id='map'
    message.height=1
    message.width=math.prod(DIM)
    message.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)
                    for i,n in enumerate(('x','y','z'))]
    message.fields.append(PointField(name='occupancy',offset=12,datatype=PointField.INT8,count=1))
    message.point_step=16
    message.row_step=message.width*16
    message.is_dense=True
    data=bytearray(message.row_step)
    offset=0
    for z in range(DIM[2]):
        for y in range(DIM[1]):
            for x in range(DIM[0]):
                p=[ORIGIN[i]+(k+0.5)*RES for i,k in enumerate((x,y,z))]
                struct.pack_into('<fffbxxx',data,offset,*p,status_at(p,known,wall))
                offset+=16
    message.data=bytes(data)
    return message

class Demo(Node):
    def __init__(self):
        super().__init__('local_planner_integration')
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.maps=self.create_publisher(PointCloud2,'voxel_map',qos)
        self.odometry=self.create_publisher(Odometry,'odometry',10)
        self.metrics=[]
        self.paths=[]
        self.create_subscription(String,'planning_metrics',lambda m:self.metrics.append(json.loads(m.data)),qos)
        self.create_subscription(RosPath,'local_path',lambda m:self.paths.append(m),qos)
        self.planner=self.create_client(PlannerSrv,'planner')
        self.map_status=self.create_client(Trigger,'map_status')
        for client in (self.planner,self.map_status):
            if not client.wait_for_service(timeout_sec=15):
                raise RuntimeError('ROS 2 service did not become available')
    def call(self,client,request):
        future=client.call_async(request)
        rclpy.spin_until_future_complete(self,future,timeout_sec=30)
        if not future.done():
            raise TimeoutError('Service response timed out')
        return future.result()
    def spin_until(self,predicate,seconds=10):
        deadline=time.monotonic()+seconds
        while not predicate():
            if time.monotonic()>deadline:
                raise TimeoutError('Expected ROS 2 event missing')
            rclpy.spin_once(self,timeout_sec=0.05)
    def revision(self):
        return json.loads(self.call(self.map_status,Trigger.Request()).message)
    def publish_map(self,message):
        previous=self.revision()['map_revision']
        self.maps.publish(message)
        self.spin_until(lambda:self.revision()['map_revision']==previous+1)
    def plan(self,frame='map',mode=PlannerSrv.Request.K_EXACT_BOUND,root=None):
        previous=len(self.metrics)
        request=PlannerSrv.Request();request.header.frame_id=frame;request.bound_mode=mode
        # ROS 2 Quaternion defaults w=1; upstream uses an all-zero pose as
        # the explicit sentinel for selecting the most recent odometry.
        request.root_pose.orientation.w=0.0
        if root is not None:
            request.root_pose=root
        response=self.call(self.planner,request)
        self.spin_until(lambda:len(self.metrics)>previous)
        return response,self.metrics[-1]
    def reject_map(self,message):
        previous=self.revision()
        self.maps.publish(message)
        self.spin_until(lambda:self.revision()['rejected_maps']>previous['rejected_maps'])
        assert self.revision()['map_revision']==previous['map_revision']

def xyz(path):
    return [[p.position.x,p.position.y,p.position.z] for p in path]

def verify_path(path):
    assert len(path)>=2
    length=0
    for a,b in zip(path,path[1:]):
        distance=math.dist(a,b);length+=distance
        steps=max(1,math.ceil(distance/0.025))
        for step in range(steps+1):
            center=[a[i]+(b[i]-a[i])*step/steps for i in range(3)]
            # Independent grid probe of the complete box, finer than map resolution.
            for dx in (-0.2,-0.1,0,0.1,0.2):
                for dy in (-0.2,-0.1,0,0.1,0.2):
                    for dz in (-0.2,-0.1,0,0.1,0.2):
                        assert status_at([center[0]+dx,center[1]+dy,center[2]+dz])==0
    return length

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--start-node',action='store_true')
    parser.add_argument('--output',default='results/local_integration.json');args=parser.parse_args()
    process=None;node=None
    try:
        if args.start_node:
            process=subprocess.Popen(['ros2','launch','gbplanner_local','local.launch.py'],start_new_session=True)
        rclpy.init();node=Demo()
        missing,metric=node.plan();assert not missing.path and metric['reason']=='map_missing'
        node.publish_map(make_map())
        missing,metric=node.plan();assert not missing.path and metric['reason']=='odometry_missing_or_invalid',metric
        odom=Odometry();odom.header.frame_id='map';odom.pose.pose.position.x=-2.0;odom.pose.pose.orientation.w=1.0
        node.spin_until(lambda:node.odometry.get_subscription_count()>0)
        node.odometry.publish(odom)
        deadline=time.monotonic()+5
        while True:
            response,first=node.plan()
            if first['reason']!='odometry_missing_or_invalid': break
            assert time.monotonic()<deadline
            node.odometry.publish(odom)
        assert first['success'],first
        coordinates=xyz(response.path);length=verify_path(coordinates)
        validation=node.create_client(ValidatePath,'validate_path');assert validation.wait_for_service(timeout_sec=5)
        check=ValidatePath.Request();check.header.frame_id='map';check.path=response.path
        validated=node.call(validation,check);assert validated.valid and validated.reason=='ok'
        assert validated.map_revision==node.revision()['map_revision']
        check.header.frame_id='odom';assert node.call(validation,check).reason=='wrong_frame'
        check.header.frame_id='map';check.path=[];assert node.call(validation,check).reason=='invalid_path_size'
        invalid_pose=Pose();invalid_pose.orientation.w=0.;check.path=[invalid_pose]
        assert node.call(validation,check).reason=='invalid_pose'
        invalid_pose.orientation.w=1.;invalid_pose.position.x=3.;check.path=[invalid_pose]
        assert node.call(validation,check).reason=='path_not_free'
        assert math.dist(coordinates[0],[-2,0,0])<1e-9
        assert abs(length-first['path_length_m'])<1e-8
        repeat,again=node.plan();assert xyz(repeat.path)==coordinates
        assert again['gain']==first['gain'] and again['vertices']==first['vertices']
        node.spin_until(lambda:any(len(p.poses)==len(coordinates) for p in node.paths))
        wrong,metric=node.plan(frame='odom');assert not wrong.path and metric['reason']=='wrong_frame'
        wrong,metric=node.plan(mode=99)
        assert not wrong.path and metric['reason']=='unsupported_bound_mode'
        blocked=Pose();blocked.position.x=3.0;blocked.orientation.w=1.0
        wrong,metric=node.plan(root=blocked);assert not wrong.path and metric['reason']=='root_not_free'
        explicit=Pose();explicit.position.x=-1.0;explicit.orientation.w=1.0
        changed,metric=node.plan(root=explicit);assert metric['success']
        assert abs(changed.path[0].position.x+1)<1e-9
        base=make_map()
        wrong_map=copy.deepcopy(base);wrong_map.header.frame_id='odom';node.reject_map(wrong_map)
        malformed=copy.deepcopy(base);malformed.data=bytes(malformed.data)[:-1];node.reject_map(malformed)
        invalid=copy.deepcopy(base);data=bytearray(invalid.data);data[12]=50;invalid.data=bytes(data);node.reject_map(invalid)
        duplicate=copy.deepcopy(base);data=bytearray(duplicate.data);data[16:32]=data[:16];duplicate.data=bytes(data);node.reject_map(duplicate)
        overlap=copy.deepcopy(base);overlap.fields[1].offset=0;node.reject_map(overlap)
        kept,metric=node.plan();assert metric['success'] and xyz(kept.path)==coordinates
        node.publish_map(make_map(known=True))
        result,metric=node.plan();assert not result.path and metric['reason']=='no_gain'
        node.spin_until(lambda:node.paths and not node.paths[-1].poses)
        node.publish_map(make_map())
        restored,metric=node.plan();assert metric['success'] and xyz(restored.path)==coordinates
        output={'status':'passed','scope':'aerial local RRG with synthetic occupancy snapshots',
                'scenario':{'resolution':RES,'origin':ORIGIN,'dimensions':DIM,'unknown_x_min':1.5,
                            'wall_x':[-0.25,0.25],'door_y':[-1.0,1.0],'robot_size':[0.4]*3},
                'planning':first,'path':coordinates,'independent_path_length_m':length,
                'checks':{'map_and_odometry_required':True,'deterministic_repeat':True,
                          'independent_footprint_collision_check':True,'path_topic':True,
                          'wrong_frame_rejected':True,'unsupported_bound_rejected':True,
                          'unknown_root_rejected':True,'explicit_root_used':True,
                          'malformed_map_preserves_snapshot':True,'known_map_no_gain':True,
                          'map_replacement_replans':True},'final_map_status':node.revision()}
        output['checks']['validate_path_contract']=True
        target=Path(args.output);target.parent.mkdir(parents=True,exist_ok=True)
        temporary=target.with_suffix('.tmp');temporary.write_text(json.dumps(output,indent=2)+'\n');temporary.replace(target)
        print(json.dumps(output,indent=2))
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if process is not None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)

if __name__=='__main__':main()

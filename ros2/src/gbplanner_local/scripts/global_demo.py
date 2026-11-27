#!/usr/bin/env python3
"""ROS 2 integration of local-generated frontiers, global routing and homing."""
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
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry, Path as RosPath
from planner_msgs.srv import PlannerGlobal, PlannerHoming
from planner_msgs.msg import Graph
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from local_demo import Demo, make_map, xyz
from mapping_demo import inspect_snapshot, verify_path

class GlobalDemo(Demo):
    def __init__(self):
        super().__init__()
        self.global_metrics=[]
        self.global_paths=[]
        self.global_graphs=[]
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String,'global_metrics',lambda m:self.global_metrics.append(json.loads(m.data)),qos)
        self.create_subscription(RosPath,'global_path',self.global_paths.append,qos)
        self.create_subscription(Graph,'global_graph',self.global_graphs.append,qos)
        self.global_client=self.create_client(PlannerGlobal,'planner_global')
        self.homing_client=self.create_client(PlannerHoming,'exploration_homing')
        self.global_status=self.create_client(Trigger,'global_status')
        for client in (self.global_client,self.homing_client,self.global_status):
            assert client.wait_for_service(timeout_sec=15)

    def status(self):
        return json.loads(self.call(self.global_status,Trigger.Request()).message)

    def global_call(self,homing=False,frame='map',target=0,ignore_time=True,manual=False):
        previous=len(self.global_metrics)
        if homing:
            request=PlannerHoming.Request();request.header.frame_id=frame
            response=self.call(self.homing_client,request)
        else:
            request=PlannerGlobal.Request();request.id=target
            request.ignore_time=ignore_time;request.not_check_frontier=manual
            response=self.call(self.global_client,request)
        self.spin_until(lambda:len(self.global_metrics)>previous)
        return response,self.global_metrics[-1]

    def move_odometry(self,position,yaw=0.0):
        odom=Odometry();odom.header.frame_id='map';odom.header.stamp=self.get_clock().now().to_msg()
        odom.pose.pose.position.x=float(position[0]);odom.pose.pose.position.y=float(position[1]);odom.pose.pose.position.z=float(position[2])
        odom.pose.pose.orientation.z=math.sin(yaw/2);odom.pose.pose.orientation.w=math.cos(yaw/2)
        self.spin_until(lambda:self.odometry.get_subscription_count()>0)
        self.odometry.publish(odom)
        # A homing response beginning at the requested pose acknowledges that
        # the subscriber has consumed this sample before subsequent service calls.
        deadline=time.monotonic()+5
        while True:
            response,metrics=self.global_call(homing=True)
            if response.path and math.dist(xyz(response.path)[0],position)<1e-8:return
            assert time.monotonic()<deadline,metrics
            rclpy.spin_once(self,timeout_sec=.02)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--start-node',action='store_true')
    parser.add_argument('--output',type=Path,default=Path('results/global_integration.json'))
    args=parser.parse_args();process=None;node=None
    try:
        if args.start_node:
            config=Path(get_package_share_directory('gbplanner_local'))/'config/local.yaml'
            process=subprocess.Popen(['ros2','run','gbplanner_local','local_planner_node',
                '--ros-args','--params-file',str(config),'-p','global.enable:=true'],start_new_session=True)
        rclpy.init();node=GlobalDemo()
        response,metric=node.global_call();assert not response.path and metric['reason']=='map_or_odometry_missing'
        base=make_map();snapshot=inspect_snapshot(base);node.publish_map(base)
        node.move_odometry([-2,0,0])
        response,local_metric=node.plan();assert local_metric['success'],local_metric
        stored=node.status();assert stored['frontiers']>0,stored
        response,frontier_metric=node.global_call();assert frontier_metric['success'],frontier_metric
        frontier_path=xyz(response.path);verify_path(frontier_path,snapshot)
        assert math.dist(frontier_path[0],[-2,0,0])<1e-8
        assert abs(sum(math.dist(a,b) for a,b in zip(frontier_path,frontier_path[1:]))-frontier_metric['distance_m'])<1e-8
        before=node.status();explicit=Pose();explicit.position.x=-1.0
        response,metric=node.plan(root=explicit);assert metric['success']
        assert node.status()==before,'Hypothetical root changed persistent history'
        response,manual_metric=node.global_call(target=frontier_metric['target'],manual=True)
        assert manual_metric['success'];verify_path(xyz(response.path),snapshot)
        failures=[]
        for options,reason in [({'ignore_time':False},'time_budget_disabled'),
                               ({'target':-1},'invalid_global_target'),
                               ({'manual':True},'explicit_target_required'),
                               ({'target':1},'manual_frontier_check_not_implemented'),
                               ({'target':999999,'manual':True},'invalid_global_target'),
                               ({'homing':True,'frame':'odom'},'wrong_frame')]:
            response,metric=node.global_call(**options)
            assert not response.path and metric['reason']==reason,metric
            failures.append(reason)
        node.spin_until(lambda:node.global_paths and not node.global_paths[-1].poses)
        # Move through observed free space into a corner whose complete local
        # sampling window and 3 m sensor rays cannot reach the unknown region.
        # The previously retained frontier remains reachable in global memory.
        outward=[[-2,0,0],[-3,0,0],[-4,0,0],[-4.5,-1,0],[-4.5,-2,0],[-4.5,-3,0],[-4.5,-4.5,0]]
        verify_path(outward,snapshot)
        for point in outward[1:]:node.move_odometry(point)
        response,stalled_metric=node.plan()
        assert not response.path and stalled_metric['reason']=='no_gain',stalled_metric
        response,fallback_metric=node.global_call()
        assert fallback_metric['success'],fallback_metric
        fallback_path=xyz(response.path);verify_path(fallback_path,snapshot)
        assert math.dist(fallback_path[0],outward[-1])<1e-8
        for point in reversed(outward[:-1]):node.move_odometry(point)
        for point in [[-1,0,0],[0,0,0],[1,0,0],[1.02,0,0]]:node.move_odometry(point)
        response,homing_metric=node.global_call(homing=True);assert homing_metric['success']
        homing_path=xyz(response.path);verify_path(homing_path,snapshot)
        assert math.dist(homing_path[0],[1.02,0,0])<1e-8 and math.dist(homing_path[-1],[-2,0,0])<1e-8
        # Full-height wall closes every route between the robot and original home.
        blocked=copy.deepcopy(base);data=bytearray(blocked.data)
        for offset in range(0,len(data),blocked.point_step):
            if abs(struct.unpack_from('<f',data,offset)[0])<.25:data[offset+12]=100
        blocked.data=bytes(data);node.publish_map(blocked)
        response,blocked_metric=node.global_call(homing=True)
        assert not response.path and blocked_metric['reason']=='global_target_unreachable',blocked_metric
        # An entirely observed map must clear all now-obsolete frontier flags.
        node.publish_map(make_map(known=True))
        response,metric=node.global_call();assert not response.path and metric['reason']=='no_global_frontier',metric
        assert node.status()['frontiers']==0
        empty=copy.deepcopy(base);data=bytearray(empty.data)
        for offset in range(12,len(data),empty.point_step):data[offset]=255
        empty.data=bytes(data);node.publish_map(empty)
        assert node.status()['vertices']==0
        node.spin_until(lambda:node.global_graphs and len(node.global_graphs[-1].vertices)==0)
        output={'status':'passed','scope':'local RRG generated frontiers + ROS 2 persistent global routing + homing',
            'global_after_local':stored,'local':local_metric,'frontier':frontier_metric,'frontier_path':frontier_path,
            'homing':homing_metric,'homing_path':homing_path,'blocked_homing':blocked_metric,
            'stalled_local':stalled_metric,'fallback':fallback_metric,'fallback_path':fallback_path,
            'rejected_requests':failures,'checks':{'real_local_frontiers':True,'global_path_collision_checked':True,
                'explicit_root_preserves_history':True,'manual_target':True,'exact_current_pose_homing':True,
                'map_change_invalidates_return_route':True,'explored_frontiers_cleared':True,
                'empty_map_resets_global_memory':True,'failed_service_clears_path_topic':True}}
        output['checks']['local_no_gain_uses_retained_global_frontier']=True
        output['checks']['reset_clears_published_global_graph']=True
        args.output.parent.mkdir(parents=True,exist_ok=True)
        temporary=args.output.with_suffix('.tmp');temporary.write_text(json.dumps(output,indent=2)+chr(10));temporary.replace(args.output)
        print(json.dumps(output,indent=2))
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if process:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)

if __name__=='__main__':main()

#!/usr/bin/env python3
"""Sensor/action simulator for the persistent ROS 2 control pipeline.

Only publishes raw lidar, TF and odometry. All planning and switching occur in
the separate controller process. No occupancy snapshot is fabricated here.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.task import Future
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from planner_msgs.action import PathFollower
from planner_msgs.srv import PciTrigger,PlannerSrv
from global_demo import GlobalDemo
from local_demo import xyz
from mapping_demo import lidar_cloud,inspect_snapshot,verify_path
from exploration_demo import save_slice

def stamp_key(stamp):return (stamp.sec,stamp.nanosec)

class SensorExecutor(GlobalDemo):
    def __init__(self,speed,provide_action=True):
        super().__init__()
        self.position=[-2.,0.,0.];self.yaw=math.pi/2;self.home=self.position[:]
        self.speed=speed;self.active=None;self.pending_scan=None;self.snapshot={}
        self.snapshot_count=0;self.scan_count=0;self.failures=[];self.mode='local'
        self.actions=[];self.history=[];self.feedback_count=0;self.distance=0.;self.enable_motion=False
        self.clouds=self.create_publisher(PointCloud2,'points',qos_profile_sensor_data)
        self.tf=TransformBroadcaster(self)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PointCloud2,'voxel_map',self.receive_snapshot,qos)
        self.create_subscription(String,'control_status',self.receive_control,qos)
        self.server=(ActionServer(self,PathFollower,'path_follower',execute_callback=self.execute,
            goal_callback=self.accept,cancel_callback=lambda goal:CancelResponse.ACCEPT) if provide_action else None)
        self.control=self.create_client(Trigger,'control_state')
        self.start_client=self.create_client(PciTrigger,'pci_trigger')
        self.mapper=self.create_client(Trigger,'mapper_status')
        self.expansion=self.create_client(Trigger,'expansion_status')
        for client in [self.control,self.start_client,self.mapper,self.expansion]:assert client.wait_for_service(timeout_sec=15)
        self.timer=self.create_timer(.04,self.tick)

    def receive_control(self,message):self.mode=json.loads(message.data)['mode']

    def receive_snapshot(self,message):
        self.snapshot=inspect_snapshot(message);self.snapshot_count+=1
        if self.pending_scan and stamp_key(message.header.stamp)==self.pending_scan['stamp']:
            self.scan_count+=1;self.pending_scan=None
            known=sum(v!=-1 for v in self.snapshot.values())
            self.history.append({'scan':self.scan_count,'x':self.position[0],'y':self.position[1],
                'z':self.position[2],'known_cells':known,'known_fraction':known/19200,
                'distance_m':self.distance,'mode':self.mode})

    def odometry_now(self):
        message=Odometry();message.header.frame_id='map';message.header.stamp=self.get_clock().now().to_msg()
        p=message.pose.pose.position;p.x,p.y,p.z=self.position
        q=message.pose.pose.orientation;q.z=math.sin(self.yaw/2);q.w=math.cos(self.yaw/2)
        self.odometry.publish(message)

    def start_scan(self):
        stamp=self.get_clock().now().to_msg()
        transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='lidar';transform.header.stamp=stamp
        t=transform.transform.translation;t.x,t.y,t.z=self.position
        q=transform.transform.rotation;q.z=math.sin(self.yaw/2);q.w=math.cos(self.yaw/2)
        self.tf.sendTransform(transform)
        self.pending_scan={'stamp':stamp_key(stamp),'transform':transform,
            'cloud':lidar_cloud(stamp,origin=self.position,yaw=self.yaw),
            'started':time.monotonic(),'published':False}

    def accept(self,request):
        if self.active is not None or len(request.path)<2:return GoalResponse.REJECT
        path=xyz([p.pose for p in request.path])
        try:
            assert all(p.header.frame_id=='map' for p in request.path)
            assert math.dist(path[0],self.position)<.1
            verify_path(path,self.snapshot)
        except AssertionError as error:
            self.failures.append('rejected_path:'+str(error));return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def execute(self,handle):
        future=Future();path=xyz([p.pose for p in handle.request.path])
        self.active={'handle':handle,'path':path,'index':1,'future':future,'mode':self.mode,
                     'poses':handle.request.path}
        self.actions.append({'mode':self.mode,'path':path,'status':'executing'})
        success=await future
        result=PathFollower.Result();result.success=success;return result

    def terminate(self,success,reason):
        active=self.active
        if not active:return
        if success:active['handle'].succeed()
        elif active['handle'].is_cancel_requested:active['handle'].canceled()
        else:active['handle'].abort()
        self.actions[-1]['status']=reason;self.active=None;active['future'].set_result(success)

    def tick(self):
        self.odometry_now()
        if self.active and self.active['handle'].is_cancel_requested:self.terminate(False,'cancelled')
        if self.pending_scan:
            scan=self.pending_scan
            if not scan['published']:
                # Send TF first, then the exact-stamp scan on a later tick.
                self.tf.sendTransform(scan['transform']);self.clouds.publish(scan['cloud']);scan['published']=True
            elif time.monotonic()-scan['started']>5:
                self.failures.append('scan_timeout');self.terminate(False,'scan_timeout');self.timer.cancel()
            return
        if not self.snapshot or not self.active or not self.enable_motion:self.start_scan();return
        active=self.active;target=active['path'][active['index']]
        length=math.dist(self.position,target);fraction=min(1.,self.speed*.04/max(length,1e-12))
        next_position=[a+(b-a)*fraction for a,b in zip(self.position,target)]
        try:verify_path([self.position,next_position],self.snapshot)
        except AssertionError as error:
            self.failures.append('motion_collision:'+str(error));self.terminate(False,'collision');return
        self.distance+=math.dist(self.position,next_position);self.position=next_position
        q=active['poses'][active['index']].pose.orientation
        self.yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.odometry_now()
        feedback=PathFollower.Feedback();feedback.remaining_waypoints=len(active['path'])-active['index']
        feedback.dist_to_goal=float(math.dist(self.position,active['path'][-1]))
        feedback.estimated_time_remaining=feedback.dist_to_goal/self.speed
        active['handle'].publish_feedback(feedback);self.feedback_count+=1
        if fraction==1.:
            active['index']+=1
            if active['index']==len(active['path']):self.terminate(True,'succeeded')
        self.start_scan()

    def state(self):return json.loads(self.call(self.control,Trigger.Request()).message)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--start-node',action='store_true')
    parser.add_argument('--speed',type=float,default=6.)
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--output',type=Path,default=Path('results/system_exploration'))
    args=parser.parse_args();process=None;node=None
    if not math.isfinite(args.speed) or not 0<args.speed<=10:raise ValueError('Invalid simulation speed')
    if not 0<=args.seed<=4294967295:raise ValueError('Invalid seed')
    try:
        if args.start_node:process=subprocess.Popen(['ros2','launch','gbplanner_local','control.launch.py',f'seed:={args.seed}'],start_new_session=True)
        rclpy.init();node=SensorExecutor(args.speed)
        visual_client=node.create_client(Trigger,'visualization_status')
        assert visual_client.wait_for_service(timeout_sec=15),'Visualization node missing'
        node.spin_until(lambda:node.clouds.get_subscription_count()>0 and node.snapshot_count>0)
        node.spin_until(lambda:node.state()['map_revision']>0 and node.revision()['known_cells']>0)
        initial_known=node.revision()['known_cells'];args.output.mkdir(parents=True,exist_ok=True)
        save_slice(node.snapshot,args.output/'initial_map.csv')
        node.enable_motion=True;request=PciTrigger.Request();request.set_auto=True;request.bound_mode=PlannerSrv.Request.K_EXACT_BOUND
        assert node.call(node.start_client,request).success
        state={};deadline=time.monotonic()+180
        while True:
            state=node.state()
            if state['state'] in {'DONE','ERROR','STOPPED'}:break
            assert time.monotonic()<deadline,('Pipeline timeout',state)
            assert not node.failures,node.failures
            rclpy.spin_once(node,timeout_sec=.02)
        assert state['state']=='DONE',state
        assert state['reason'] in {'returned_home','already_home'},state
        assert math.dist(node.position,node.home)<=.1,node.position
        assert any(a['mode']=='local' for a in node.actions)
        assert all(a['status']=='succeeded' for a in node.actions),node.actions
        assert not node.failures,node.failures
        assert node.feedback_count>0
        mapper=json.loads(node.call(node.mapper,Trigger.Request()).message)
        expansion=json.loads(node.call(node.expansion,Trigger.Request()).message)
        assert expansion['timer_enabled'] and expansion['updates']>0,expansion
        assert mapper['rejected_scans']==0,mapper
        visual=lambda:json.loads(node.call(visual_client,Trigger.Request()).message)
        node.spin_until(lambda:visual()['known_cells']==mapper['known_cells'])
        visualization=visual()
        assert visualization['local_updates']>0 and visualization['global_updates']>0
        assert visualization['odom_updates']>0 and visualization['history_points']>1
        assert visualization['rejected']==0,visualization
        node.timer.cancel();save_slice(node.snapshot,args.output/'final_map.csv')
        with (args.output/'history.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(node.history[0]));writer.writeheader();writer.writerows(node.history)
        output={'status':'passed','scope':'persistent control + dynamic lidar/TF + occupancy + native planner + test-only action executor',
            'seed':args.seed,'simulation_step_speed_mps':args.speed,
            'control':state,'initial_known_cells':initial_known,'final_known_cells':mapper['known_cells'],
            'total_cells':19200,'distance_m':node.distance,'final_home_error_m':math.dist(node.position,node.home),
            'mapper':mapper,'background':expansion,'visualization':visualization,'actions':node.actions,'feedback_messages':node.feedback_count,
            'checks':{'sensor_only_map_input':True,'controller_manages_all_planning':True,
                'independent_path_and_step_collision_checks':True,'returned_to_home':True,
                'dynamic_tf_same_stamp_as_cloud':True,'no_rejected_scans':True,'live_visualization_updates':True}}
        temp=args.output/'summary.tmp';temp.write_text(json.dumps(output,indent=2)+chr(10));temp.replace(args.output/'summary.json')
        print(json.dumps({k:v for k,v in output.items() if k not in {'actions','control'}},indent=2))
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if process:
            # Let launch forward one orderly shutdown to its own child nodes.
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
            assert process.returncode==0,process.returncode

if __name__=='__main__':main()

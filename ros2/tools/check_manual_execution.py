#!/usr/bin/env python3
"""Short native ROS check of manual PCI commands with a kinematic consumer."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

os.environ['ROS_DOMAIN_ID']='123'
ROOT=Path(__file__).resolve().parents[1]
for package in ('gbplanner_local','gbplanner_execution'):
    sys.path.insert(0,str(ROOT/'src'/package/'scripts'))
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import PoseStamped,TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import MultiDOFJointTrajectory
from tf2_ros import StaticTransformBroadcaster
from planner_msgs.action import PathFollower
from planner_msgs.srv import PciInitialization,PciToWaypoint
from local_demo import make_map
from execution_demo import decode,sample
from path_follower_node import pose
from trajectory_tools import wrap


class Probe(Node):
    def __init__(self):
        super().__init__('manual_execution_probe')
        self.position=(-2.,0.,0.,.45);self.commands=[];self.motion=None
        self.states={};self.execution={};self.map_known=0
        durable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.maps=self.create_publisher(PointCloud2,'voxel_map',durable)
        self.odom=self.create_publisher(Odometry,'odometry',10)
        self.create_subscription(MultiDOFJointTrajectory,'command/trajectory',self.command,10)
        for prefix in ('',):
            self.create_subscription(String,prefix+'control_status',
                lambda msg,p=prefix:self.states.update({p:json.loads(msg.data)}),durable)
        self.create_subscription(String,'execution_status',lambda msg:self.execution.update(json.loads(msg.data)),durable)
        self.create_subscription(String,'map_updates',self.map_update,durable)
        self.rotate=self.create_client(Trigger,'pci_rotate_180_trigger')
        self.waypoint=self.create_client(PciToWaypoint,'pci_to_waypoint')
        self.air_init=self.create_client(PciInitialization,'pci_initialization_trigger')
        self.action=ActionClient(self,PathFollower,'path_follower')
        self.tf=StaticTransformBroadcaster(self)
        self.create_timer(.01,self.tick)

    def command(self,message):
        points,times=decode(message)
        record={'points':points,'times':times};self.commands.append(record)
        self.motion=(points,times,time.monotonic(),self.position)

    def map_update(self,message):
        self.map_known=json.loads(message.data).get('known_cells',0)

    def tick(self):
        if self.motion:
            points,times,started,initial=self.motion
            self.position=sample(points,times,time.monotonic()-started,initial)
        message=Odometry();message.header.frame_id='map';message.child_frame_id='base_link'
        message.header.stamp=self.get_clock().now().to_msg();message.pose.pose=pose(self.position)
        self.odom.publish(message)

    def until(self,predicate,seconds=8.):
        deadline=time.monotonic()+seconds
        while not predicate():
            if time.monotonic()>deadline:raise TimeoutError(str(self.states)+' '+str(self.execution))
            rclpy.spin_once(self,timeout_sec=.02)

    def call(self,client,request):
        future=client.call_async(request);self.until(future.done)
        return future.result()

    def finish_control(self,before,prefix=''):
        self.until(lambda:len(self.commands)>before)
        self.until(lambda:self.states.get(prefix,{}).get('state') in {'DONE','ERROR'})
        assert self.states[prefix]['state']=='DONE',self.states[prefix]
        assert len(self.commands)==before+1,'Unexpected hold or duplicate command'
        return self.commands[-1]

    def speed_goal(self,speed):
        start=self.position;target=(start[0]+.12,*start[1:])
        goal=PathFollower.Goal();goal.manual_path=True;goal.vel_max=speed
        for state in (start,target):
            stamped=PoseStamped();stamped.header.frame_id='map';stamped.pose=pose(state);goal.path.append(stamped)
        before=len(self.commands)
        future=self.action.send_goal_async(goal);self.until(future.done)
        handle=future.result();assert handle.accepted
        result=handle.get_result_async();self.until(result.done)
        assert result.result().result.success,result.result()
        assert len(self.commands)==before+1
        return self.commands[-1]['times'][-1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'results/manual_execution/summary.json')
    args=parser.parse_args();started=time.monotonic();processes=[];node=None
    summary={'status':'failed','domain':123,'consumer':'kinematic trajectory topic consumer; no vehicle dynamics'}
    with tempfile.TemporaryDirectory(prefix='manual_ros_') as temporary:
        logs=[]
        def launch(arguments):
            log=open(Path(temporary)/f'{len(processes)}.log','w+');logs.append(log)
            processes.append(subprocess.Popen(arguments,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        try:
            launch(['ros2','run','gbplanner_local','local_planner_node','--ros-args',
                    '--params-file',str(ROOT/'src/gbplanner_local/config/local.yaml')])
            launch(['ros2','run','gbplanner_execution','path_follower_node.py','--ros-args',
                    '-p','yaw_rate:=4.0','-p','sample_period:=0.02'])
            launch(['ros2','run','gbplanner_local','control_node.py','--ros-args',
                    '-p','init_motion.z_takeoff:=0.08','-p','init_motion.z_drop:=0.03',
                    '-p','init_motion.x_forward:=0.08'])
            rclpy.init(args=[]);node=Probe()
            node.until(lambda:all(client.service_is_ready() for client in (node.rotate,node.waypoint,node.air_init))
                       and node.action.server_is_ready() and node.maps.get_subscription_count()>0,12.)
            node.maps.publish(make_map(known=True,wall=False))
            node.until(lambda:node.map_known>0)
            # Let subscribers observe several identical stamped poses first.
            ready_at=time.monotonic()+.15;node.until(lambda:time.monotonic()>ready_at)

            before=len(node.commands);start=node.position
            reply=node.call(node.rotate,Trigger.Request());assert reply.success,reply.message
            command=node.finish_control(before)
            assert abs(wrap(command['points'][-1][3]-math.pi))<1e-9
            assert all(math.dist(p[:3],start[:3])<1e-9 for p in command['points'])
            summary['rotate_180']={'start_yaw':start[3],'target_yaw':command['points'][-1][3],
                                  'absolute_pi':True,'duration_s':command['times'][-1]}

            # One translated/rotated static TF also checks the full target yaw.
            transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='manual_target_frame'
            transform.transform.translation.x=node.position[0]+.08
            transform.transform.translation.y=node.position[1]
            transform.transform.translation.z=node.position[2]
            transform.transform.rotation.z=transform.transform.rotation.w=math.sqrt(.5)
            node.tf.sendTransform(transform)
            ready_at=time.monotonic()+.15;node.until(lambda:time.monotonic()>ready_at)
            target=(node.position[0]+.08,node.position[1]+.04,node.position[2],math.pi/2-1.)
            request=PciToWaypoint.Request();request.header.frame_id='manual_target_frame';request.waypoint=pose((.04,0.,0.,-1.))
            before=len(node.commands);node.call(node.waypoint,request);command=node.finish_control(before)
            assert math.dist(command['points'][-1][:3],target[:3])<1e-8
            assert abs(wrap(command['points'][-1][3]-target[3]))<1e-8
            summary['direct_waypoint']={'tf_applied':True,'target_yaw_preserved':True,'endpoint':list(command['points'][-1])}

            before=len(node.commands);start=node.position
            assert node.call(node.air_init,PciInitialization.Request()).success
            command=node.finish_control(before)
            expected=[(start[0],start[1],start[2]+.08),(start[0],start[1],start[2]+.05),
                      (start[0]+.08,start[1],start[2]+.05)]
            cursor=0
            for point in expected:
                while cursor<len(command['points']) and math.dist(command['points'][cursor][:3],point)>1e-8:cursor+=1
                assert cursor<len(command['points']),('Initialization corner missing',point)
            assert abs(wrap(command['points'][-1][3]))<1e-8
            summary['aerial_initialization']={'up_m':.08,'drop_m':.03,'forward_world_x_m':.08,'all_corners_preserved':True,'completed':True}

            # Reuse the same original PCI names for its ground factory config.
            os.killpg(processes[2].pid,signal.SIGINT);processes[2].wait(timeout=2.)
            launch(['ros2','run','gbplanner_local','control_node.py','--ros-args','-p','robot_type:=kGround'])
            node.until(lambda:node.states.get('',{}).get('initialization',{}).get('robot_type')=='kGround')
            node.until(lambda:node.air_init.service_is_ready())
            ready_at=time.monotonic()+.3;node.until(lambda:time.monotonic()>ready_at)
            before=len(node.commands);start=node.position
            reply=node.call(node.air_init,PciInitialization.Request())
            assert reply.success,node.states
            command=node.finish_control(before)
            assert math.dist(command['points'][-1][:3],(start[0]+3.,start[1],start[2]))<1e-8
            assert abs(wrap(command['points'][-1][3]-start[3]))<1e-8
            summary['ground_initialization']={'default_forward_world_x_m':3.,'height_and_yaw_preserved':True,
                                              'scope':'ground factory path with shared occupancy validator and kinematic executor'}

            slow=node.speed_goal(.2);fast=node.speed_goal(1.)
            assert slow>3.5*fast,(slow,fast)
            summary['velocity']={'distance_m':.12,'slow_mps':.2,'slow_duration_s':slow,
                                 'fast_mps':1.,'fast_duration_s':fast}
            summary['status']='passed';summary['trajectory_commands']=len(node.commands)
        except Exception as error:
            summary['error']=f'{type(error).__name__}: {error}'
            for log in logs:
                log.flush();log.seek(0);content=log.read()
                if 'Traceback' in content:summary.setdefault('process_errors',[]).append(content[-2000:])
        finally:
            if node is not None:node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
            for process in processes:
                if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
            for process in processes:
                try:process.wait(timeout=2.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=1.)
            for log in logs:log.close()
            summary['elapsed_s']=round(time.monotonic()-started,3)
            summary['processes_stopped']=all(p.poll() is not None for p in processes)
            args.output.parent.mkdir(parents=True,exist_ok=True)
            args.output.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
            print(json.dumps(summary,ensure_ascii=False))
    return 0 if summary['status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())

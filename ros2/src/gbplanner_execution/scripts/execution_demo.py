#!/usr/bin/env python3
"""Test-only topic consumer for the separate execution action process."""
import argparse
import bisect
import copy
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
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory
from planner_msgs.action import PathFollower
from std_srvs.srv import Trigger
from trajectory_tools import state_from_pose, wrap
sys.path.insert(0, str(Path(get_package_prefix('gbplanner_local'))/'lib/gbplanner_local'))
from global_demo import GlobalDemo
from local_demo import make_map
from path_follower_node import pose


def decode(message):
    assert message.header.frame_id == 'map' and message.joint_names == ['base_link']
    points, times = [], []
    for point in message.points:
        assert len(point.transforms) == 1
        transform = point.transforms[0]
        stamped = pose((transform.translation.x, transform.translation.y, transform.translation.z, 0.))
        stamped.orientation = transform.rotation
        points.append(state_from_pose(stamped))
        times.append(point.time_from_start.sec + point.time_from_start.nanosec*1e-9)
    assert points and all(t > 0 for t in times)
    assert all(b > a for a,b in zip(times,times[1:]))
    return points,times


def sample(points, times, elapsed, initial):
    index=bisect.bisect_right(times,elapsed)
    if index>=len(points):return points[-1]
    start=initial if index==0 else points[index-1]
    previous=0. if index==0 else times[index-1]
    fraction=max(0.,min(1.,(elapsed-previous)/(times[index]-previous)))
    end=points[index]
    return (*[a+(b-a)*fraction for a,b in zip(start[:3],end[:3])],wrap(start[3]+wrap(end[3]-start[3])*fraction))


class Consumer(GlobalDemo):
    def __init__(self):
        super().__init__()
        self.position=(-2.,0.,0.,0.)
        self.commands=[];self.active_command=None;self.errors=[]
        self.publish_odometry=True;self.follow=True;self.feedback=[]
        self.odom_frame='map'
        self.action=ActionClient(self,PathFollower,'path_follower')
        self.execution=self.create_client(Trigger,'execution_state')
        self.subscription=self.create_subscription(MultiDOFJointTrajectory,'command/trajectory',self.command,10)
        self.timer=self.create_timer(.02,self.tick)
        assert self.action.wait_for_server(timeout_sec=15)
        assert self.execution.wait_for_service(timeout_sec=15)

    def state(self):return json.loads(self.call(self.execution,Trigger.Request()).message)

    def command(self,message):
        try:
            points,times=decode(message)
            for a,b,ta,tb in zip(points,points[1:],times,times[1:]):
                assert math.dist(a[:3],b[:3])/(tb-ta)<=1.000001
                assert abs(wrap(b[3]-a[3]))/(tb-ta)<=1.000001
            self.commands.append({'points':points,'times':times,'received':time.monotonic()})
            self.active_command=(points,times,time.monotonic(),self.position)
        except (AssertionError,ValueError) as error:self.errors.append(str(error))

    def tick(self):
        if self.active_command and self.follow:
            points,times,started,initial=self.active_command
            self.position=sample(points,times,time.monotonic()-started,initial)
        if self.publish_odometry:
            odom=Odometry();odom.header.frame_id=self.odom_frame;odom.header.stamp=self.get_clock().now().to_msg()
            odom.pose.pose=pose(self.position);self.odometry.publish(odom)

    def goal(self,path):
        request=PathFollower.Goal()
        for state in path:
            stamped=PoseStamped();stamped.header.frame_id='map';stamped.pose=pose(state);request.path.append(stamped)
        return request

    def send(self,request):
        future=self.action.send_goal_async(request,feedback_callback=lambda m:self.feedback.append(m.feedback))
        self.spin_until(future.done);return future.result()

    def result(self,handle,seconds=12):
        future=handle.get_result_async();self.spin_until(future.done,seconds=seconds);return future.result()

    def settle(self):
        target=self.position;deadline=time.monotonic()+.15
        while time.monotonic()<deadline:rclpy.spin_once(self,timeout_sec=.02)
        self.spin_until(lambda:self.state()['command_subscribers']>0)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--start-node',action='store_true')
    parser.add_argument('--output',type=Path,default=Path('results/execution_integration.json'))
    args=parser.parse_args();processes=[];node=None
    try:
        if args.start_node:
            processes.append(subprocess.Popen(['ros2','run','gbplanner_local','local_planner_node',
                '--ros-args','-p','global.enable:=true'],start_new_session=True))
            processes.append(subprocess.Popen(['ros2','run','gbplanner_execution','path_follower_node.py',
                '--ros-args','-p','yaw_rate:=1.0','-p','odom_timeout:=0.4','-p','execution_grace:=0.4'],start_new_session=True))
        rclpy.init();node=Consumer()
        assert not node.send(node.goal([node.position,(-1.7,0.,0.,0.)])).accepted
        node.publish_map(make_map(known=True,wall=False));node.settle()
        start=node.position;goal=node.goal([start,(-1.6,.2,0.,.4),(-1.2,.2,0.,.6)])
        handle=node.send(goal);assert handle.accepted
        node.spin_until(lambda:node.state()['state']=='EXECUTING')
        assert not node.send(goal).accepted,'Concurrent goals must be rejected'
        result=node.result(handle);assert result.status==GoalStatus.STATUS_SUCCEEDED and result.result.success
        assert math.dist(node.position[:3],(-1.2,.2,0.))<.05 and node.feedback and not node.errors,node.errors
        first=node.commands[0]
        assert first['times'][0]==.05 and math.dist(first['points'][-1][:3],(-1.2,.2,0.))<1e-12
        bad=node.goal([node.position,(-1.,0.,0.,0.)]);bad.path[-1].header.frame_id='odom'
        assert not node.send(bad).accepted
        bad=node.goal([node.position]);bad.path[0].pose.orientation.w=0.;bad.path[0].pose.orientation.z=0.
        assert not node.send(bad).accepted
        assert not node.send(node.goal([(3.,0.,0.,0.),(4.,0.,0.,0.)])).accepted

        # A connected command still crosses a full-height wall: validation
        # must abort before publishing any motion to the external consumer.
        before=len(node.commands);node.publish_map(make_map(known=True,wall=True))
        handle=node.send(node.goal([node.position,(1.,2.,0.,0.)]));assert handle.accepted
        result=node.result(handle);assert result.status==GoalStatus.STATUS_ABORTED
        assert len(node.commands)==before
        node.publish_map(make_map(known=True,wall=False))

        handle=node.send(node.goal([node.position,(2.,.2,0.,0.)]));assert handle.accepted
        node.spin_until(lambda:node.state()['state']=='EXECUTING' and len(node.commands)>before)
        cancel=handle.cancel_goal_async();node.spin_until(cancel.done)
        assert cancel.result().goals_canceling
        result=node.result(handle);assert result.status==GoalStatus.STATUS_CANCELED
        assert len(node.commands[-1]['points'])==1 and node.state()['hold_commands']>=1

        # Odom loss prevents claiming a successful hold or a reached endpoint.
        node.settle();handle=node.send(node.goal([node.position,(2.,.2,0.,0.)]));assert handle.accepted
        node.spin_until(lambda:node.state()['state']=='EXECUTING')
        node.publish_odometry=False
        result=node.result(handle);assert result.status==GoalStatus.STATUS_ABORTED
        assert 'odometry_lost' in node.state()['reason'],node.state()
        node.publish_odometry=True;node.settle()

        # A consumer that accepts but never moves cannot cause action success.
        node.follow=False
        end=(node.position[0]+.35,node.position[1],node.position[2],node.position[3])
        handle=node.send(node.goal([node.position,end]));assert handle.accepted
        result=node.result(handle);assert result.status==GoalStatus.STATUS_ABORTED
        assert 'endpoint_timeout' in node.state()['reason'],node.state()
        node.follow=True;node.settle()

        # New obstacles invalidate an already published command and cause a
        # hold. The injected wall is ahead of the current footprint.
        handle=node.send(node.goal([node.position,(2.,.2,0.,0.)]));assert handle.accepted
        node.spin_until(lambda:node.state()['state']=='EXECUTING')
        blocked=make_map(known=True,wall=False);data=bytearray(blocked.data)
        for i in range(0,len(data),blocked.point_step):
            if .25 < struct.unpack_from('<f',data,i)[0] < .75:data[i+12]=100
        blocked.data=bytes(data);node.publish_map(blocked)
        result=node.result(handle);assert result.status==GoalStatus.STATUS_ABORTED
        assert node.state()['reason']=='path_invalidated:path_not_free',node.state()
        assert len(node.commands[-1]['points'])==1
        node.publish_map(make_map(known=True,wall=False));node.settle()

        # Invalid incoming odometry must still replace the command with a
        # hold at the last valid pose; success cannot be inferred without data.
        before=node.state()['hold_commands']
        handle=node.send(node.goal([node.position,(2.,.2,0.,0.)]));assert handle.accepted
        node.spin_until(lambda:node.state()['state']=='EXECUTING')
        node.odom_frame='wrong_frame'
        result=node.result(handle);assert result.status==GoalStatus.STATUS_ABORTED
        assert node.state()['hold_commands']>before
        node.odom_frame='map';node.settle()

        # Explicitly starting again after failure works with fresh inputs.
        end=(node.position[0]+.2,node.position[1],node.position[2],node.position[3])
        handle=node.send(node.goal([node.position,end]));assert handle.accepted
        result=node.result(handle);assert result.status==GoalStatus.STATUS_SUCCEEDED

        # Mid-execution map reset sends a hold and aborts the action.
        handle=node.send(node.goal([node.position,(2.,.2,0.,0.)]));assert handle.accepted
        node.spin_until(lambda:node.state()['state']=='EXECUTING')
        unknown=make_map(known=True,wall=False);data=bytearray(unknown.data)
        for i in range(12,len(data),unknown.point_step):data[i]=255
        unknown.data=bytes(data);node.publish_map(unknown)
        result=node.result(handle);assert result.status==GoalStatus.STATUS_ABORTED
        assert node.state()['reason']=='map_reset',node.state()
        assert not node.errors,node.errors
        output={'status':'passed','scope':'Separate native trajectory action process and test-only timed-topic consumer',
            'first_trajectory_points':len(first['points']),'first_duration_s':first['times'][-1],
            'final':node.state(),'checks':{'trajectory_endpoint_and_timing':True,'linear_and_yaw_limits':True,
            'actual_odometry_required':True,'concurrent_goal_rejected':True,'invalid_goal_rejected':True,
            'collision_before_publication':True,'cancel_publishes_hold_and_waits_for_odometry':True,
            'odometry_loss_aborts':True,'stuck_consumer_aborts':True,'map_reset_aborts':True,
            'obstacle_during_motion_sends_hold':True,'invalid_odometry_uses_last_valid_hold':True,
            'explicit_restart_after_failure':True}}
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for process in reversed(processes):
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
            assert process.returncode==0,process.returncode
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(output,indent=2)+chr(10));print(json.dumps(output,indent=2))


if __name__=='__main__':main()

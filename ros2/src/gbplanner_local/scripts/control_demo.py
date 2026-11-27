#!/usr/bin/env python3
"""Exercise the control node through a separate synthetic action executor.

The action server lives only in this test. It checks the map, interpolates
kinematic poses, publishes odometry and observations, and supports cancellation.
It is not a port of the robot-specific PCIManager or a real controller.
"""
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
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.task import Future
from nav_msgs.msg import Odometry
from planner_msgs.action import PathFollower
from planner_msgs.srv import PciTrigger, PciStop, PciHomingTrigger, PlannerSrv
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
from global_demo import GlobalDemo
from local_demo import make_map,xyz
from mapping_demo import inspect_snapshot,verify_path

class ExecutorDemo(GlobalDemo):
    def __init__(self):
        super().__init__()
        self.position=[-2.,0.,0.];self.snapshot_message=make_map()
        self.snapshot=inspect_snapshot(self.snapshot_message)
        self.reject=False;self.hold=False;self.publish_pose=True;self.abort=False;self.publish_maps=True
        self.active=None;self.goals=[];self.cancelled=0;self.feedback_count=0
        self.server=ActionServer(self,PathFollower,'path_follower',execute_callback=self.execute,
            goal_callback=self.accept,cancel_callback=lambda goal:CancelResponse.ACCEPT)
        self.timer=self.create_timer(.02,self.step)
        self.control=self.create_client(Trigger,'control_state')
        self.start_client=self.create_client(PciTrigger,'pci_trigger')
        self.stop_client=self.create_client(PciStop,'pci_stop')
        self.home_client=self.create_client(PciHomingTrigger,'pci_homing_trigger')
        for client in [self.control,self.start_client,self.stop_client,self.home_client]:
            assert client.wait_for_service(timeout_sec=15)

    def accept(self,request):
        if self.reject or self.active is not None or len(request.path)<2:return GoalResponse.REJECT
        if any(p.header.frame_id!='map' for p in request.path):return GoalResponse.REJECT
        path=xyz([p.pose for p in request.path]);verify_path(path,self.snapshot)
        assert math.dist(path[0],self.position)<.11
        return GoalResponse.ACCEPT

    async def execute(self,handle):
        path=xyz([p.pose for p in handle.request.path]);future=Future()
        self.active={'handle':handle,'path':path,'index':1,'future':future}
        self.goals.append(path)
        success=await future
        result=PathFollower.Result();result.success=success
        return result

    def publish_odometry(self):
        odom=Odometry();odom.header.frame_id='map';odom.header.stamp=self.get_clock().now().to_msg()
        p=odom.pose.pose.position;p.x,p.y,p.z=self.position;odom.pose.pose.orientation.w=1.
        self.odometry.publish(odom)

    def step(self):
        if self.publish_pose:self.publish_odometry()
        active=self.active
        if active is None:
            # Supply an observation after completion, while the vehicle waits.
            if self.publish_maps:self.maps.publish(self.snapshot_message)
            return
        handle=active['handle']
        if handle.is_cancel_requested:
            handle.canceled();self.cancelled+=1;self.active=None;active['future'].set_result(False);return
        if self.abort:
            handle.abort();self.active=None;active['future'].set_result(False);return
        if self.hold:return
        target=active['path'][active['index']];distance=math.dist(target,self.position)
        fraction=min(1.,.15/max(distance,1e-12))
        self.position=[a+(b-a)*fraction for a,b in zip(self.position,target)]
        self.publish_odometry()
        if self.publish_maps:self.maps.publish(self.snapshot_message)
        feedback=PathFollower.Feedback();feedback.remaining_waypoints=len(active['path'])-active['index']
        feedback.dist_to_goal=float(math.dist(self.position,active['path'][-1]));feedback.estimated_time_remaining=feedback.dist_to_goal/7.5
        handle.publish_feedback(feedback);self.feedback_count+=1
        if fraction==1.:
            active['index']+=1
            if active['index']==len(active['path']):
                handle.succeed();self.active=None;active['future'].set_result(True)

    def control_status(self):return json.loads(self.call(self.control,Trigger.Request()).message)

    def publish_map(self,message):
        # Isolate deliberate fixture replacement from periodic observations.
        # Otherwise several accepted revisions can overtake an exact +1 wait.
        periodic=self.publish_maps;self.publish_maps=False
        try:
            deadline=time.monotonic()+.1
            while time.monotonic()<deadline:rclpy.spin_once(self,timeout_sec=.02)
            super().publish_map(message)
        finally:self.publish_maps=periodic

    def wait_state(self,states,seconds=20):
        value={}
        def done():
            value.update(self.control_status());return value['state'] in states
        self.spin_until(done,seconds);return value

    def trigger_control(self,auto=False,preview=False,velocity=0.,bound=PlannerSrv.Request.K_EXACT_BOUND):
        request=PciTrigger.Request();request.set_auto=auto;request.not_exe_path=preview
        request.bound_mode=bound;request.vel_max=velocity
        return self.call(self.start_client,request).success

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--start-node',action='store_true')
    parser.add_argument('--output',type=Path,default=Path('results/control_integration.json'))
    args=parser.parse_args();processes=[];node=None
    try:
        if args.start_node:
            config=Path(get_package_share_directory('gbplanner_local'))/'config/local.yaml'
            processes.append(subprocess.Popen(['ros2','run','gbplanner_local','local_planner_node','--ros-args','--params-file',str(config),'-p','global.enable:=true'],start_new_session=True))
            processes.append(subprocess.Popen(['ros2','run','gbplanner_local','control_node.py','--ros-args','-p','max_cycles:=3'],start_new_session=True))
        rclpy.init();node=ExecutorDemo();node.publish_map(node.snapshot_message)
        node.spin_until(lambda:node.odometry.get_subscription_count()>=2)
        node.move_odometry(node.position)
        node.spin_until(lambda:node.control_status()['map_revision']>0)
        assert not node.trigger_control(velocity=math.inf),'Non-finite velocity accepted'
        assert not node.trigger_control(bound=99),'Invalid bound accepted'
        assert not node.trigger_control(auto=True,preview=True),'Automatic preview loop accepted'
        before=len(node.goals);assert node.trigger_control(preview=True)
        preview=node.wait_state({'PREVIEW','ERROR'});assert preview['state']=='PREVIEW',preview
        assert len(node.goals)==before
        assert node.trigger_control(auto=True)
        automatic=node.wait_state({'DONE','ERROR'});assert automatic['state']=='DONE' and automatic['reason']=='returned_home',automatic
        assert automatic['completed_actions']==4,automatic
        assert math.dist(node.position,[-2,0,0])<1e-8
        assert node.feedback_count>0
        auto_goals=node.goals[:]
        # Default policy stops after the planner's four no-frontier rounds;
        # an empty local response alone must not request global or homing.
        node.snapshot_message=make_map(known=True);node.snapshot=inspect_snapshot(node.snapshot_message)
        node.publish_map(node.snapshot_message)
        assert node.trigger_control(auto=True)
        exhausted=node.wait_state({'STOPPED','ERROR'})
        assert exhausted['reason']=='global_frontier_unavailable',exhausted
        requests=[e['reason'] for e in exhausted['events'] if e['reason'].endswith('_requested')]
        assert requests==['local_requested']*4,requests
        node.snapshot_message=make_map();node.snapshot=inspect_snapshot(node.snapshot_message);node.publish_map(node.snapshot_message)
        node.hold=True;assert node.trigger_control(auto=True)
        running=node.wait_state({'EXECUTING','ERROR'});assert running['state']=='EXECUTING',running
        assert not node.trigger_control(),'Concurrent task accepted'
        assert node.call(node.stop_client,PciStop.Request()).success
        stopped=node.wait_state({'STOPPED','ERROR'});assert stopped['state']=='STOPPED',stopped
        assert node.cancelled==1 and node.active is None
        goal_count=len(node.goals)
        deadline=time.monotonic()+.3
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
        assert len(node.goals)==goal_count,'Stopped auto task restarted'
        node.hold=False;node.reject=True;assert node.trigger_control()
        rejected=node.wait_state({'ERROR'});assert rejected['reason']=='executor_rejected',rejected
        node.reject=False;node.abort=True;assert node.trigger_control()
        aborted=node.wait_state({'ERROR'});assert aborted['reason']=='execution_failed',aborted
        node.abort=False;assert node.trigger_control()
        restarted=node.wait_state({'DONE','ERROR'});assert restarted['state']=='DONE' and restarted['reason']=='manual_path_complete',restarted
        node.hold=True;assert node.trigger_control()
        assert node.wait_state({'EXECUTING','ERROR'})['state']=='EXECUTING'
        node.publish_pose=False
        stale=node.wait_state({'ERROR'},seconds=8)
        assert stale['reason']=='odometry_lost_during_execution',stale
        assert node.cancelled==2
        assert not node.trigger_control(),'Stale pose was accepted for a new task'
        node.publish_pose=True;node.hold=False;node.publish_odometry()
        node.spin_until(lambda:node.active is None)
        # Explicit homing request with preview is also routed through PCI.
        request=PciHomingTrigger.Request();request.not_exe_path=True
        deadline=time.monotonic()+5
        while not node.call(node.home_client,request).success:
            assert time.monotonic()<deadline;rclpy.spin_once(node,timeout_sec=.02)
        explicit_home=node.wait_state({'DONE','PREVIEW','ERROR'})
        assert explicit_home['state'] in {'DONE','PREVIEW'},explicit_home
        # Rejected snapshots cannot satisfy the controller's observation gate.
        node.publish_maps=False
        deadline=time.monotonic()+.1
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
        prior=node.control_status()['map_revision']
        invalid=copy.deepcopy(node.snapshot_message);invalid.data=bytes(invalid.data)[:-1]
        node.reject_map(invalid)
        assert node.control_status()['map_revision']==prior
        # Reset during an active action cancels it and prevents another task.
        node.hold=True;assert node.trigger_control()
        assert node.wait_state({'EXECUTING','ERROR'})['state']=='EXECUTING'
        empty=copy.deepcopy(node.snapshot_message);data=bytearray(empty.data)
        for offset in range(12,len(data),empty.point_step):data[offset]=255
        empty.data=bytes(data);node.publish_map(empty)
        reset=node.wait_state({'ERROR'});assert reset['reason']=='map_reset_during_task',reset
        assert node.cancelled==3 and not node.trigger_control()
        # A new obstacle on a valid route must cancel the active goal too.
        node.publish_map(node.snapshot_message);node.publish_odometry()
        node.spin_until(lambda:node.control_status()['known_cells']>0)
        assert node.trigger_control()
        assert node.wait_state({'EXECUTING','ERROR'})['state']=='EXECUTING'
        a,b=node.active['path'][:2];mid=[(x+y)/2 for x,y in zip(a,b)]
        obstacle=copy.deepcopy(node.snapshot_message);data=bytearray(obstacle.data)
        key=[math.floor((mid[i]-(-5,-5,-1.5)[i])/.25) for i in range(3)]
        offset=(key[0]+40*(key[1]+40*key[2]))*16;data[offset+12]=100
        obstacle.data=bytes(data);node.publish_map(obstacle)
        invalidated=node.wait_state({'ERROR'});assert invalidated['reason'].startswith('path_invalidated:'),invalidated
        assert node.cancelled==4
        output={'status':'passed','scope':'ROS 2 control state machine + PathFollower action client + test-only kinematic action server',
                'automatic':automatic,'automatic_action_paths':auto_goals,'exhausted':exhausted,'stopped':stopped,
                'rejected':rejected,'aborted':aborted,'restarted':restarted,'stale_odometry':stale,'map_reset':reset,'path_invalidated':invalidated,'action_feedback_messages':node.feedback_count,
                'cancelled_actions':node.cancelled,'checks':{'preview_sends_no_action':True,
                'automatic_local_then_homing':True,'no_frontier_respects_planner_policy':True,
                'concurrent_request_rejected':True,'nonfinite_velocity_and_invalid_bounds_rejected':True,
                'action_feedback_received':True,'stop_cancels_active_action':True,
                'stop_does_not_restart':True,'goal_rejection_is_terminal':True,
                'action_abort_is_terminal':True,'explicit_restart_after_failure':True,
                'rejected_map_not_counted_as_observation':True,'map_reset_cancels_action':True,
                'new_obstacle_cancels_action':True,
                'lost_odometry_cancels_action':True,'stale_pose_blocks_new_task':True,
                'independent_action_path_collision_checks':True}}
        args.output.parent.mkdir(parents=True,exist_ok=True)
        temp=args.output.with_suffix('.tmp');temp.write_text(json.dumps(output,indent=2)+chr(10));temp.replace(args.output)
        print(json.dumps(output,indent=2))
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for process in reversed(processes):
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
            assert process.returncode==0,('Child process did not shut down cleanly',process.returncode)

if __name__=='__main__':main()

#!/usr/bin/env python3
"""Targeted native action continuation exercise with measured topic consumer."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import rclpy
from action_msgs.msg import GoalStatus
from planner_msgs.srv import ContinuePath, PlanAhead
from execution_demo import Consumer, pose
from local_demo import make_map


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,default=Path('results/plan_ahead/continuation.json'))
    parser.add_argument('--profile', choices=('upstream','quintic','quintic_collinear'), default='upstream')
    args=parser.parse_args();processes=[];node=None;report={'status':'failed'}
    try:
        for command in [
                ['ros2','run','gbplanner_local','local_planner_node','--ros-args','-p','global.enable:=true',
                 '-p','mission.enabled:=true','-p','mission.time_limit:=480.0','-p','mission.homing_speed:=1.0'],
                ['ros2','run','gbplanner_execution','path_follower_node.py','--ros-args','-p','yaw_rate:=1.0',
                 '-p','trajectory_profile:='+args.profile]]:
            processes.append(subprocess.Popen(command,start_new_session=True))
        rclpy.init();node=Consumer();continuation=node.create_client(ContinuePath,'continue_path')
        ahead=node.create_client(PlanAhead,'planner_ahead')
        assert continuation.wait_for_service(timeout_sec=15) and ahead.wait_for_service(timeout_sec=15)
        node.publish_map(make_map(known=True,wall=False));node.settle()
        start=node.position;old_end=(1.,0.,0.,0.);final_end=(1.,1.,0.,0.)
        handle=node.send(node.goal([start,(-.5,0.,0.,0.),old_end]));assert handle.accepted
        result=handle.get_result_async()
        node.spin_until(lambda:node.position[0]>-1.7 and node.state()['state']=='EXECUTING')
        request=ContinuePath.Request();request.header.frame_id='map';request.goal_id=handle.goal_id.uuid
        request.expected_generation=0;request.budget_enabled=True;request.required_time_after_root=30.;request.path=[pose(old_end),pose(final_end)]
        before=node.position;started=time.monotonic();response=node.call(continuation,request)
        assert response.accepted,response.reason
        node.spin_until(lambda:len(node.commands)>=2)
        assert not result.done(),'Extension must not complete the action at splice time'
        committed=node.state();commit_latency=time.monotonic()-started;commit_position=node.position
        assert committed['continuations']==1 and committed['hold_commands']==0,committed
        if args.profile!='upstream':
            splice=committed['last_continuation']
            assert math.hypot(*splice['initial_reference_velocity'][:3])>.05,splice
            assert abs(splice['initial_reference_velocity'][0]-splice['measured_velocity'][0])<1e-9,splice
            assert splice['new_duration_s']==committed['topic_timing']['duration_s'],splice
        assert math.dist(node.position[:3],old_end[:3])>1.5,'Exercise must use the far suffix branch'
        assert response.generation==1 and math.dist((response.path[-1].position.x,response.path[-1].position.y,response.path[-1].position.z),final_end[:3])<1e-10
        # A delayed retry must not append the same segment again.
        stale=node.call(continuation,request)
        assert not stale.accepted and stale.reason=='stale_generation',stale.reason
        node.spin_until(lambda:node.position[1]>.3)
        assert not result.done(),'Crossing the old endpoint is not action success'
        node.spin_until(result.done,seconds=15)
        assert result.result().status==GoalStatus.STATUS_SUCCEEDED and result.result().result.success
        assert math.dist(node.position[:3],final_end[:3])<.05 and node.state()['hold_commands']==0
        first={'request_position':before,'reply_position':tuple(commit_position),
               'commit_latency_s':commit_latency,'generation':response.generation,
               'command_points':[len(command['points']) for command in node.commands],
               'executor':node.state()}
        # Stop, then reject a late continuation bound to the canceled goal.
        handle2=node.send(node.goal([node.position,(-2.,1.,0.,0.)]));assert handle2.accepted
        node.spin_until(lambda:node.state()['state']=='EXECUTING' and node.position[0]<.85)
        cancel=handle2.cancel_goal_async();node.spin_until(cancel.done)
        stopped=node.result(handle2);assert stopped.status==GoalStatus.STATUS_CANCELED
        request.goal_id=handle2.goal_id.uuid;request.expected_generation=0
        request.path=[pose((-2.,1.,0.,0.)),pose((-2.,2.,0.,0.))]
        count=len(node.commands);late=node.call(continuation,request)
        assert not late.accepted and len(node.commands)==count
        # A newer map blocks only a speculative extension. Existing safe route
        # must still reach its old endpoint; rejection itself emits no hold.
        node.settle();old_end=(-2.,1.,0.,0.)
        handle3=node.send(node.goal([node.position,old_end]));assert handle3.accepted
        node.spin_until(lambda:node.state()['state']=='EXECUTING')
        blocked=make_map(known=True,wall=False);data=bytearray(blocked.data)
        for i in range(0,len(data),blocked.point_step):
            x,y,z=struct.unpack_from('<fff',data,i)
            if y>1.75:data[i+12]=100
        blocked.data=bytes(data);node.publish_map(blocked)
        request.goal_id=handle3.goal_id.uuid;request.path=[pose(old_end),pose((-2.,3.,0.,0.))]
        holds=node.state()['hold_commands'];rejected=node.call(continuation,request)
        assert not rejected.accepted and rejected.reason=='extension_invalid:path_not_free',rejected.reason
        final=node.result(handle3)
        assert final.status==GoalStatus.STATUS_SUCCEEDED and node.state()['hold_commands']==holds
        # Mission budget is honored for explicit future roots before planning.
        budget=PlanAhead.Request();budget.header.frame_id='map';budget.root_pose=pose(node.position)
        budget.arrival_time=1000.
        rejected_budget=node.call(ahead,budget)
        assert not rejected_budget.success and rejected_budget.homing_required,rejected_budget.reason
        assert rejected_budget.reason=='plan_ahead_time_insufficient'
        assert not node.errors,node.errors
        report={'status':'passed','scope':'Native planner/follower with deterministic kinematic odometry; no physical flight',
                'trajectory_profile':args.profile,
                'continuation':first,'final_executor':node.state(),
                'checks':{'replacement_while_moving':True,'no_hold_during_normal_continuation':True,
                          'old_endpoint_does_not_complete_action':True,'final_endpoint_required':True,
                          'stale_generation_rejected':True,'cancel_then_late_reply_rejected':True,
                          'blocked_extension_keeps_old_safe_trajectory':True,'future_root_budget_enforced':True},
                'commands':node.commands}
    except Exception as error:
        report['error']=repr(error)
        if node:report['executor_at_error']=node.state()
        raise
    finally:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for process in reversed(processes):
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
    print(json.dumps({k:v for k,v in report.items() if k!='commands'},indent=2))


if __name__=='__main__':main()

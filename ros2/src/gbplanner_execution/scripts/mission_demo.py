#!/usr/bin/env python3
"""Mission budget integration with independent planning/control/execution nodes.

Battery estimates and trajectory consumption are explicit test fixtures. The
native controller, not this test, interprets K_HOMING and cancels stale missions.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from ament_index_python.packages import get_package_share_directory
from planner_msgs.msg import RobotStatus
from planner_msgs.srv import PciTrigger
from std_srvs.srv import Trigger
from pci_global_demo import GlobalConsumer
from local_demo import make_map, xyz


class MissionConsumer(GlobalConsumer):
    def __init__(self):
        super().__init__()
        self.battery=self.create_publisher(RobotStatus,'robot_status',10)
        self.mission=self.create_client(Trigger,'mission_state')
        assert self.mission.wait_for_service(timeout_sec=15)
        self.reported_remaining=120.
        self.battery_enabled=False
        self.battery_timer=self.create_timer(.05,self.report_battery)

    def report_battery(self):
        if not self.battery_enabled:return
        message=RobotStatus();message.header.stamp=self.get_clock().now().to_msg()
        message.time_remaining=self.reported_remaining;self.battery.publish(message)

    def budget(self):return json.loads(self.call(self.mission,Trigger.Request()).message)

    def set_battery(self,seconds):
        self.reported_remaining=float(seconds);self.battery_enabled=True;self.report_battery()
        self.spin_until(lambda:self.budget()['valid'] and abs(self.budget()['remaining_s']-seconds)<.4)
        # Also wait for the control node's periodic mission subscription.
        deadline=time.monotonic()+.2
        while time.monotonic()<deadline:rclpy.spin_once(self,timeout_sec=.02)

    def trigger_local(self):
        request=PciTrigger.Request();request.bound_mode=3;request.set_auto=True
        return self.call(self.pci_local,request).success


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--start-node',action='store_true')
    parser.add_argument('--output',type=Path,default=Path('results/mission_integration.json'))
    args=parser.parse_args();processes=[];node=None;output=None
    try:
        if args.start_node:
            config=Path(get_package_share_directory('gbplanner_local'))/'config/local.yaml'
            commands=[['ros2','run','gbplanner_local','local_planner_node','--ros-args','--params-file',str(config),
                       '-p','global.enable:=true','-p','mission.enabled:=true','-p','mission.require_battery:=true',
                       '-p','mission.battery_timeout:=0.7'],
                      ['ros2','run','gbplanner_local','control_node.py','--ros-args','-p','require_mission_status:=true'],
                      ['ros2','run','gbplanner_execution','path_follower_node.py']]
            for command in commands:processes.append(subprocess.Popen(command,start_new_session=True))
        rclpy.init();node=MissionConsumer();node.publish_map(make_map());node.settle()
        node.spin_until(lambda:node.control_state()['known_cells']>0)
        assert node.budget()['reason']=='battery_status_missing'
        assert not node.trigger_local(),'Required missing battery allowed a new task'
        node.set_battery(120);node.move_odometry(node.position[:3])
        local,metric=node.plan();assert metric['success']
        route,frontier=node.global_call(ignore_time=False);assert frontier['success'],frontier
        target=frontier['target'];expected=xyz(route.path)
        node.set_battery(22)
        route,insufficient=node.global_call(target=target,manual=True,ignore_time=False)
        assert not route.path and insufficient['reason']=='global_time_insufficient',insufficient
        route,ignored=node.global_call(target=target,manual=True,ignore_time=True)
        assert route.path and ignored['success']
        assert node.trigger_global(id=target,not_check_frontier=True,ignore_time=False)
        rejected_task=node.wait_control({'ERROR','DONE'})
        assert rejected_task['reason']=='explicit_global_path_empty' and not node.commands,rejected_task

        # High budget permits a real native global trajectory to the frontier.
        node.set_battery(120)
        assert node.trigger_global(id=target,not_check_frontier=True,ignore_time=False)
        outward=node.wait_control({'DONE','ERROR'})
        assert outward['state']=='DONE' and outward['completed_actions']==1,outward
        assert math.dist(node.position[:3],expected[-1])<.05
        home_route,home_metric=node.global_call(homing=True);assert home_metric['success']
        return_distance=home_metric['distance_m']
        assert return_distance>.9  # 21 s minus the original 20 s reserve is insufficient.
        node.set_battery(21)
        assert node.trigger_local()
        returned=node.wait_control({'DONE','ERROR'})
        assert returned['state']=='DONE' and returned['reason']=='returned_home',returned
        assert returned['completed_actions']==1,'Budget homing was truncated into local edges'
        assert len(node.commands)==2 and math.dist(node.position[:3],(-2.,0.,0.))<.05
        actual=node.commands[-1]['points'];cursor=0
        for vertex in xyz(home_route.path):
            while cursor<len(actual) and math.dist(vertex,actual[cursor][:3])>1e-8:cursor+=1
            assert cursor<len(actual),('Budget homing omitted a route vertex',vertex)

        # Once reporting has begun, an old positive number cannot remain valid.
        node.set_battery(120)
        assert node.trigger_global(id=target,not_check_frontier=True,ignore_time=False)
        active=node.wait_control({'EXECUTING','ERROR'});assert active['state']=='EXECUTING',active
        node.battery_enabled=False
        stale=node.wait_control({'ERROR'},seconds=8)
        assert stale['reason']=='battery_status_stale',stale
        node.spin_until(lambda:not node.state()['active'])
        assert node.state()['hold_commands']>=1 and len(node.commands[-1]['points'])==1
        assert not node.trigger_local()

        # Invalid/replayed estimates cannot restore liveness.
        old=node.budget()['rejected_battery']
        for value,stamp in [(float('nan'),node.get_clock().now().to_msg()),(-1.,node.get_clock().now().to_msg()),(120.,RobotStatus().header.stamp)]:
            message=RobotStatus();message.header.stamp=stamp;message.time_remaining=value;node.battery.publish(message)
        node.spin_until(lambda:node.budget()['rejected_battery']>=old+3)
        assert node.budget()['reason']=='battery_status_stale'
        node.set_battery(120)
        request=node.pci_home.srv_type.Request()
        assert node.call(node.pci_home,request).success
        recovered=node.wait_control({'DONE','ERROR'})
        assert recovered['state']=='DONE' and recovered['reason']=='returned_home',recovered
        assert math.dist(node.position[:3],(-2.,0.,0.))<.05

        # Exhaustion is terminal even for an unrestricted explicit global request.
        node.set_battery(0)
        assert node.budget()['reason']=='mission_budget_exhausted'
        assert not node.trigger_local() and not node.trigger_global(id=target,not_check_frontier=True)
        local,exhausted=node.plan()
        assert not local.path and local.status==local.K_HOMING and exhausted['reason']=='mission_budget_exhausted'
        # A map reset cannot renew the mission clock or cached battery estimate.
        before=node.budget()['elapsed_s']
        unknown=make_map();data=bytearray(unknown.data)
        for index in range(12,len(data),unknown.point_step):data[index]=255
        unknown.data=bytes(data);node.publish_map(unknown)
        assert node.budget()['elapsed_s']>=before
        assert node.budget()['remaining_s']==0
        assert not node.errors,node.errors
        output={'status':'passed','scope':'Mission budget + original RobotStatus + native global and homing execution',
                'target':target,'global_path':expected,'home_distance_before_return_m':return_distance,
                'low_battery_seconds':21,'reserve_seconds':20,'budget_homing':returned,
                'stale_battery_stop':stale,'recovered':recovered,'exhausted_metric':exhausted,
                'execution':node.state(),'mission':node.budget(),
                'checks':{'missing_required_battery_rejected':True,'time_constrained_global_rejected':True,
                          'ignore_time_route_override':True,'native_global_execution':True,
                          'low_battery_local_returns_full_homing_path':True,'stale_battery_cancels_and_holds':True,
                          'invalid_battery_does_not_refresh':True,'fresh_battery_allows_explicit_restart':True,
                          'exhaustion_prevents_new_motion':True,'map_reset_preserves_mission_budget':True}}
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for process in reversed(processes):
            if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
            assert process.returncode==0,('Unclean child exit',process.returncode)
    if output is not None:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        temp=args.output.with_suffix('.tmp');temp.write_text(json.dumps(output,indent=2)+'\n');temp.replace(args.output)
        print(json.dumps(output,indent=2))

if __name__=='__main__':main()

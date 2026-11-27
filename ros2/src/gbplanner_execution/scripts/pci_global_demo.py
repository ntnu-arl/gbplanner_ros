#!/usr/bin/env python3
"""Verify original PCI global request routing against separate ROS 2 processes.

The consumer is a test fixture. Planning, orchestration, and native trajectory
execution run in their own processes; the test never computes a planner path.
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
from std_srvs.srv import Trigger
from planner_msgs.srv import PciGlobal, PciTrigger, PciHomingTrigger
from execution_demo import Consumer, wrap
from local_demo import make_map, xyz
from mapping_demo import inspect_snapshot, verify_path


class GlobalConsumer(Consumer):
    def __init__(self):
        self.snapshot=inspect_snapshot(make_map())
        super().__init__()
        self.control=self.create_client(Trigger,'control_state')
        self.pci_global=self.create_client(PciGlobal,'pci_global')
        self.pci_home=self.create_client(PciHomingTrigger,'pci_homing_trigger')
        self.pci_local=self.create_client(PciTrigger,'pci_trigger')
        for client in (self.control,self.pci_global,self.pci_home,self.pci_local):
            assert client.wait_for_service(timeout_sec=15)

    def control_state(self):
        return json.loads(self.call(self.control,Trigger.Request()).message)

    def wait_control(self,states,seconds=45):
        result={}
        def finished():
            result.update(self.control_state())
            return result['state'] in states
        self.spin_until(finished,seconds=seconds)
        return result

    def trigger_global(self,**overrides):
        request=PciGlobal.Request()
        request.bound_mode=3
        request.ignore_time=True
        for key,value in overrides.items():setattr(request,key,value)
        return self.call(self.pci_global,request).success

    def command(self,message):
        before=len(self.commands)
        super().command(message)
        if len(self.commands)==before:return
        command=self.commands[-1]
        try:
            verify_path([point[:3] for point in command['points']],self.snapshot)
            for a,b,ta,tb in zip(command['points'],command['points'][1:],command['times'],command['times'][1:]):
                assert abs(wrap(b[3]-a[3]))/(tb-ta)<=.150001
        except AssertionError as error:self.errors.append(str(error))

    def tick(self):
        before=self.position
        super().tick()
        if self.position!=before:
            try:verify_path([before[:3],self.position[:3]],self.snapshot)
            except AssertionError as error:self.errors.append(str(error))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--start-node',action='store_true')
    parser.add_argument('--output',type=Path,default=Path('results/pci_global_integration.json'))
    args=parser.parse_args();processes=[];node=None;output=None
    try:
        if args.start_node:
            config=Path(get_package_share_directory('gbplanner_local'))/'config/local.yaml'
            commands=[
                ['ros2','run','gbplanner_local','local_planner_node','--ros-args','--params-file',str(config),'-p','global.enable:=true'],
                ['ros2','run','gbplanner_local','control_node.py'],
                ['ros2','run','gbplanner_execution','path_follower_node.py'],
            ]
            for command in commands:processes.append(subprocess.Popen(command,start_new_session=True))
        rclpy.init();node=GlobalConsumer();node.publish_map(make_map());node.settle()
        node.spin_until(lambda:node.control_state()['known_cells']>0)
        node.move_odometry(node.position[:3])
        local,metric=node.plan();assert metric['success'],metric
        route,frontier=node.global_call();assert frontier['success'],frontier
        target=frontier['target'];assert target>0
        expected=xyz(route.path);verify_path(expected,node.snapshot)

        # Unsupported requests are rejected before starting a task.
        invalid=[{'set_auto':True},{'ignore_time':False},{'vel_max':math.inf},
                 {'bound_mode':99},{'id':-1},{'not_check_frontier':True}]
        for options in invalid:assert not node.trigger_global(**options),options
        assert not node.commands

        # Automatic frontier selection is one global request, not an auto loop.
        assert node.trigger_global(not_exe_path=True)
        automatic=node.wait_control({'PREVIEW','ERROR'})
        assert automatic['state']=='PREVIEW' and not node.commands,automatic
        assert automatic['explicit_global']=={'id':0,'not_check_frontier':False,'ignore_time':True,'bound_mode':3}
        assert [event['mode'] for event in automatic['events'] if event['state']=='PLANNING']==['global']

        assert node.trigger_global(id=target,not_check_frontier=True,not_exe_path=True)
        preview=node.wait_control({'PREVIEW','ERROR'})
        assert preview['state']=='PREVIEW' and preview['explicit_global']['id']==target,preview
        assert not node.commands

        # A valid-shaped but absent ID must fail as that requested task.
        assert node.trigger_global(id=999999,not_check_frontier=True)
        missing=node.wait_control({'ERROR','DONE'})
        assert missing['state']=='ERROR' and missing['reason']=='explicit_global_path_empty',missing
        assert [event['mode'] for event in missing['events'] if event['state']=='PLANNING']==['global']
        assert not node.commands,'Missing target unexpectedly sent a homing command'

        # Native execution receives the complete multi-edge global route.
        assert node.trigger_global(id=target,not_check_frontier=True)
        active=node.wait_control({'EXECUTING','ERROR'})
        assert active['state']=='EXECUTING',active
        assert not node.trigger_global(not_exe_path=True),'Concurrent global task accepted'
        finished=node.wait_control({'DONE','ERROR'})
        assert finished['state']=='DONE' and finished['completed_actions']==1,finished
        assert math.dist(node.position[:3],expected[-1])<.05
        assert len(node.commands)==1 and node.state()['succeeded']==1
        actual=node.commands[0]['points']
        # Reconnection and yaw interpolation may add points, but each original
        # route vertex must appear in order, including its exact terminal point.
        cursor=0
        for vertex in expected:
            while cursor<len(actual) and math.dist(vertex,actual[cursor][:3])>1e-8:cursor+=1
            assert cursor<len(actual),('Global route vertex omitted',vertex)
        assert not node.errors,node.errors

        home=PciHomingTrigger.Request()
        assert node.call(node.pci_home,home).success
        returned=node.wait_control({'DONE','ERROR'})
        assert returned['state']=='DONE' and returned['reason']=='returned_home',returned
        assert math.dist(node.position[:3],(-2.,0.,0.))<.05
        assert len(node.commands)==2 and node.state()['succeeded']==2

        # No automatic frontier must also fail without rerouting.
        full=make_map(known=True);node.snapshot=inspect_snapshot(full);node.publish_map(full)
        assert node.trigger_global()
        exhausted=node.wait_control({'ERROR','DONE'})
        assert exhausted['reason']=='explicit_global_path_empty' and len(node.commands)==2,exhausted
        # A later ordinary local request must clear the explicit-global mode.
        local_request=PciTrigger.Request();local_request.bound_mode=3;local_request.set_auto=True
        assert node.call(node.pci_local,local_request).success
        fallback=node.wait_control({'STOPPED','ERROR'})
        assert fallback['reason']=='global_frontier_unavailable' and fallback['explicit_global'] is None,fallback
        requests=[event['reason'] for event in fallback['events'] if event['reason'].endswith('_requested')]
        assert requests==['local_requested']*4,requests
        assert len(node.commands)==2 and not node.errors,node.errors
        output={'status':'passed','scope':'PCI global service + independent planner/control/native trajectory processes + test consumer',
                'selected_target':target,'expected_global_path':expected,
                'global_trajectory_points':len(actual),'global_endpoint_error_m':math.dist(actual[-1][:3],expected[-1]),
                'home_error_m':math.dist(node.position[:3],(-2.,0.,0.)),
                'execution':node.state(),'manual_global':finished,'returned':returned,
                'missing_target':missing,'no_frontier':exhausted,'local_fallback':fallback,
                'checks':{'unsupported_parameters_rejected':len(invalid),
                          'automatic_frontier_preview':True,'explicit_target_forwarded':True,
                          'preview_sends_no_trajectory':True,'missing_target_never_homes':True,
                          'no_frontier_never_homes':True,'full_global_route_executed':True,
                          'native_trajectory_and_motion_collision_checks':True,
                          'native_trajectory_rate_checks':True,'concurrent_request_rejected':True,
                          'home_after_global':True,'later_local_uses_planner_frontier_policy':True}}
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
        temporary=args.output.with_suffix('.tmp')
        temporary.write_text(json.dumps(output,indent=2)+'\n');temporary.replace(args.output)
        print(json.dumps(output,indent=2))


if __name__=='__main__':main()

#!/usr/bin/env python3
"""Verify remaining-route validation by changing the map during native motion."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import sys
import rclpy
from ament_index_python.packages import get_package_share_directory
from planner_msgs.srv import PciHomingTrigger
from pci_global_demo import GlobalConsumer
from local_demo import make_map,xyz
from mapping_demo import inspect_snapshot


def obstacle(center):
    message=make_map();data=bytearray(message.data)
    changed=0
    for offset in range(0,len(data),message.point_step):
        position=struct.unpack_from('<fff',data,offset)
        if all(abs(a-b)<=.26 for a,b in zip(position,center)):
            data[offset+12]=100;changed+=1
    assert changed>0
    message.data=bytes(data);return message


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--start-node',action='store_true')
    parser.add_argument('--output',type=Path,default=Path('results/path_progress_integration.json'))
    parser.add_argument('--case',choices=['all','behind','ahead'],default='all')
    args=parser.parse_args();processes=[];node=None;output=None
    if args.case=='all':
        cases={}
        for case in ['behind','ahead']:
            destination=args.output.with_name(args.output.stem+'_'+case+'.json')
            command=[sys.executable,__file__,'--case',case,'--output',str(destination)]
            if args.start_node:command.append('--start-node')
            subprocess.run(command,check=True,timeout=90)
            cases[case]=json.loads(destination.read_text())
        result={'status':'passed','scope':'Fresh processes per obstacle case; no automatic restoration of pruned graph edges',
                'cases':cases,'checks':{'passed_obstacle_does_not_cancel':True,'future_obstacle_cancels_and_holds':True}}
        args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return
    try:
        if args.start_node:
            config=Path(get_package_share_directory('gbplanner_local'))/'config/local.yaml'
            for command in [
                ['ros2','run','gbplanner_local','local_planner_node','--ros-args','--params-file',str(config),'-p','global.enable:=true'],
                ['ros2','run','gbplanner_local','control_node.py'],
                ['ros2','run','gbplanner_execution','path_follower_node.py']]:
                processes.append(subprocess.Popen(command,start_new_session=True))
        rclpy.init();node=GlobalConsumer();node.publish_map(make_map());node.settle()
        node.spin_until(lambda:node.control_state()['known_cells']>0)
        node.move_odometry(node.position[:3]);node.plan()
        path,metric=node.global_call();assert metric['success'],metric
        target=metric['target'];route=xyz(path.path);home=tuple(node.position[:3])
        assert node.trigger_global(id=target,not_check_frontier=True)
        if args.case=='behind':
            node.spin_until(lambda:math.dist(node.position[:3],home)>1.5,seconds=10)
            changed=obstacle(home);node.snapshot=inspect_snapshot(changed);node.publish_map(changed)
            completed=node.wait_control({'DONE','ERROR'})
            assert completed['state']=='DONE',completed
            assert node.state()['succeeded']==1 and node.state()['hold_commands']==0
            assert math.dist(node.position[:3],route[-1])<.05
            check='passed_obstacle_does_not_cancel'
        else:
            assert node.wait_control({'EXECUTING','ERROR'})['state']=='EXECUTING'
            # Wait for receipt and real movement before changing the map; the
            # original command must be checked against its publication map.
            node.spin_until(lambda:len(node.commands)==1 and math.dist(node.position[:3],home)>.5,seconds=10)
            changed=obstacle(route[-1]);node.snapshot=inspect_snapshot(changed);node.publish_map(changed)
            completed=node.wait_control({'ERROR'})
            node.spin_until(lambda:not node.state()['active'])
            assert node.state()['hold_commands']>=1 and len(node.commands[-1]['points'])==1
            check='future_obstacle_cancels_and_holds'
        assert not node.errors,node.errors
        output={'status':'passed','case':args.case,'result':completed,'execution':node.state(),
                'route':route,'checks':{check:True,'independent_command_and_motion_collision_checks':True}}
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

#!/usr/bin/env python3
"""Dynamic lidar + controller + separate PCI trajectory action process.

This process provides only a test trajectory consumer, sensing and odometry.
It never accepts planning actions or calls planning services to advance motion.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import rclpy
from ament_index_python.packages import get_package_prefix
from trajectory_msgs.msg import MultiDOFJointTrajectory
from std_srvs.srv import Trigger
from planner_msgs.srv import PciTrigger,PlannerSrv
from execution_demo import decode,sample
sys.path.insert(0,str(Path(get_package_prefix('gbplanner_local'))/'lib/gbplanner_local'))
from system_demo import SensorExecutor
from mapping_demo import verify_path
from exploration_demo import save_slice


class SensorConsumer(SensorExecutor):
    def __init__(self):
        self.command=None;self.trajectories=[];self.last_stamp=0
        super().__init__(1.,provide_action=False)
        self.create_subscription(MultiDOFJointTrajectory,'command/trajectory',self.receive_command,10)

    def receive_command(self,message):
        try:
            points,times=decode(message)
            assert math.dist(points[0][:3],self.position)<.1
            verify_path([p[:3] for p in points],self.snapshot)
            for a,b,ta,tb in zip(points,points[1:],times,times[1:]):
                assert math.dist(a[:3],b[:3])/(tb-ta)<=1.000001
                assert abs(math.atan2(math.sin(b[3]-a[3]),math.cos(b[3]-a[3])))/(tb-ta)<=.150001
            self.command=(points,times,time.monotonic(),(*self.position,self.yaw))
            self.trajectories.append({'mode':self.mode,'points':points,'times':times})
        except (AssertionError,ValueError) as error:self.failures.append('command:'+str(error))

    def tick(self):
        if self.command:
            points,times,started,initial=self.command
            next_state=sample(points,times,time.monotonic()-started,initial)
            try:verify_path([self.position,next_state[:3]],self.snapshot)
            except AssertionError as error:self.failures.append('motion:'+str(error));self.timer.cancel();return
            self.distance+=math.dist(self.position,next_state[:3]);self.position=list(next_state[:3]);self.yaw=next_state[3]
        self.odometry_now()
        if self.pending_scan:
            scan=self.pending_scan
            if not scan['published']:
                self.tf.sendTransform(scan['transform']);self.clouds.publish(scan['cloud']);scan['published']=True
            elif time.monotonic()-scan['started']>5:
                self.failures.append('scan_timeout');self.timer.cancel()
            return
        self.start_scan()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--start-node',action='store_true')
    parser.add_argument('--backend',choices=['occupancy','voxblox'],default='occupancy')
    parser.add_argument('--output',type=Path,default=Path('results/execution_system'))
    args=parser.parse_args();processes=[];node=None
    try:
        if args.start_node:
            launch=['ros2','launch','gbplanner_local','control.launch.py'] if args.backend=='occupancy' else ['ros2','launch','gbplanner_voxblox','mapping.launch.py','global_planning:=true','background:=true','control:=true']
            processes.append(subprocess.Popen(launch,start_new_session=True))
            processes.append(subprocess.Popen(['ros2','launch','gbplanner_execution','execution.launch.py'],start_new_session=True))
        rclpy.init();node=SensorConsumer()
        executor=node.create_client(Trigger,'execution_state');assert executor.wait_for_service(timeout_sec=15)
        execution=lambda:json.loads(node.call(executor,Trigger.Request()).message)
        node.spin_until(lambda:node.clouds.get_subscription_count()>0 and node.snapshot_count>0)
        node.spin_until(lambda:node.revision()['known_cells']>0 and execution()['known_cells']>0)
        initial=node.revision()['known_cells'];args.output.mkdir(parents=True,exist_ok=True)
        save_slice(node.snapshot,args.output/'initial_map.csv')
        request=PciTrigger.Request();request.set_auto=True;request.bound_mode=PlannerSrv.Request.K_EXACT_BOUND
        assert node.call(node.start_client,request).success
        deadline=time.monotonic()+180
        while True:
            state=node.state()
            if state['state'] in {'DONE','ERROR','STOPPED'}:break
            assert not node.failures,node.failures
            assert time.monotonic()<deadline,('system_timeout',state,execution())
            rclpy.spin_once(node,timeout_sec=.02)
        assert state['state']=='DONE',state
        assert state['reason'] in {'returned_home','already_home'}
        assert math.dist(node.position,node.home)<.1
        assert not node.failures,node.failures
        stats=execution();assert stats['succeeded']>=2 and stats['aborted']==0 and stats['cancelled']==0,stats
        mapper=json.loads(node.call(node.mapper,Trigger.Request()).message)
        assert mapper['rejected_scans']==0,mapper
        assert any(t['mode']=='homing' for t in node.trajectories),node.trajectories
        node.timer.cancel();save_slice(node.snapshot,args.output/'final_map.csv')
        with (args.output/'history.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(node.history[0]));writer.writeheader();writer.writerows(node.history)
        output={'status':'passed','scope':'Native PCI trajectory action backend with timed test consumer; no physical dynamics',
            'mapping_backend':args.backend, 'initial_known_cells':initial,'final_known_cells':mapper['known_cells'],'total_cells':19200,
            'distance_m':node.distance,'home_error_m':math.dist(node.position,node.home),'control':state,
            'executor':stats,'mapper':mapper,'commands':node.trajectories,
            'checks':{'separate_action_server':True,'topic_consumer_only':True,
                'timed_linear_and_yaw_limits':True,'independent_command_and_motion_collision_checks':True,
                'dynamic_lidar_and_tf':True,'returned_home':True,'odometry_gates_action_success':True}}
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for process in reversed(processes):
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
            assert process.returncode==0,process.returncode
    (args.output/'summary.json').write_text(json.dumps(output,indent=2)+chr(10))
    print(json.dumps({k:v for k,v in output.items() if k not in ['commands','control']},indent=2))


if __name__=='__main__':main()

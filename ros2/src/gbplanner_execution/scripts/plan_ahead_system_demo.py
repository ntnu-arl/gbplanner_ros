#!/usr/bin/env python3
"""Live lidar/planning/action plan-ahead scenario with a timed topic consumer.

This is kinematic integration, not physical flight. All routes are chosen by
native nodes; the fixture supplies only lidar, TF and measured odometry.
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
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from planner_msgs.srv import PciTrigger, PlannerSrv
from execution_system_demo import SensorConsumer
from exploration_demo import save_slice


class PlanAheadConsumer(SensorConsumer):
    def __init__(self):
        self.started = time.monotonic()
        self.command_observations = []
        self.control_events = []
        self.execution_events = []
        self.measured_speed = 0.
        self.motion_samples = []
        self.last_motion_time = None
        super().__init__()
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'execution_status', self.receive_execution, durable)

    def receive_control(self, message):
        super().receive_control(message)
        self.control_events.append({'time_s': time.monotonic()-self.started,
                                    'position': list(self.position), 'status': json.loads(message.data)})

    def receive_execution(self, message):
        self.execution_events.append({'time_s': time.monotonic()-self.started,
                                      'position': list(self.position), 'status': json.loads(message.data)})

    def receive_command(self, message):
        now = time.monotonic()
        previous = self.command
        record = {'time_s': now-self.started, 'position': list(self.position),
                  'measured_speed_mps': self.measured_speed,
                  'previous_remaining_s': 0., 'previous_endpoint_distance_m': 0.,
                  'received_point_count': len(message.points)}
        if previous is not None:
            points, times, started, _ = previous
            record['previous_remaining_s'] = max(0., times[-1]-(now-started))
            record['previous_endpoint_distance_m'] = math.dist(self.position, points[-1][:3])
        before = len(self.trajectories)
        super().receive_command(message)
        record['accepted_by_consumer'] = len(self.trajectories)==before+1
        record['replaced_while_moving'] = (record['accepted_by_consumer'] and previous is not None and
            record['previous_remaining_s']>.1 and record['previous_endpoint_distance_m']>.1 and self.measured_speed>.05)
        self.command_observations.append(record)

    def tick(self):
        before = tuple(self.position)
        now = time.monotonic()
        super().tick()
        if self.last_motion_time is not None and now>self.last_motion_time:
            self.measured_speed = math.dist(before,self.position)/(now-self.last_motion_time)
        self.last_motion_time = now
        self.motion_samples.append({'time_s': now-self.started, 'x': self.position[0],
                                    'y': self.position[1], 'z': self.position[2],
                                    'speed_mps': self.measured_speed, 'mode': self.mode})


def stop_processes(processes):
    results = []
    for name, process, log in reversed(processes):
        if process.poll() is None:
            if name=='follower':
                process.send_signal(signal.SIGINT)  # launch forwards to child
            else:
                os.killpg(process.pid,signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        results.append({'name':name,'returncode':process.returncode})
        log.close()
    return results


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--start-node',action='store_true')
    parser.add_argument('--lead-time',type=float,default=3.)
    parser.add_argument('--max-cycles',type=int,default=5)
    parser.add_argument('--timeout',type=float,default=90.)
    parser.add_argument('--output',type=Path,default=Path('results/plan_ahead/system.json'))
    args=parser.parse_args()
    if not math.isfinite(args.lead_time) or args.lead_time<=0 or args.max_cycles<2:
        raise ValueError('Positive lead time and at least two cycles required')
    if not math.isfinite(args.timeout) or args.timeout<=0:
        raise ValueError('Positive timeout required')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    evidence=args.output.parent/(args.output.stem+'_data');evidence.mkdir(exist_ok=True)
    node=None;processes=[];error=None
    report={'status':'failed','scope':'Native ROS 2 automatic plan-ahead with live synthetic lidar/TF and a timed kinematic trajectory consumer',
            'parameters':{'lead_time_s':args.lead_time,'max_cycles':args.max_cycles,'timeout_s':args.timeout,
                          'speed_mps':1.,'yaw_rate_rps':.15,'mission_limit_s':180.,'mission_homing_speed_mps':1.}}
    try:
        if args.start_node:
            config=Path(get_package_share_directory('gbplanner_local'))/'config/local.yaml'
            commands=[
                ('planner',['ros2','run','gbplanner_local','local_planner_node','--ros-args','--params-file',str(config),
                            '-p','global.enable:=true','-p','global.background_enable:=true','-p','mission.enabled:=true',
                            '-p','mission.homing_speed:=1.0','-p','mission.time_limit:=180.0']),
                ('mapper',['ros2','run','gbplanner_local','occupancy_mapper_node','--ros-args',
                           '-p','frame_id:=map','-p','sensor_frame:=lidar','-p','map.resolution:=0.25',
                           '-p','map.origin:=[-5.0,-5.0,-1.5]','-p','map.dimensions:=[40,40,12]',
                           '-p','sensor.min_range:=0.1','-p','sensor.max_range:=6.0']),
                ('control',['ros2','run','gbplanner_local','control_node.py','--ros-args',
                            '-p',f'planner_trigger_lead_time:={args.lead_time}',
                            '-p',f'max_cycles:={args.max_cycles}','-p','require_mission_status:=true',
                            '-p','execution_timeout:=90.0']),
                ('follower',['ros2','launch','gbplanner_execution','execution.launch.py'])]
            for name,command in commands:
                log=(evidence/(name+'.log')).open('w')
                processes.append((name,subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True),log))
        rclpy.init();node=PlanAheadConsumer()
        client=node.create_client(Trigger,'execution_state')
        assert client.wait_for_service(timeout_sec=15),'execution_state unavailable'
        execution=lambda:json.loads(node.call(client,Trigger.Request()).message)
        node.spin_until(lambda:node.clouds.get_subscription_count()>0 and node.snapshot_count>0)
        node.spin_until(lambda:node.revision()['known_cells']>0 and execution()['known_cells']>0)
        report['initial_known_cells']=node.revision()['known_cells']
        save_slice(node.snapshot,evidence/'initial_map.csv')
        request=PciTrigger.Request();request.set_auto=True;request.bound_mode=PlannerSrv.Request.K_EXACT_BOUND
        assert node.call(node.start_client,request).success,'Automatic exploration trigger rejected'
        started=time.monotonic();deadline=started+args.timeout
        while True:
            state=node.state();report['control']=state
            if state['state'] in {'DONE','ERROR','STOPPED'}:break
            assert not node.failures,node.failures
            assert time.monotonic()<deadline,('mission_timeout',state)
            rclpy.spin_once(node,timeout_sec=.02)
        report['mission_elapsed_s']=time.monotonic()-started
        stats=execution();mapper=json.loads(node.call(node.mapper,Trigger.Request()).message)
        report.update(executor=stats,mapper=mapper,scan_count=node.scan_count,final_known_cells=mapper['known_cells'],
                      total_cells=19200,distance_m=node.distance,home_error_m=math.dist(node.position,node.home))
        overlaps=[c for c in node.command_observations if c['replaced_while_moving']]
        report['replacements_while_moving']=len(overlaps)
        assert state['state']=='DONE',state
        assert state['reason'] in {'returned_home','already_home'},state
        assert state['plan_ahead']['continuations']>=1,state
        assert stats['continuations']==state['plan_ahead']['continuations'],(stats,state)
        assert overlaps,'No replacement received during measured motion before the old endpoint'
        assert stats['hold_commands']==0 and stats['aborted']==0 and stats['cancelled']==0,stats
        assert stats['trajectory_commands']==stats['accepted']+stats['continuations'],stats
        assert stats['succeeded']==stats['accepted'] and stats['succeeded']>=2,stats
        assert math.dist(node.position,node.home)<.1,node.position
        assert mapper['rejected_scans']==0 and mapper['integrated_scans']>1,mapper
        assert mapper['known_cells']>report['initial_known_cells'],mapper
        assert not node.failures,node.failures
        report['checks']={'real_planner_controls_all_routes':True,'dynamic_lidar_and_timestamped_tf':True,
            'new_map_observations_during_motion':True,'replacement_before_old_endpoint':True,
            'measured_motion_at_replacement':True,'no_hold_on_normal_continuation':True,
            'same_action_survives_replacement':True,'actual_home_arrival':True,
            'independent_command_and_motion_collision_checks':True,
            'mission_budget_enabled_with_matching_translation_speed':True}
        report['status']='passed'
    except Exception as caught:
        error=caught;report['error']=repr(caught)
    finally:
        if node is not None:
            node.timer.cancel()
            report.update(failures=node.failures,commands=node.trajectories,
                          command_observations=node.command_observations,
                          control_events=node.control_events,execution_events=node.execution_events)
            if node.snapshot:save_slice(node.snapshot,evidence/'final_map.csv')
            for filename,rows in [('history.csv',node.history),('motion.csv',node.motion_samples)]:
                if rows:
                    with (evidence/filename).open('w',newline='') as stream:
                        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        report['processes']=stop_processes(processes)
        if report['status']=='passed' and any(p['returncode']!=0 for p in report['processes']):
            report['status']='failed';error=RuntimeError('A launched ROS process exited unsuccessfully');report['error']=repr(error)
        temporary=args.output.with_suffix('.tmp');temporary.write_text(json.dumps(report,indent=2)+'\n');temporary.replace(args.output)
    print(json.dumps({k:v for k,v in report.items() if k not in
                     {'commands','command_observations','control','control_events','execution_events'}},indent=2))
    if error is not None:raise error


if __name__=='__main__':main()

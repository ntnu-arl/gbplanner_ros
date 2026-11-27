#!/usr/bin/env python3
"""Run native planning with original Lee control and Bullet physics.

The runner only requests PCI missions and records outputs. It never publishes
odometry, pointclouds, trajectories, forces, or robot poses.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from trajectory_msgs.msg import MultiDOFJointTrajectory
from std_srvs.srv import Trigger
from planner_msgs.srv import PciTrigger,PciHomingTrigger,PlannerSrv
from gbplanner_voxblox.srv import FilePath
from ament_index_python.packages import get_package_prefix
sys.path.insert(0,str(Path(get_package_prefix('gbplanner_local'))/'lib/gbplanner_local'))
from mapping_demo import inspect_snapshot
from exploration_demo import save_slice


class Mission(Node):
    def __init__(self):
        super().__init__('physical_mission_runner')
        self.control={};self.physics={};self.snapshot={};self.position=None;self.events=[]
        self.started=time.monotonic();self.odom_received=0.;self.measured_speed=0.
        self.execution_status={};self.execution_events=[];self.command_observations=[];self.previous_command=None
        durable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String,'control_status',self.receive_control,durable)
        self.create_subscription(String,'dynamics_status',lambda m:setattr(self,'physics',json.loads(m.data)),10)
        self.create_subscription(String,'execution_status',self.receive_execution,durable)
        self.create_subscription(PointCloud2,'voxel_map',lambda m:setattr(self,'snapshot',inspect_snapshot(m)),durable)
        self.create_subscription(Odometry,'odometry',self.receive_odometry,10)
        self.create_subscription(MultiDOFJointTrajectory,'command/trajectory',self.receive_command,10)
        self.trigger=self.create_client(PciTrigger,'pci_trigger');self.home=self.create_client(PciHomingTrigger,'pci_homing_trigger')
        self.state_client=self.create_client(Trigger,'control_state')
        self.mapper=self.create_client(Trigger,'mapper_status');self.execution=self.create_client(Trigger,'execution_state')
        self.sensor_gain=self.create_client(Trigger,'sensor_gain')
        self.save=self.create_client(FilePath,'save_map');self.mesh=self.create_client(FilePath,'export_mesh')
        for client in [self.state_client,self.trigger,self.home,self.mapper,self.execution,self.save,self.mesh]:
            if not client.wait_for_service(timeout_sec=20):raise RuntimeError('missing service '+client.srv_name)

    def receive_odometry(self,message):
        p=message.pose.pose.position;v=message.twist.twist.linear
        self.position=[p.x,p.y,p.z]
        # Linear-velocity norm is invariant under the body/world rotation.
        self.measured_speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
        self.odom_received=time.monotonic()

    def receive_control(self,message):
        self.control=json.loads(message.data)
        pair=(self.control['state'],self.control['reason'])
        if not self.events or pair!=(self.events[-1]['state'],self.events[-1]['reason']):
            self.events.append({'time_s':time.monotonic()-self.started,'state':pair[0],
                                'reason':pair[1],'mode':self.control['mode'],
                                'position':list(self.position) if self.position is not None else None})
            print('mission',pair,'at',self.position,flush=True)

    def receive_execution(self,message):
        self.execution_status=json.loads(message.data)
        self.execution_events.append({'time_s':time.monotonic()-self.started,
                                      'position':list(self.position) if self.position is not None else None,
                                      'status':self.execution_status})

    def receive_command(self,message):
        now=time.monotonic()
        endpoint=message.points[-1].transforms[0].translation
        duration=message.points[-1].time_from_start.sec+message.points[-1].time_from_start.nanosec*1e-9
        record={'command_number':len(self.command_observations)+1,'time_s':now-self.started,
                'header_stamp_s':message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                'point_count':len(message.points),'duration_s':duration,
                'endpoint':[endpoint.x,endpoint.y,endpoint.z],
                'position':list(self.position) if self.position is not None else None,
                'measured_speed_mps':self.measured_speed,'odometry_age_s':now-self.odom_received,
                'previous_endpoint':None,'previous_remaining_s':0.,'previous_endpoint_distance_m':0.}
        if self.previous_command and self.position is not None:
            previous=self.previous_command
            record['previous_endpoint']=previous['endpoint']
            record['previous_remaining_s']=max(0.,previous['duration_s']-(now-self.started-previous['time_s']))
            record['previous_endpoint_distance_m']=math.dist(self.position,previous['endpoint'])
        record['replaced_while_moving']=(record['point_count']>1 and record['odometry_age_s']<.2 and
            record['measured_speed_mps']>.05 and record['previous_remaining_s']>.1 and
            record['previous_endpoint_distance_m']>.1)
        self.command_observations.append(record);self.previous_command=record

    def moving_continuations(self):
        """Join independent command/status topics by the executor command count."""
        result=[]
        for event in self.execution_events:
            status=event['status']
            if status.get('reason')!='continuation_committed':continue
            index=status['trajectory_commands']+status['hold_commands']-1
            if not 0<=index<len(self.command_observations):continue
            command=self.command_observations[index]
            if command['replaced_while_moving']:
                result.append({'goal_id':status['goal_id'],'generation':status['generation'],
                               'command':command,'status_time_s':event['time_s']})
        return result

    def call(self,client,request,timeout=20):
        future=client.call_async(request);rclpy.spin_until_future_complete(self,future,timeout_sec=timeout)
        if not future.done():raise TimeoutError(client.srv_name)
        return future.result()

    def wait(self,predicate,seconds):
        deadline=time.monotonic()+seconds
        while not predicate():
            if time.monotonic()>deadline:raise TimeoutError((self.control,self.physics))
            rclpy.spin_once(self,timeout_sec=.02)
            if self.physics.get('contact_steps',0)>0:raise RuntimeError(('physical collision',self.physics))

    def execute(self,auto):
        request=PciTrigger.Request();request.bound_mode=PlannerSrv.Request.K_EXACT_BOUND;request.set_auto=auto
        result=self.call(self.trigger,request);assert result.success,('trigger rejected',self.control)
        self.wait(lambda:self.control.get('state') not in ['IDLE','DONE','STOPPED','ERROR'],10)

    def finish(self,seconds):
        self.wait(lambda:self.control.get('state') in ['DONE','ERROR','STOPPED'],seconds)
        assert self.control['state']=='DONE',self.control


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=Path('results/dynamics'))
    parser.add_argument('--local-cycles',type=int,default=0,help='0 enables automatic exploration until planner returns home')
    parser.add_argument('--speed',type=float,default=.25)
    parser.add_argument('--lead-time',type=float,default=0.)
    parser.add_argument('--mission',action='store_true')
    parser.add_argument('--lidar-layout',choices=['legacy','dual'],default='legacy')
    parser.add_argument('--timeout',type=float,default=480.)
    args=parser.parse_args()
    if not math.isfinite(args.speed) or args.speed<=0 or not math.isfinite(args.timeout) or args.timeout<=0:
        raise ValueError('Positive speed and timeout required')
    if not math.isfinite(args.lead_time) or args.lead_time<0 or args.local_cycles<0:
        raise ValueError('Nonnegative lead time and local cycles required')
    if args.lead_time>0 and args.local_cycles:
        raise ValueError('Plan-ahead requires automatic exploration (--local-cycles 0)')
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    process=None;node=None;log=None;caught=None
    report={'status':'failed','requested_speed_mps':args.speed,'lead_time_s':args.lead_time,
            'lidar_layout':args.lidar_layout,
            'mission_budget_enabled':args.mission,'automatic_exploration':args.local_cycles==0,
            'timeout_s':args.timeout,'launch_log':'nodes.log'}
    try:
        log=(out/'nodes.log').open('w')
        process=subprocess.Popen(['ros2','launch','gbplanner_dynamics','exploration.launch.py',
          'telemetry_file:='+str(out/'flight.csv'),'speed:='+str(args.speed),
          'lidar_layout:='+args.lidar_layout,
          'planner_trigger_lead_time:='+str(args.lead_time),'mission:='+str(args.mission).lower()],
          stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init();node=Mission();node.wait(lambda:node.snapshot and node.physics and node.position is not None,25)
        node.wait(lambda:json.loads(node.call(node.state_client,Trigger.Request()).message)['known_cells']>0,10)
        if args.lidar_layout=='dual':
            # Both halves must have arrived before the robot starts moving.
            def both_sources_ready():
                sources=json.loads(node.call(node.mapper,Trigger.Request()).message).get('input_sources',{})
                return all(sources.get(name,{}).get('integrated_scans',0)>0 for name in ('front','rear'))
            node.wait(both_sources_ready,15)
            assert node.sensor_gain.wait_for_service(timeout_sec=5)
            gain_response=node.call(node.sensor_gain,Trigger.Request())
            assert gain_response.success,gain_response.message
            report['initial_sensor_gain']=json.loads(gain_response.message)
        report['initial_mapper']=json.loads(node.call(node.mapper,Trigger.Request()).message)
        initial=len(node.snapshot);home=list(node.position);save_slice(node.snapshot,out/'initial_map.csv')
        report.update(initial_known_cells=initial,home_position=home)
        started=time.monotonic()
        if args.local_cycles:
            for _ in range(args.local_cycles):node.execute(False);node.finish(min(80,args.timeout))
            request=PciHomingTrigger.Request();assert node.call(node.home,request).success
            node.wait(lambda:node.control.get('state') not in ['DONE','IDLE'],10);node.finish(min(100,args.timeout))
        else:node.execute(True);node.finish(args.timeout)
        report['mission_elapsed_s']=time.monotonic()-started
        assert node.control['reason'] in {'returned_home','already_home'},node.control
        node.wait(lambda:node.physics['speed_mps']<.05,10)
        mapper=json.loads(node.call(node.mapper,Trigger.Request()).message)
        execution=json.loads(node.call(node.execution,Trigger.Request()).message)
        if args.lidar_layout=='dual':
            sources=mapper['input_sources']
            assert set(sources)=={'front','rear'},sources
            assert all(source['integrated_scans']>10 for source in sources.values()),sources
            assert sum(source['integrated_scans'] for source in sources.values())==mapper['integrated_scans'],sources
            response=node.call(node.sensor_gain,Trigger.Request());assert response.success,response.message
            report['final_sensor_gain']=json.loads(response.message)
            models=report['final_sensor_gain']['sensors']
            assert [model['name'] for model in models]==['front','rear'],models
        home_error=math.dist(node.position,home);assert home_error<.1,(home_error,node.position,home)
        assert execution['succeeded']>=2 and execution['aborted']==0 and execution['cancelled']==0,execution
        assert node.physics['steps']>1000 and node.physics['distance_m']>1 and node.physics['maximum_tilt_rad']>.001,node.physics
        assert node.physics['contact_steps']==0 and node.physics['rejected_commands']==0
        if args.lead_time>0:
            assert execution['continuations']>=1,execution
            assert node.moving_continuations(),'No continuation during measured physical motion before old endpoint'
            assert execution['hold_commands']==0,execution
            assert execution['trajectory_commands']==execution['accepted']+execution['continuations'],execution
        save_slice(node.snapshot,out/'final_map.csv')
        for client,name in [(node.save,'room.vxblx'),(node.mesh,'room.ply')]:
            request=FilePath.Request();request.file_path=str(out/name);response=node.call(client,request);assert response.success,response.message
        report.update(status='passed',
            scope='Original RotorS Lee control + native Bullet rigid body + motor lag + ray-cast lidar + Voxblox + planning + PCI execution',
            final_known_cells=mapper['known_cells'],home_error_m=home_error,mapper=mapper,execution=execution,
            checks={'force_and_torque_integration':True,'original_lee_controller_cpp':True,
                    'rotor_speed_lag_and_limits':True,'physical_collision_detection':True,
                    'body_attitude_transforms_lidar':True,'planner_uses_live_scans':True,
                    'no_pose_assignment_during_flight':True,'returned_home':True})
        if args.lead_time>0:
            report['checks'].update(continuation_during_measured_motion=True,same_action_continuation=True,
                                    no_hold_during_continuation=True)
        if args.lidar_layout=='dual':
            report['checks'].update(two_physical_sensor_origins=True,
                both_sources_fused_during_flight=True,independent_front_rear_gain_models=True)
    except Exception as error:
        caught=error;report.update(status='failed',error=repr(error))
        if node and node.execution.service_is_ready():
            try:report['execution']=json.loads(node.call(node.execution,Trigger.Request(),timeout=2).message)
            except Exception as status_error:report['execution_query_error']=repr(status_error)
    finally:
        if node:
            report.update(control=node.control,physics=node.physics,events=node.events,
                          latest_execution_status=node.execution_status,execution_events=node.execution_events,
                          command_observations=node.command_observations,
                          moving_continuations=node.moving_continuations(),
                          replacements_while_moving=len(node.moving_continuations()))
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if process:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
            report['launch_returncode']=process.returncode
            if process.returncode!=0 and caught is None:
                caught=RuntimeError('ROS launch exited unsuccessfully')
                report.update(status='failed',error=repr(caught))
        if log:log.close()
        (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in {'execution_events','command_observations'}},indent=2),flush=True)
    if caught is not None:raise caught


if __name__=='__main__':main()

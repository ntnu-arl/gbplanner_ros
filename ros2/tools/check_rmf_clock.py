#!/usr/bin/env python3
"""Directed ROS protocol check for the observed native RMF pause/thrust bug.

Synthetic clock/odometry only; no physics or autonomous mission claim.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve(); output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(100+os.getpid()%80))
    import rclpy
    from actuator_msgs.msg import Actuators
    from geometry_msgs.msg import Transform
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
    rclpy.init();node=Node('rmf_clock_probe')
    clock=node.create_publisher(Clock,'/clock',10)
    odom=node.create_publisher(Odometry,'odometry',10)
    trajectory=node.create_publisher(MultiDOFJointTrajectory,'command/trajectory',10)
    states=[];motors=[]
    node.create_subscription(String,'rmf_controller_state',lambda m:states.append(json.loads(m.data)),10)
    node.create_subscription(Actuators,'rmf/command/motor_speed',lambda m:motors.append(list(m.velocity)),10)
    result={'status':'failed','scope':'Synthetic ROS clock/odometry against native RMF controller transport',
            'physics':False,'checks':{}}
    log=output.with_suffix('.log').open('w')
    process=subprocess.Popen(['ros2','run','gbplanner_gazebo','rmf_controller_node','--ros-args',
                              '-p','use_sim_time:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    started=time.monotonic()
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
    def require(value,message):
        if not value:raise AssertionError(message)
    def set_clock(seconds):
        msg=Clock();stamp=round(seconds*1e9);msg.clock.sec,msg.clock.nanosec=divmod(stamp,1_000_000_000)
        clock.publish(msg);spin(.06)
    def observation(seconds):
        msg=Odometry();msg.header.frame_id='map';msg.child_frame_id='base_link'
        stamp=round(seconds*1e9);msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(stamp,1_000_000_000)
        msg.pose.pose.position.z=1.;msg.pose.pose.orientation.w=1.;odom.publish(msg);spin(.06)
    try:
        end=time.monotonic()+5.
        while not (clock.get_subscription_count() and odom.get_subscription_count() and trajectory.get_subscription_count() and states):
            spin(.03);require(time.monotonic()<end,'Controller discovery failed')
        set_clock(10.);observation(10.)
        command=MultiDOFJointTrajectory();command.header.frame_id='map';command.header.stamp.sec=10;command.joint_names=['base_link']
        for sec,z in [(0,1.),(1,1.2)]:
            point=MultiDOFJointTrajectoryPoint();transform=Transform();transform.translation.z=z;transform.rotation.w=1.
            point.transforms=[transform];point.time_from_start.sec=sec;command.points.append(point)
        trajectory.publish(command);spin(.06)
        set_clock(10.01);observation(10.01);spin(.12)
        require(states[-1]['accepted_commands']==1 and min(motors[-1])>0,'Controller did not produce hover input')
        baseline=list(motors[-1]);start_index=len(motors)
        spin(.75)
        require(states[-1]['state']=='SIMULATION_WAIT' and states[-1]['sample_index']==0 and
                all(v==baseline for v in motors[start_index:]),'Frozen simulation cut thrust or advanced samples')
        result['checks']['frozen_simulation_retains_motor_input']=True
        result['paused_state']=states[-1]
        set_clock(10.7);spin(.16)
        require(states[-1]['state']=='WAITING_ODOMETRY' and motors[-1]==[0.,0.,0.,0.],
                'Advancing clock without odometry bypassed original source timeout')
        result['checks']['advancing_simulation_stale_source_stops_motors']=True
        observation(10.7);spin(.12)
        require(min(motors[-1])>0 and states[-1]['accepted_commands']==1 and states[-1]['sample_index']==0,
                'Fresh simulated observation did not resume the existing command')
        set_clock(11.1);observation(11.1);spin(.12)
        require(states[-1]['sample_index']==1,'Fresh acquisition time did not advance timed sample')
        result['checks']['fresh_source_resumes_and_advances_original_trajectory']=True
        set_clock(10.);spin(.16)
        require(states[-1]['state']=='CLOCK_REVERSED' and states[-1]['samples']==0 and motors[-1]==[0.,0.,0.,0.],
                'Clock reversal failed to clear old trajectory and thrust')
        result['checks']['clock_reversal_clears_trajectory']=True
        result['status']='passed'
    except Exception as error:
        result['error']=f'{type(error).__name__}: {error}'
    finally:
        # ros2 run is a CLI parent, not the controller process itself.
        os.killpg(process.pid, signal.SIGINT)
        try:result['exit_code']=process.wait(timeout=3.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL);result['exit_code']=process.wait();result['status']='failed'
        if result['exit_code']!=0:result['status']='failed'
        result['last_state']=states[-1] if states else None
        node.destroy_node();rclpy.shutdown();log.close()
        result['wall_elapsed_s']=time.monotonic()-started
        root=Path(__file__).resolve().parents[1]
        result['controller_source_sha256']=hashlib.sha256((root/'src/gbplanner_gazebo/src/rmf_controller_node.cpp').read_bytes()).hexdigest()
        output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return 0 if result['status']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())

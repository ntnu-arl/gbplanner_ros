#!/usr/bin/env python3
"""Native rotated sampling configuration and planning against a fixed voxel map."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from planner_msgs.msg import Graph
from planner_msgs.srv import ValidatePath
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from local_demo import Demo,make_map,xyz
from mapping_demo import inspect_snapshot

ROTATIONS=(.7,-.2,.1)
LOW=(-2.5,-.6,-.3)
HIGH=(2.5,.6,.3)
ROOT=(-1.,.3,0.)


def sampling_coordinates(v):
    """Independent scalar Rz(yaw) Ry(pitch) Rx(roll) world-to-local mapping."""
    x,y,z=v;yaw,pitch,roll=ROTATIONS
    c,s=math.cos(roll),math.sin(roll);y,z=c*y-s*z,s*y+c*z
    c,s=math.cos(pitch),math.sin(pitch);x,z=c*x+s*z,-s*x+c*z
    c,s=math.cos(yaw),math.sin(yaw);return (c*x-s*y,s*x+c*y,z)



def verify_path(path,snapshot):
    # Probe the supplied occupancy snapshot, without assuming obstacles from a
    # different fixture. Both unknown and occupied cells forbid the robot box.
    for a,b in zip(path,path[1:]):
        steps=max(1,math.ceil(math.dist(a,b)/.025))
        for step in range(steps+1):
            center=[a[i]+(b[i]-a[i])*step/steps for i in range(3)]
            for dx in [-.2,-.1,0.,.1,.2]:
                for dy in [-.2,-.1,0.,.1,.2]:
                    for dz in [-.2,-.1,0.,.1,.2]:
                        p=[center[0]+dx,center[1]+dy,center[2]+dz]
                        key=tuple(math.floor((p[i]-(-5,-5,-1.5)[i])/.25) for i in range(3))
                        assert snapshot.get(key,-1)==0,('path left supplied free map',p)

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,default=Path('results/rotated_sampling/summary.json'))
    args=parser.parse_args();node=None;process=None;report={'status':'failed'}
    try:
        process=subprocess.Popen(['ros2','run','gbplanner_local','local_planner_node','--ros-args',
            '-p','sampling.rotations:=[0.7, -0.2, 0.1]',
            '-p','sampling.min:=[-2.5, -0.6, -0.3]',
            '-p','sampling.max:=[2.5, 0.6, 0.3]'],start_new_session=True)
        rclpy.init();node=Demo();graphs=[]
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Graph,'local_graph',graphs.append,qos)
        validator=node.create_client(ValidatePath,'validate_path');assert validator.wait_for_service(timeout_sec=10)
        point=Pose();point.position.x,point.position.y,point.position.z=ROOT;point.orientation.w=1.
        def odometry():
            message=Odometry();message.header.frame_id='map';message.header.stamp=node.get_clock().now().to_msg()
            message.pose.pose=point;node.odometry.publish(message)
        timer=node.create_timer(.02,odometry)
        snapshot=make_map(known=False,wall=False);node.publish_map(snapshot)
        node.spin_until(lambda:node.odometry.get_subscription_count()>0)
        deadline=time.monotonic()+.1
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
        response,metrics=node.plan(root=point)
        assert metrics['success'],metrics
        node.spin_until(lambda:graphs and len(graphs[-1].vertices)==metrics['vertices'])
        graph=graphs[-1];vertices=[];outside_unrotated=0
        for vertex in graph.vertices:
            p=vertex.pose.position;world=(p.x,p.y,p.z)
            offset=tuple(a-b for a,b in zip(world,ROOT));local=sampling_coordinates(offset)
            assert all(a-1e-10<=b<=c+1e-10 for a,b,c in zip(LOW,local,HIGH)),(world,local)
            if any(not a-1e-10<=b<=c+1e-10 for a,b,c in zip(LOW,offset,HIGH)):outside_unrotated+=1
            vertices.append({'id':vertex.id,'world':world,'sampling':local})
        assert outside_unrotated>0,'Rotated long axis must extend beyond the old world-axis box'
        points=xyz(response.path);verify_path(points,inspect_snapshot(snapshot))
        request=ValidatePath.Request();request.header.frame_id='map';request.path=response.path
        assert node.call(validator,request).valid
        report={'status':'passed','scope':'ROS 2 parameter -> rotated local RRG -> independent frame and collision checks; static map, no motion',
                'rotations_ypr_rad':ROTATIONS,'root':ROOT,'min':LOW,'max':HIGH,
                'metrics':metrics,'vertices_outside_old_world_axis_box':outside_unrotated,
                'vertices':vertices,'path':points}
        timer.cancel()
    except Exception as error:
        report['error']=repr(error);raise
    finally:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if process:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
    print(json.dumps({k:v for k,v in report.items() if k not in {'vertices','path'}},indent=2))


if __name__=='__main__':main()

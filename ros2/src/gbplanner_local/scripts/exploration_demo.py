#!/usr/bin/env python3
"""ROS 2 closed loop with ideal ray returns and kinematic first-edge motion."""
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
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from local_demo import Demo, xyz
from mapping_demo import lidar_cloud, inspect_snapshot, verify_path
from global_demo import GlobalDemo

def save_slice(snapshot,path):
    with path.open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['x','y','status'])
        for y in range(40):
            for x in range(40):
                writer.writerow([-5+(x+.5)*.25,-5+(y+.5)*.25,snapshot.get((x,y,6),-1)])

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--start-node',action='store_true')
    parser.add_argument('--steps',type=int,default=20)
    parser.add_argument('--global-planning',action='store_true')
    parser.add_argument('--output',type=Path,default=Path('results/ros_exploration'))
    args=parser.parse_args()
    if not 1<=args.steps<=100:raise ValueError('steps must be 1..100')
    process=None;node=None
    try:
        if args.start_node:
            launch='global.launch.py' if args.global_planning else 'mapping.launch.py'
            process=subprocess.Popen(['ros2','launch','gbplanner_local',launch],start_new_session=True)
        rclpy.init();node=GlobalDemo() if args.global_planning else Demo();mapper=node.create_client(Trigger,'mapper_status')
        assert mapper.wait_for_service(timeout_sec=15)
        clouds=node.create_publisher(PointCloud2,'points',qos_profile_sensor_data)
        broadcaster=TransformBroadcaster(node)
        snapshots=[]
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(PointCloud2,'voxel_map',snapshots.append,qos)
        node.spin_until(lambda:clouds.get_subscription_count()>0 and node.odometry.get_subscription_count()>0)
        read=lambda:json.loads(node.call(mapper,Trigger.Request()).message)
        assert read()['integrated_scans']==0,'Use a fresh mapper for a reproducible experiment'
        args.output.mkdir(parents=True,exist_ok=True)
        position=[-2.0,0.0,0.0];yaw=math.pi/2
        trajectory=[position[:]];history=[];distance=0;last_known=0;stop='step_budget'
        home=position[:];global_target=None;relocations=0;global_requests=[]

        def observe():
            # Same explicit stamp for TF and point cloud. Re-send only while waiting
            # for discovery/TF delivery; no future or latest transform fallback.
            stamp=node.get_clock().now().to_msg()
            transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='lidar'
            transform.header.stamp=stamp
            transform.transform.translation.x=float(position[0]);transform.transform.translation.y=float(position[1]);transform.transform.translation.z=float(position[2])
            transform.transform.rotation.z=math.sin(yaw/2);transform.transform.rotation.w=math.cos(yaw/2)
            cloud=lidar_cloud(stamp,origin=position,yaw=yaw)
            before=read()['integrated_scans'];snapshot_count=len(snapshots)
            deadline=time.monotonic()+15
            while read()['integrated_scans']==before:
                assert time.monotonic()<deadline,'Timed out integrating moving lidar scan'
                broadcaster.sendTransform(transform);clouds.publish(cloud)
                rclpy.spin_once(node,timeout_sec=.05)
            node.spin_until(lambda:len(snapshots)>snapshot_count)
            status=read()
            node.spin_until(lambda:node.revision()['known_cells']==status['known_cells'])
            return inspect_snapshot(snapshots[-1]),status

        def plan_at_current_pose():
            if args.global_planning:node.move_odometry(position,yaw)
            odom=Odometry();odom.header.frame_id='map';odom.header.stamp=node.get_clock().now().to_msg()
            odom.pose.pose.position.x=float(position[0]);odom.pose.pose.position.y=float(position[1]);odom.pose.pose.position.z=float(position[2])
            odom.pose.pose.orientation.z=math.sin(yaw/2);odom.pose.pose.orientation.w=math.cos(yaw/2)
            deadline=time.monotonic()+5
            while True:
                node.odometry.publish(odom);response,metrics=node.plan()
                if metrics['reason']=='odometry_missing_or_invalid':
                    assert time.monotonic()<deadline;continue
                coordinates=xyz(response.path)
                if not coordinates or math.dist(coordinates[0],position)<1e-8:return response,metrics
                assert time.monotonic()<deadline,'Planner used stale odometry'

        snapshot,status=observe();initial_known=status['known_cells'];last_known=initial_known
        save_slice(snapshot,args.output/'initial_map.csv')
        for iteration in range(args.steps):
            mode='local'
            if global_target is not None:
                # Finish relocation before requesting another local plan. Query
                # again after every scan so changed occupancy invalidates edges.
                node.move_odometry(position,yaw)
                response,metrics=node.global_call(target=global_target,manual=True)
                global_requests.append(metrics);mode='global'
                if not metrics['success']:
                    stop='relocation_'+metrics['reason'];break
                if metrics['distance_m']<.05:global_target=None
            if global_target is None:
                response,metrics=plan_at_current_pose();mode='local'
                if not metrics['success']:
                    if not args.global_planning or metrics['reason']!='no_gain':
                        stop=metrics['reason'];break
                    response,metrics=node.global_call();global_requests.append(metrics);mode='global'
                    if not metrics['success']:
                        stop=metrics['reason'];break
                    global_target=metrics['target'];relocations+=1
            path=xyz(response.path);assert len(path)>=2
            verify_path(path,snapshot)
            # Execute a single checked edge. No command is sent to any real robot.
            next_position=path[1];step=math.dist(position,next_position)
            assert step>0
            q=response.path[1].orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            position=next_position;distance+=step;trajectory.append(position[:])
            snapshot,status=observe()
            assert status['known_cells']>=last_known
            history.append({'iteration':iteration,'x':position[0],'y':position[1],'z':position[2],
                'known_before':last_known,'known_after':status['known_cells'],
                'known_fraction':status['known_cells']/19200,'executed_length_m':step,
                'planned_length_m':metrics.get('path_length_m',metrics.get('distance_m')),
                'gain':metrics.get('gain'),'planning_ms':metrics.get('build_ms',0)+metrics.get('evaluate_ms',0),
                'mode':mode,'global_frontiers':node.status()['frontiers'] if args.global_planning else 0,
                'integrated_scans':status['integrated_scans']})
            last_known=status['known_cells']
            print(f'iteration {iteration+1}: known={last_known}/19200, traveled={distance:.3f} m',flush=True)
        assert history,'Closed loop did not execute a path'
        save_slice(snapshot,args.output/'final_map.csv')
        return_trajectory=[position[:]];return_distance=0;return_metrics=[]
        if args.global_planning:
            # This is kinematic execution in a synthetic scene, not PathFollower.
            for _ in range(100):
                node.move_odometry(position,yaw)
                response,metric=node.global_call(homing=True);return_metrics.append(metric)
                assert metric['success'],metric
                path=xyz(response.path);verify_path(path,snapshot)
                assert math.dist(path[0],position)<1e-8 and math.dist(path[-1],home)<1e-8
                if math.dist(position,home)<1e-8:break
                assert len(path)>1
                next_position=path[1];step=math.dist(position,next_position);assert step>1e-8
                q=response.path[1].orientation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                position=next_position;return_distance+=step;return_trajectory.append(position[:])
                snapshot,status=observe()
            else:raise AssertionError('Homing exceeded the bounded execution budget')
            assert math.dist(position,home)<1e-8
            save_slice(snapshot,args.output/'after_homing_map.csv')
        with (args.output/'history.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(history[0]));writer.writeheader();writer.writerows(history)
        with (args.output/'trajectory.csv').open('w',newline='') as stream:
            writer=csv.writer(stream);writer.writerow(['index','x','y','z'])
            writer.writerows([i,*p] for i,p in enumerate(trajectory))
        result={'status':'passed','scope':'ROS 2 dynamic TF + synthetic lidar + occupancy + local planning + kinematic execution',
                'requested_steps':args.steps,'executed_steps':len(history),'stop_reason':stop,
                'initial_known_cells':initial_known,'final_known_cells':last_known,'total_cells':19200,
                'distance_m':distance,'mapper_status':read(),
                'all_planned_paths_checked_against_observed_map':True,'independent_wall_check':True,
                'monotonic_known_cell_count':True,'trajectory':trajectory}
        if args.global_planning:
            result.update(scope='ROS 2 dynamic TF + synthetic lidar + local/global planning + checked kinematic homing',
                global_relocations=relocations,global_requests=global_requests,global_status=node.status(),
                homing={'status':'passed','distance_m':return_distance,'executed_edges':len(return_trajectory)-1,
                        'path':return_trajectory,'requests':return_metrics,
                        'final_home_error_m':math.dist(position,home)},
                known_cells_after_homing=status['known_cells'])
            with (args.output/'homing_trajectory.csv').open('w',newline='') as stream:
                writer=csv.writer(stream);writer.writerow(['index','x','y','z'])
                writer.writerows([i,*p] for i,p in enumerate(return_trajectory))
        temporary=args.output/'summary.tmp';temporary.write_text(json.dumps(result,indent=2)+'\n')
        temporary.replace(args.output/'summary.json');print(json.dumps(result,indent=2))
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if process:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)

if __name__=='__main__':main()

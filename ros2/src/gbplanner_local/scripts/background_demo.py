#!/usr/bin/env python3
"""Verify timer/service global expansion with original 10 m and 5 m radii."""
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
from sensor_msgs.msg import PointCloud2,PointField
from std_srvs.srv import Trigger
from global_demo import GlobalDemo

def grid_map(unknown=False):
    message=PointCloud2();message.header.frame_id='map';message.height=1
    message.width=64*64*16;message.point_step=16;message.row_step=message.width*16;message.is_dense=True
    message.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))]
    message.fields.append(PointField(name='occupancy',offset=12,datatype=PointField.INT8,count=1))
    data=bytearray()
    for z in range(16):
        for y in range(64):
            for x in range(64):data.extend(struct.pack('<fffbxxx',-8+(x+.5)*.25,-8+(y+.5)*.25,-2+(z+.5)*.25,-1 if unknown else 0))
    message.data=bytes(data);return message

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--start-node',action='store_true')
    parser.add_argument('--budget-mode',choices=('raw_draws','time'),default='raw_draws')
    parser.add_argument('--time-budget',type=float,default=.1)
    parser.add_argument('--output',type=Path,default=Path('results/background_integration.json'))
    args=parser.parse_args();node=None;process=None
    try:
        if args.start_node:
            process=subprocess.Popen(['ros2','run','gbplanner_local','local_planner_node','--ros-args',
                '-p','global.enable:=true','-p','global.background_enable:=true','-p','global.background_attempts:=200',
                '-p','global.background_budget_mode:='+args.budget_mode,
                '-p','global.background_time_budget:='+str(args.time_budget),
                '-p','global.background_period:=0.5','-p','map.origin:=[-8.0,-8.0,-2.0]',
                '-p','map.dimensions:=[64,64,16]','-p','sampling.min:=[-7.0,-7.0,-1.0]',
                '-p','sampling.max:=[7.0,7.0,1.0]','-p','planning.max_vertices:=2'],start_new_session=True)
        rclpy.init();node=GlobalDemo()
        expand=node.create_client(Trigger,'expand_global');status=node.create_client(Trigger,'expansion_status')
        assert expand.wait_for_service(timeout_sec=15) and status.wait_for_service(timeout_sec=15)
        read=lambda:json.loads(node.call(status,Trigger.Request()).message)
        assert not node.call(expand,Trigger.Request()).success
        node.publish_map(grid_map());node.move_odometry([0,0,0])
        assert not node.call(expand,Trigger.Request()).success,'Expanded before a real local planning request'
        response,metric=node.plan();assert metric['reason']=='no_gain',metric
        first=node.call(expand,Trigger.Request());assert first.success
        first_stats=json.loads(first.message)
        assert first_stats['budget_mode']==args.budget_mode and first_stats['total_added']>0,first_stats
        if args.budget_mode=='time':
            assert first_stats['rounds']>0 and first_stats['sampling_batches']>=first_stats['rounds'],first_stats
            assert first_stats['attempts']>=first_stats['accepted_samples'],first_stats
            assert first_stats['time_budget_reached'] and first_stats['elapsed_seconds']>=args.time_budget,first_stats
            assert first_stats['stop_reason']=='time_budget',first_stats
        else:
            assert first_stats['attempts']==200,first_stats
        previous=read()['updates'];node.spin_until(lambda:read()['updates']>previous,seconds=3)
        periodic=read();assert periodic['timer_enabled'] and periodic['local_triggered']
        graph_count=node.status()['vertices'];assert graph_count>1
        node.spin_until(lambda:node.global_graphs and len(node.global_graphs[-1].vertices)>1)
        graph=node.global_graphs[-1];vertices={v.id:v for v in graph.vertices}
        for edge in graph.edges:
            assert edge.source_id in vertices and edge.target_id in vertices
            a=vertices[edge.source_id].pose.position;b=vertices[edge.target_id].pose.position
            length=math.dist((a.x,a.y,a.z),(b.x,b.y,b.z))
            assert abs(length-edge.weight)<1e-5 and length>0
            # The known free map is a convex box: checking both endpoints and
            # box margins proves the entire straight edge lies in free space.
        for vertex in vertices.values():
            p=vertex.pose.position
            assert -7.8<p.x<7.8 and -7.8<p.y<7.8 and -1.8<p.z<1.8
        node.publish_map(grid_map(unknown=True))
        reset=read();assert reset['updates']==0 and not reset['local_triggered']
        assert node.status()['vertices']==0 and not node.call(expand,Trigger.Request()).success
        deadline=time.monotonic()+.7
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
        assert read()['updates']==0,'Timer expanded after map reset without a local request'
        output={'status':'passed','scope':'ROS 2 background timer and explicit expansion service',
            'first_expansion':first_stats,'periodic_expansion':periodic,'vertices_before_reset':graph_count,
            'map_dimensions':[64,64,16],'resolution':.25,'sampling_half_extent':[7,7,1],
            'original_cluster_radius':10.,'original_sparse_radius':5.,
            'checks':{'requires_local_trigger':True,'explicit_service_adds_vertices':True,
                'timer_runs_after_trigger':True,'vertices_in_free_bounds':True,'map_reset_disables_timer_work':True}}
        output['checks']['all_edges_have_valid_endpoints_and_weights']=True
        args.output.parent.mkdir(parents=True,exist_ok=True);temp=args.output.with_suffix('.tmp')
        temp.write_text(json.dumps(output,indent=2)+chr(10));temp.replace(args.output);print(json.dumps(output,indent=2))
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if process:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
            assert process.returncode==0,process.returncode

if __name__=='__main__':main()

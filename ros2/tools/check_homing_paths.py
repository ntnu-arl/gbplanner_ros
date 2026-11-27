#!/usr/bin/env python3
"""Check raw homing graph-chain publication and lifecycle on one real ROS node.

Reuse the existing aerial graph seed. Check explicit, local-budget and global
fully-explored returns, rejection, late subscription and map-reset clearing.
No controller, Action, simulator or robot is involved.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time
import traceback

sys.dont_write_bytecode=True
from check_global_home_report import require, xyz


def main():
    root=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot',type=Path,default=root/'results/global_home_report/seed1.bin')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--planner-executable',type=Path,help='Optional just-built production node; default uses ros2 run')
    args=parser.parse_args()
    seed,output=args.snapshot.resolve(),args.output.resolve()
    log_path,params=output.with_suffix('.log'),output.with_suffix('.yaml')
    require(seed.is_file(),'Existing production-core seed snapshot is required')
    require(not any(p.exists() for p in (output,log_path,params)),'Use a new output name')
    output.parent.mkdir(parents=True,exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID',str(100+os.getpid()%80))
    started=time.monotonic();deadline=started+15.
    report={'status':'failed','scope':__doc__,'cases':{},'planner_requests':0,
            'capture_trigger':'Actual homing_path_geometry and vis/homing_path subscribers; visualization.planning_diagnostics=false',
            'seed_sha256':hashlib.sha256(seed.read_bytes()).hexdigest()}
    process=node=None;initialized=False
    geometry,markers=[],[]
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from planner_msgs.srv import PlannerGlobal,PlannerHoming,PlannerSrv,PlannerStringTrigger
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy,QoSProfile,ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import PointCloud2,PointField
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from visualization_msgs.msg import Marker,MarkerArray
        rclpy.init(args=[]);initialized=True
        ns='/homing_paths_'+str(os.getpid());node=Node('probe',namespace=ns)
        durable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        clocks=node.create_publisher(Clock,ns+'/clock',10)
        maps=node.create_publisher(PointCloud2,'voxel_map',durable)
        odometry=node.create_publisher(Odometry,'odometry',10)
        node.create_subscription(String,'homing_path_geometry',lambda m:geometry.append(json.loads(m.data)),durable)
        node.create_subscription(MarkerArray,'vis/homing_path',markers.append,durable)
        clock=Clock();clock.clock.sec=100
        node.create_timer(.02,lambda:clocks.publish(clock))
        clients={}

        def until(predicate,label):
            while not predicate():
                require(process is None or process.poll() is None,'Planner exited: '+label)
                if time.monotonic()>=deadline:raise TimeoutError(label)
                rclpy.spin_once(node,timeout_sec=.01)

        def call(name,request,kind=Trigger):
            if name not in clients:clients[name]=node.create_client(kind,name)
            client=clients[name];until(client.service_is_ready,name+' discovery')
            future=client.call_async(request);until(future.done,name+' response')
            require(future.result() is not None,'No response: '+name)
            return future.result()

        def status(name):return json.loads(call(name,Trigger.Request()).message)

        def outputs(before):
            until(lambda:len(geometry)>before[0] and len(markers)>before[1],'JSON and MarkerArray snapshots')
            return geometry[-1],markers[-1]

        def checked_snapshot(data,message,populated):
            path=data['homing_path']
            require(data['frame_id']=='map' and data['stamp_ns']==100000000000 and
                    path['geometry']=='raw_graph_vertices' and path['height_convention']=='planning',
                    'Wrong frame, stamp or geometry convention')
            require(message.markers and message.markers[0].action==Marker.DELETEALL,'Snapshot lacks DELETEALL')
            for marker in message.markers:
                require(marker.header.frame_id=='map' and marker.header.stamp.sec==100 and
                        marker.header.stamp.nanosec==0,'Marker and JSON stamp/frame differ')
            if populated:
                require(data['request']=='homing' and data['success'] and path['captured'],
                        'Successful homing did not publish captured geometry')
                require(path['vertex_ids']==[4,0] and path['vertices']==[[.5,0.,0.,0.],[0.,0.,0.,0.]],
                        'Homing raw graph chain is not the existing route report')
                require(len(message.markers)==2,'Expected homing line and DELETEALL')
                marker=message.markers[1]
                require(marker.ns=='best_path' and marker.type==Marker.LINE_LIST and marker.action==Marker.ADD and
                        marker.id==0 and abs(marker.scale.x-.4)<1e-8 and
                        [marker.color.r,marker.color.g,marker.color.b,marker.color.a]==[0.,1.,0.,1.] and
                        [xyz(point) for point in marker.points]==[[.5,0.,0.],[0.,0.,0.]],
                        'Homing marker geometry/style differs from upstream')
            else:
                require(not path['captured'] and not path['vertex_ids'] and not path['vertices'] and
                        len(message.markers)==1,'Stale homing geometry survived clear')
            return {'geometry':data,'marker_count':len(message.markers)-1,'deleteall':True}

        def late(data,message):
            jsons,arrays=[],[]
            subscriptions=[node.create_subscription(String,'homing_path_geometry',lambda m:jsons.append(json.loads(m.data)),durable),
                           node.create_subscription(MarkerArray,'vis/homing_path',arrays.append,durable)]
            try:
                until(lambda:jsons and arrays,'Late subscriber snapshots')
                require(jsons[-1]==data and arrays[-1]==message,'Late subscriber retained a different snapshot')
            finally:
                for subscription in subscriptions:node.destroy_subscription(subscription)

        # Same 50k-cell free-square scene and graph attachment settings as
        # check_global_home_report, with a frozen 20.25 s mission budget for the local return branch.
        resolution,origin,dimensions=.1,(-2.,-2.,-1.),(50,50,20)
        options={'use_sim_time':True,'map.resolution':resolution,'map.origin':list(origin),'map.dimensions':list(dimensions),
            'robot.type':'kAerial','robot.size':[.1,.1,.1],'global.enable':True,'global.auto_home':False,
            'global.background_enable':False,'planning.auto_global_planner_enable':False,
            'visualization.planning_diagnostics':False,
            'planning.path_safety_enhance_enable':False,'planning.edge_min':.15,'planning.edge_max':.8,
            'planning.neighbor_radius':.8,'planning.neighbor_max':.8,'planning.overshoot':0.,'planning.graph_mode':True,
            'sensor.range':1.,'sensor.horizontal_fov':2.*math.pi,'sensor.vertical_fov':.5,
            'sensor.horizontal_resolution':.4,'sensor.vertical_resolution':.25,
            'gain.frontier_threshold':.1,'gain.unknown':1.,'gain.free':0.,'gain.occupied':0.,
            'planning.go_home_if_fully_explored':True,'mission.enabled':True,'mission.auto_homing':True,
            'mission.time_limit':20.25,'mission.homing_speed':1.,'mission.auto_landing_enable':False}
        params.write_text(json.dumps({'/**':{'ros__parameters':options}},indent=2)+'\n')
        report['parameters']=options
        command=([str(args.planner_executable.resolve())] if args.planner_executable else
                 ['ros2','run','gbplanner_local','local_planner_node'])
        command+=['--ros-args','-r','__ns:='+ns,'-r','/clock:='+ns+'/clock','--params-file',str(params)]
        report['command']=command
        with log_path.open('x') as log:process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        until(lambda:maps.get_subscription_count() and odometry.get_subscription_count(),'Input discovery')
        until(lambda:status('mission_state')['stamp_s']==100.,'Paused ROS clock')
        cloud=PointCloud2();cloud.header.frame_id,cloud.header.stamp='map',clock.clock
        cloud.height,cloud.width,cloud.point_step=1,math.prod(dimensions),16
        cloud.row_step,cloud.is_dense=cloud.width*16,True
        cloud.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(('x','y','z'))]
        cloud.fields.append(PointField(name='occupancy',offset=12,datatype=PointField.INT8,count=1))
        data=bytearray(cloud.row_step);offset=0
        for z in range(dimensions[2]):
            for y in range(dimensions[1]):
                for x in range(dimensions[0]):
                    p=[base+(index+.5)*resolution for base,index in zip(origin,(x,y,z))]
                    known=-.25<p[0]<1.25 and -.25<p[1]<1.25 and abs(p[2])<.25
                    struct.pack_into('<fffbxxx',data,offset,*p,0 if known else -1);offset+=16
        data_bytes=bytes(data);cloud.data=data_bytes;report['scene_sha256']=hashlib.sha256(data).hexdigest()
        maps.publish(cloud);until(lambda:status('map_status')['known_cells']>0,'Free square accepted')
        odom=Odometry();odom.header.frame_id,odom.header.stamp='map',clock.clock
        odom.pose.pose.position.x,odom.pose.pose.orientation.w=.5,1.
        odometry.publish(odom)
        def observed():
            response=call('sensor_gain',Trigger.Request())
            return response.success and json.loads(response.message)['root']==[.5,0.,0.,0.]
        until(observed,'Measured current point')

        before=len(geometry),len(markers)
        require(call('gbplanner/load_graph',PlannerStringTrigger.Request(message=str(seed)),PlannerStringTrigger).success,
                'Seed graph load failed')
        report['cases']['graph_load_clear']=checked_snapshot(*outputs(before),False)

        def home(label):
            before=len(geometry),len(markers)
            request=PlannerHoming.Request();request.header.frame_id='map';request.bound_mode=3
            response=call('gbplanner/homing',request,PlannerHoming);report['planner_requests']+=1
            data,message=outputs(before);record=checked_snapshot(data,message,True)
            require([xyz(p.position) for p in response.path]==[[.5,0.,0.],[0.,0.,0.]],'Homing execution response changed')
            late(data,message);record['late_subscriber_matches']=True
            report['cases'][label]=record

        home('explicit_homing')
        before=len(geometry),len(markers)
        request=PlannerHoming.Request();request.header.frame_id='wrong';request.bound_mode=3
        response=call('gbplanner/homing',request,PlannerHoming);report['planner_requests']+=1
        data,message=outputs(before);record=checked_snapshot(data,message,False)
        require(not response.path and data['reason']=='wrong_frame' and not data['success'],'Invalid homing did not clear')
        late(data,message);record['late_subscriber_matches']=True;report['cases']['failed_homing_clear']=record

        # The frozen 20.25 s budget and 0.5 m return at 1 m/s must select
        # the existing cached home route before a local graph is sampled.
        before=len(geometry),len(markers)
        request=PlannerSrv.Request();request.header.frame_id='map';request.bound_mode=3
        request.root_pose.orientation.w=0.  # Original zero-pose sentinel selects real odometry.
        response=call('gbplanner',request,PlannerSrv);report['planner_requests']+=1
        data,message=outputs(before);record=checked_snapshot(data,message,True)
        require(response.status==response.K_HOMING and data['reason']=='mission_time_homing' and
                [xyz(p.position) for p in response.path]==[[.5,0.,0.],[0.,0.,0.]],'Local budget did not use cached home route')
        report['cases']['local_budget_homing']=record

        before=len(geometry),len(markers)
        response=call('planner_global',PlannerGlobal.Request(id=3,ignore_time=True,bound_mode=3),PlannerGlobal)
        report['planner_requests']+=1
        data,message=outputs(before);record=checked_snapshot(data,message,False)
        require(response.path and not response.homing and data['request']=='frontier','Frontier replacement did not clear homing')
        report['cases']['frontier_replacement_clear']=record

        # Remove unknown gain without removing the graph. An automatic global
        # request now follows the original fully-explored home branch.
        previous_revision=status('map_status')['map_revision']
        free_cloud=copy.deepcopy(cloud);free_data=bytearray(data_bytes)
        for offset in range(12,len(free_data),16):free_data[offset]=0
        free_cloud.data=bytes(free_data);maps.publish(free_cloud)
        until(lambda:status('map_status')['map_revision']>previous_revision,'All-free map accepted')
        before=len(geometry),len(markers)
        response=call('planner_global',PlannerGlobal.Request(id=0,ignore_time=True,bound_mode=3),PlannerGlobal)
        report['planner_requests']+=1
        data,message=outputs(before);record=checked_snapshot(data,message,True)
        require(response.homing and response.path and data['reason']=='fully_explored_homing',
                'Global exhaustion did not publish the returned home chain')
        late(data,message);record['late_subscriber_matches']=True;report['cases']['global_exhaustion_homing']=record

        before=len(geometry),len(markers)
        empty=copy.deepcopy(cloud);empty.width=empty.row_step=0;empty.data=b''
        maps.publish(empty);until(lambda:status('map_status')['known_cells']==0,'Empty map accepted')
        output_json,output_markers=outputs(before)
        cleared=checked_snapshot(output_json,output_markers,False)
        require(output_json['request']=='clear' and status('global_status')['vertices']==0,'Reset retained graph/display state')
        late(output_json,output_markers);cleared['late_subscriber_matches']=True
        report['cases']['map_reset_clear']=cleared
        report['status']='passed'
    except Exception as error:
        report['error'],report['traceback']=type(error).__name__+': '+str(error),traceback.format_exc()
        report['last_geometry']=geometry[-1] if geometry else None
    finally:
        if process is not None:
            forced=False
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=4.)
                except subprocess.TimeoutExpired:
                    forced=True;os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=2.)
            report['cleanup']={'exit_code':process.returncode,'forced':forced}
            if forced or process.returncode!=0:report['status']='failed'
        if node is not None:node.destroy_node()
        if initialized and rclpy.ok():rclpy.shutdown()
        report['elapsed_s']=time.monotonic()-started
        sources=['src/gbplanner_local/src/local_planner_node.cpp','src/gbplanner_local/src/global_memory.cpp',
                 'src/gbplanner_local/include/gbplanner_local/global_path_markers.hpp','tools/check_homing_paths.py']
        report['source_sha256']={p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in sources}
        output.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'status':report['status'],'output':str(output),'elapsed_s':report['elapsed_s'],'error':report.get('error')}))
    return 0 if report['status']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())

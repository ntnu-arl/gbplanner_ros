#!/usr/bin/env python3
"""Check one native ground connector beyond neighbor_radius after graph load.

Save home alone, observe a real ROS odometry input at x=2.75, reload the saved
one-vertex graph, then request homing before publishing any more odometry.
The native planner must preserve the full current-to-home route with neighbor
radius 2.5 m and edge maximum 3 m, without fabricating a visit from the query.
No controller, follower, simulator or physical vehicle is started. Rejected
connections are covered by the separate core check, not a ROS matrix here.
"""
import argparse
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

sys.dont_write_bytecode = True


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def xyz(point):
    return [point.x, point.y, point.z]


def close(actual, expected, label):
    require(len(actual)==len(expected) and all(abs(a-b)<1e-8 for a,b in zip(actual,expected)),label)


def stamp_ns(header):
    return header.stamp.sec*1_000_000_000+header.stamp.nanosec


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--timeout',type=float,default=30.)
    args = parser.parse_args()
    require(math.isfinite(args.timeout) and args.timeout>0.,'Timeout must be finite and positive')
    output = args.output.resolve()
    log_path, stored_graph, parameter_path = output.with_suffix('.log'),output.with_suffix('.graph.bin'),output.with_suffix('.yaml')
    require(not any(path.exists() for path in (output,log_path,stored_graph,parameter_path)),
            'Use a new output name; prior evidence is preserved')
    output.parent.mkdir(parents=True,exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID',str(100+os.getpid()%80))
    started = time.monotonic()
    deadline = started+args.timeout
    report = {'status':'failed','scope':__doc__,'snapshots':{},'odometry':[],'checks':{}}
    process = node = None
    initialized = False
    try:
        import rclpy
        from nav_msgs.msg import Odometry, Path as RosPath
        from planner_msgs.msg import Graph
        from planner_msgs.srv import PlannerHoming, PlannerSetHomingPos, PlannerStringTrigger
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        resolution,origin,dimensions = .05,(-1.,-1.,-1.),(100,40,60)
        options = {
            'map.resolution':resolution,'map.origin':list(origin),'map.dimensions':list(dimensions),
            'robot.type':'kGround','robot.size':[.2,.2,.2],
            'ground.max_ground_height':.8,'ground.robot_height':.6,
            'ground.interpolate_projection_distance':False,'ground.ray_projection_mode':'voxel_surface',
            'global.enable':True,'global.auto_home':False,'global.background_enable':False,
            'mission.enabled':False,'planning.auto_global_planner_enable':False,
            'planning.path_safety_enhance_enable':False,
            'planning.edge_min':.1,'planning.edge_max':3.,'planning.neighbor_radius':2.5,
            'planning.neighbor_min':.1,'planning.neighbor_max':3.,'planning.overshoot':.1,
            'planning.path_interpolation_distance':.1,
            'sensor.range':.25,'sensor.min_range':0.,'sensor.horizontal_fov':.1,'sensor.vertical_fov':.1,
            'sensor.horizontal_resolution':.1,'sensor.vertical_resolution':.1,
        }
        parameter_path.write_text(json.dumps({'/**':{'ros__parameters':options}},indent=2)+'\n')
        report['parameters'] = options
        rclpy.init(args=[]);initialized = True
        namespace = '/global_connector_'+str(os.getpid())
        node = Node('probe',namespace=namespace)
        durable = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        maps = node.create_publisher(PointCloud2,'voxel_map',durable)
        odometry = node.create_publisher(Odometry,'odometry',10)
        accepted, metrics, paths = [], [], []
        node.create_subscription(PointCloud2,'accepted_voxel_map',accepted.append,durable)
        node.create_subscription(String,'global_metrics',lambda m:metrics.append(json.loads(m.data)),durable)
        node.create_subscription(RosPath,'global_path',paths.append,durable)
        clients = {}

        def until(predicate,label):
            while not predicate():
                require(process is not None and process.poll() is None,'Planner exited before '+label)
                if time.monotonic()>=deadline:raise TimeoutError(label)
                rclpy.spin_once(node,timeout_sec=.02)

        def call(name,request,kind=Trigger):
            if name not in clients:clients[name] = node.create_client(kind,name)
            client = clients[name]
            until(client.service_is_ready,name+' discovery')
            future = client.call_async(request)
            until(future.done,name+' response')
            return future.result()

        def status(name):
            response = call(name,Trigger.Request())
            require(response.success,name+' unavailable')
            return json.loads(response.message)

        def sample(x):
            message = Odometry()
            message.header.frame_id,message.child_frame_id = 'map','base_link'
            message.header.stamp = node.get_clock().now().to_msg()
            message.pose.pose.position.x,message.pose.pose.position.z,message.pose.pose.orientation.w = x,.6,1.
            odometry.publish(message)
            # No periodic publisher: this exact source-stamped barrier proves
            # the sole odometry callback completed before the next operation.
            def consumed():
                response = call('sensor_gain',Trigger.Request())
                return response.success and json.loads(response.message)['odometry_stamp_ns']==stamp_ns(message.header)
            until(consumed,'Exact odometry callback')
            observed = status('sensor_gain')
            close(observed['root'],[x,0.,.8,0.],'Odometry did not convert to the precise planning height')
            report['odometry'].append({'base':[x,0.,.6],'planning':observed['root'],'stamp_ns':stamp_ns(message.header)})

        def snapshot(label):
            histories, graphs = [], []
            subscriptions = [node.create_subscription(String,'state_history',lambda m:histories.append(json.loads(m.data)),durable),
                             node.create_subscription(Graph,'global_graph',graphs.append,durable)]
            try:
                until(lambda:bool(histories and graphs),label+' retained snapshot')
                graph = graphs[-1]
                record = {'history':histories[-1], 'graph':{
                    'frame':graph.header.frame_id,'stamp_ns':stamp_ns(graph.header),
                    'vertices':[[v.id,*xyz(v.pose.position)] for v in graph.vertices],
                    'edges':[[e.source_id,e.target_id,e.weight] for e in graph.edges]}}
                report['snapshots'][label] = record
                return record
            finally:
                for subscription in subscriptions:node.destroy_subscription(subscription)

        cloud = PointCloud2()
        cloud.header.frame_id,cloud.height,cloud.width,cloud.point_step = 'map',1,math.prod(dimensions),16
        cloud.row_step,cloud.is_dense = cloud.width*16,True
        cloud.fields = [PointField(name=name,offset=4*axis,datatype=PointField.FLOAT32,count=1)
                        for axis,name in enumerate(('x','y','z'))]
        cloud.fields.append(PointField(name='occupancy',offset=12,datatype=PointField.INT8,count=1))
        data = bytearray(cloud.row_step)
        offset = 0
        for iz in range(dimensions[2]):
            for iy in range(dimensions[1]):
                for ix in range(dimensions[0]):
                    point = [base+(index+.5)*resolution for base,index in zip(origin,(ix,iy,iz))]
                    struct.pack_into('<fffbxxx',data,offset,*point,100 if point[2]<0. else 0)
                    offset += 16
        cloud.data = bytes(data)
        report['scene'] = {'sha256':hashlib.sha256(data).hexdigest(),'cells':cloud.width,'ground_surface_z':0.}
        command = ['ros2','run','gbplanner_local','local_planner_node','--ros-args','-r','__ns:='+namespace,
                   '--params-file',str(parameter_path)]
        report['command'] = command
        with log_path.open('x') as log:
            process = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        until(lambda:maps.get_subscription_count()>0 and odometry.get_subscription_count()>0,'Planner inputs')
        cloud.header.stamp = node.get_clock().now().to_msg()
        maps.publish(cloud)
        until(lambda:bool(accepted),'Accepted plane map')
        require(bytes(accepted[-1].data)==bytes(cloud.data),'Accepted fixture map changed')
        sample(0.)
        require(call('gbplanner/set_homing_pos',PlannerSetHomingPos.Request(),PlannerSetHomingPos).success,'Home setup failed')
        home = snapshot('saved_home')
        require(home['graph']['vertices']==[[0,0.,0.,.8]] and not home['graph']['edges'] and
                home['history']['total_states']==0,'Saved graph is not exactly the unvisited home anchor')
        storage = PlannerStringTrigger.Request(message=str(stored_graph))
        require(call('gbplanner/save_graph',storage,PlannerStringTrigger).success and stored_graph.is_file(),'Home snapshot save failed')
        report['stored_graph_sha256'] = hashlib.sha256(stored_graph.read_bytes()).hexdigest()

        sample(2.75)
        observed = snapshot('observed_before_reload')
        require(len(observed['graph']['vertices'])==2 and observed['history']['total_states']==1,
                'Actual odometry did not first populate the live graph and visit history')
        close(observed['graph']['vertices'][-1],[1,2.75,0.,.8],'Observed graph root differs')
        close(observed['history']['states'][0],[2.75,0.,.8,0.],'Observed history differs')
        require(call('gbplanner/load_graph',storage,PlannerStringTrigger).success,'Home snapshot load failed')
        loaded = snapshot('loaded_home_before_homing')
        report['loaded_storage_status'] = status('global_graph_storage')
        require(loaded['graph']['vertices']==home['graph']['vertices'] and not loaded['graph']['edges'] and
                loaded['history']['total_states']==0 and loaded['history']['states']==[] and
                loaded['history']['restored_prefix_count']==0 and
                loaded['history']['revision']>observed['history']['revision'],
                'Reload did not remove the observed connector and restore the original empty history')
        root_before = status('sensor_gain')
        close(root_before['root'],[2.75,0.,.8,0.],'Graph load changed the actual robot root')
        require(root_before['odometry_stamp_ns']==report['odometry'][-1]['stamp_ns'],
                'Unexpected odometry arrived after loading the old graph')

        # This is the only route request. No map/odometry is published between
        # loading the one-vertex graph and this native homing call.
        request = PlannerHoming.Request()
        request.header.frame_id,request.header.stamp,request.bound_mode = 'map',node.get_clock().now().to_msg(),3
        metric_before,path_before = len(metrics),len(paths)
        response = call('exploration_homing',request,PlannerHoming)
        until(lambda:len(metrics)>metric_before and len(paths)>path_before,'Homing metric and path')
        route = [xyz(p.position) for p in response.path]
        report['homing'] = {'path':route,'path_points':len(route),'metrics':metrics[-1]}
        require(metrics[-1]['success'] and metrics[-1]['mode']=='homing' and metrics[-1]['target']==0 and
                len(route)>=2,'Native homing could not attach outside neighbor_radius')
        close(route[0],[2.75,0.,.6],'Homing truncated or shifted the actual current source')
        close(route[-1],[0.,0.,.6],'Homing endpoint differs from the saved home')
        require(all(abs(point[1])<1e-8 and abs(point[2]-.6)<1e-8 for point in route) and
                all(b[0]<=a[0]+1e-8 for a,b in zip(route,route[1:])),
                'Flat native route changed support height or reversed order')
        length = sum(math.dist(a,b) for a,b in zip(route,route[1:]))
        close([length,metrics[-1]['distance_m']],[2.75,2.75],'Full query distance was shortened')
        report['homing']['measured_length_m'] = length
        require([xyz(p.pose.position) for p in paths[-1].poses]==route and paths[-1].header.frame_id=='map',
                'Published global path differs from the service response')
        after = snapshot('after_homing')
        require(after['history']==loaded['history'],'A routing query fabricated/re-published an observed visit')
        require(len(after['graph']['vertices'])==2 and len(after['graph']['edges'])==1,
                'Native connector was not retained as one exact-source graph vertex')
        close(after['graph']['vertices'][-1],[1,2.75,0.,.8],'Graph connector shortened the current source')
        require(status('sensor_gain')['odometry_stamp_ns']==root_before['odometry_stamp_ns'] and len(report['odometry'])==2,
                'An extra odometry callback hid the loaded-graph connector case')
        report['checks'] = dict.fromkeys(['loaded_graph_has_only_home','actual_root_remains_outside_neighbor_radius',
            'homing_uses_complete_2_75m_source_to_home','response_and_published_path_match',
            'connector_retained_at_exact_planning_height','query_preserves_loaded_visit_history',
            'no_odometry_between_load_and_homing'],True)
        report['status'] = 'passed'
    except Exception as error:
        report['error'],report['traceback'] = str(error),traceback.format_exc()
    finally:
        if process is not None:
            forced = False
            if process.poll() is None:
                try:os.killpg(process.pid,signal.SIGINT)
                except ProcessLookupError:pass
                try:process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    forced=True;os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=2.)
            report['cleanup'] = {'exit_code':process.returncode,'forced':forced}
            if forced or process.returncode!=0:
                report['status']='failed';report.setdefault('error','Planner failed orderly shutdown')
        if node is not None:node.destroy_node()
        if initialized:rclpy.shutdown()
        report['elapsed_s'] = time.monotonic()-started
        source_paths = ['tools/check_global_connector.py','src/gbplanner_local/src/global_memory.cpp',
                        'src/gbplanner_local/src/local_planner_node.cpp','src/gbplanner_local/src/global_persistence.cpp']
        report['source_sha256'] = {p:hashlib.sha256((workspace/p).read_bytes()).hexdigest() for p in source_paths}
        output.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'status':report['status'],'output':str(output),'elapsed_s':report['elapsed_s'],'error':report.get('error')}))
    return 0 if report['status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())

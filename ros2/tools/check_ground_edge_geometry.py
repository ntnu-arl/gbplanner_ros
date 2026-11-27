#!/usr/bin/env python3
"""Check real directed ground-edge geometry and durable ROS marker snapshots.

One native planner processes a known ramp and then a supported-bank map with an
unknown support gap. Local and PlanAhead calls expose the original admission
records, including projected overshoot and the rejected reverse direction.
There is no motion consumer, simulator or physical robot in this fixture.
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
from check_planning_visualization import close, close_points, length, require, snapshot_record, stamp_ns, xyz


RESOLUTION = .05
ORIGIN = (-2.5, -1., -1.)
DIMENSIONS = (100, 40, 80)
ROOT = (-1.5, 0., .8, 0.)
UPHILL_LIMIT, DOWNHILL_LIMIT, OVERSHOOT = .9, .35, .15


def validate_markers(metadata, message, marker_type):
    """Compare the transport product to every recorded point, without rays."""
    require(message.markers and message.markers[0].action==marker_type.DELETEALL,
            'Snapshot does not begin with DELETEALL')
    markers = {}
    for index, marker in enumerate(message.markers):
        require(marker.header.frame_id=='map' and stamp_ns(marker.header)==metadata['stamp_ns'],
                'Marker frame or stamp differs from its geometry snapshot')
        if index==0:
            continue
        key = marker.ns, marker.id
        require(marker.action==marker_type.ADD and key not in markers, 'Duplicate or incremental marker')
        close([marker.pose.orientation.x,marker.pose.orientation.y,marker.pose.orientation.z,marker.pose.orientation.w],
              [0.,0.,0.,1.], 'Marker orientation')
        markers[key] = marker
    expected = set()
    for index, item in enumerate(metadata['geometry']['records']):
        polyline = [point[:3] for point in item['polyline']]
        color = [1.,.15,.1,1.] if not item['admitted'] else [.95,.65,.2,1.] if item['speculative'] else [.1,.8,.2,1.]
        if len(polyline)>1:
            key = 'ground_checked_polyline', index
            expected.add(key)
            require(key in markers and markers[key].type==marker_type.LINE_STRIP, 'Actual projected polyline is absent')
            close_points([xyz(point) for point in markers[key].points],polyline,'Complete original projected polyline')
            close(xyz(markers[key].pose.position),[0.,0.,0.],'Polyline transform')
            close(xyz(markers[key].scale),[.035]*3,'Polyline width')
        segment = next(((a,b) for a,b in reversed(list(zip(polyline,polyline[1:]))) if math.dist(a,b)>1e-10), None)
        arrow_from, arrow_to = segment if segment is not None else (item['checked_from'][:3],item['checked_to'][:3])
        if math.dist(arrow_from,arrow_to)>1e-10:
            key = ('ground_polyline_direction' if segment is not None else 'ground_request_direction'), index
            expected.add(key)
            require(key in markers and markers[key].type==marker_type.ARROW, 'Directed check arrow is absent')
            close_points([xyz(point) for point in markers[key].points],[arrow_from,arrow_to],'Direction uses actual terminal segment')
            close(xyz(markers[key].scale),[.025,.07,.1],'Direction arrow scale')
        key = 'ground_check_status', index
        expected.add(key)
        require(key in markers and markers[key].type==marker_type.TEXT_VIEW_FACING,'Admission status label is absent')
        close(xyz(markers[key].pose.position),[arrow_to[0],arrow_to[1],arrow_to[2]+.13],'Label location')
        text = f"{item['check_id']} {item['role']} {item['direction']} "
        text += ('overshoot' if item['overshoot'] else 'base')+' '
        text += ('admitted ' if item['admitted'] else 'rejected ')+item['status']
        if segment is None:text += ' (request)'
        if item['dropped_points']:text += ' (capped)'
        require(markers[key].text==text,'Status label changes direction, admission or prefix meaning')
        for identity in expected:
            if identity[1]==index:
                marker = markers[identity]
                close([marker.color.r,marker.color.g,marker.color.b,marker.color.a],color,'Admission color',1e-6)
    require(set(markers)==expected,'Marker snapshot contains stale or invented geometry')
    return {'records_checked':len(metadata['geometry']['records']), 'added_markers':len(markers)}


def validate_ramp(metadata):
    geometry = metadata['geometry']
    records = geometry['records']
    require(geometry['total_checks']==geometry['completed_checks']==2 and len(records)==3,
            'Ramp did not expose forward base/extension and reverse rejection')
    by_role = {(item['direction'],item['overshoot']):item for item in records}
    require(set(by_role)=={('forward',False),('forward',True),('reverse',False)}, 'Unexpected ramp check roles')
    forward, extra, reverse = by_role['forward',False], by_role['forward',True], by_role['reverse',False]
    high = [1.5,0.,1.8,0.]
    close(forward['from'],ROOT,'Logical root');close(forward['to'],high,'Logical sampled target')
    close(reverse['from'],high,'Reverse source');close(reverse['to'],ROOT,'Reverse target')
    require(forward['admitted'] and extra['admitted'] and forward['status']==extra['status']=='admissible' and
            not reverse['admitted'] and reverse['status']=='steep','Ramp directional admission differs')
    require(forward['check_id']==extra['check_id'] and reverse['check_id']!=forward['check_id'], 'Check identities lost grouping')
    close(extra['from'],forward['from'],'Overshoot keeps logical from')
    close(extra['to'],forward['to'],'Overshoot keeps logical to')
    close(extra['checked_from'],forward['from'],'Root overshoot must not extend behind root')
    delta = [b-a for a,b in zip(ROOT[:3],high[:3])]
    norm = math.sqrt(sum(value*value for value in delta))
    expected_end = [high[i]+OVERSHOOT*delta[i]/norm for i in range(3)]+[high[3]]
    close(extra['checked_to'],expected_end,'Actual overshoot input endpoint')
    close(extra['polyline'][-1][:3],[expected_end[0],0.,1.8],'Overshoot endpoint reprojects onto plateau')
    require(abs(extra['polyline'][-1][2]-extra['checked_to'][2])>.04,
            'Fixture does not distinguish requested overshoot height from projected height')
    summaries = []
    for item in records:
        require(item['role']=='parent' and not item['speculative'],'Ramp observation changed its role or support policy')
        points = item['polyline']
        require(len(points)>20,'Ramp geometry was flattened to a requested endpoint line')
        signed_slopes = []
        for a,b in zip(points,points[1:]):
            horizontal = math.hypot(b[0]-a[0],b[1]-a[1])
            if math.dist(a[:3],b[:3])<=1e-10:continue
            signed_slopes.append(math.atan2(b[2]-a[2],horizontal))
            require((b[0]-a[0])*(1 if item['direction']=='forward' else -1)>0., 'Projected ramp order changed direction')
        actual_length = length([point[:3] for point in points])
        max_up, max_down = max([0.,*signed_slopes]), max([0.,*[-slope for slope in signed_slopes]])
        mean = sum(abs(slope) for slope in signed_slopes)/len(signed_slopes)
        close([item['actual_length'],item['max_uphill'],item['max_downhill'],item['mean_inclination']],
              [actual_length,max_up,max_down,mean],'Independent polyline length and inclination summaries')
        # All emitted ramp states lie above the observed stepwise surface.
        # The .05 m cells bound the difference from the analytic ramp by one
        # voxel, without pretending that the discrete surface is continuous.
        for point in points:
            height = .5*min(2.,max(0.,point[0]+1.))
            require(abs(point[2]-(height+.8))<=RESOLUTION+1e-8 and abs(point[1])<1e-8,
                    'Projected point does not follow the known ramp surface')
        if item['admitted']:
            require(max_up<=UPHILL_LIMIT and max_down<=DOWNHILL_LIMIT,'Admitted direction exceeds slope threshold')
        else:
            require(max_down>DOWNHILL_LIMIT and max_up<=UPHILL_LIMIT,'Reverse rejection does not exceed downhill threshold')
        summaries.append({'direction':item['direction'],'overshoot':item['overshoot'],
                          'points':len(points),'length_m':actual_length,'max_uphill':max_up,'max_downhill':max_down})
    return {'directions':summaries, 'forward_length':forward['actual_length']}


def validate_gap(metadata):
    geometry = metadata['geometry']
    require(geometry['total_checks']==geometry['completed_checks']==3 and len(geometry['records'])==3,
            'Gap fixture did not retain three rejected native attempts')
    prefixes = []
    for item in geometry['records']:
        require(item['role']=='parent' and item['direction']=='forward' and not item['overshoot'] and
                item['status']=='hanging' and not item['speculative'] and not item['admitted'],
                'Two supported banks obtained hanging permission or lost rejection reason')
        close(item['from'],ROOT,'Supported left bank')
        close(item['to'],[1.5,0.,.8,0.],'Supported right bank after vertical projection')
        close(item['checked_from'],item['from'],'Gap checked from')
        close(item['checked_to'],item['to'],'Gap checked to')
        require(item['polyline'] and math.dist(item['polyline'][-1][:3],item['to'][:3])>.5,
                'Rejected gap prefix was extended to the unvisited requested endpoint')
        for point in item['polyline']:
            require(abs(point[2]-.8)<1e-8 and abs(point[0])>=.75-1e-8,
                    'Gap prefix claims supported ground inside the unknown strip')
        prefixes.append(item['polyline'])
    require(prefixes[0]==prefixes[1]==prefixes[2],'Constant repeated samples produced inconsistent captured prefixes')
    return {'rejected_attempts':3,'prefix_points':len(prefixes[0]),'prefix_endpoint':prefixes[0][-1]}


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    require(not output.exists(),'Use a new result filename')
    output.parent.mkdir(parents=True,exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID',str(100+os.getpid()%80))
    report = {'status':'failed','scope':__doc__,'phases':{},'scenes':{}}
    started, process, node, initialized = time.monotonic(), None, None, False
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from planner_msgs.msg import Graph
        from planner_msgs.srv import PlanAhead, PlannerSrv
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from visualization_msgs.msg import Marker, MarkerArray

        rclpy.init(args=[]);initialized = True
        durable = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)

        class Probe(Node):
            def __init__(self):
                super().__init__('probe',namespace=f'/ground_edge_geometry_{os.getpid()}')
                self.rpc, self.geometry, self.markers, self.graphs = {}, [], [], []
                self.metrics = {'local':[],'ahead':[]}
                self.maps = self.create_publisher(PointCloud2,'voxel_map',durable)
                self.odometry = self.create_publisher(Odometry,'odometry',10)
                self.create_subscription(String,'ground_edge_geometry',lambda m:self.geometry.append(json.loads(m.data)),durable)
                self.create_subscription(MarkerArray,'vis/ground_edge_checks',self.markers.append,durable)
                self.create_subscription(Graph,'local_graph',self.graphs.append,durable)
                for label,topic in (('local','planning_metrics'),('ahead','plan_ahead_metrics')):
                    self.create_subscription(String,topic,lambda m,values=self.metrics[label]:values.append(json.loads(m.data)),durable)

            def until(self,predicate,label):
                limit = min(started+100.,time.monotonic()+25.)
                while not predicate():
                    require(process.poll() is None,'Planner exited before '+label)
                    if time.monotonic()>=limit:raise TimeoutError(label)
                    rclpy.spin_once(self,timeout_sec=.01)

            def call(self,name,request,kind=Trigger):
                if name not in self.rpc:self.rpc[name] = self.create_client(kind,name)
                client = self.rpc[name]
                self.until(client.service_is_ready,name+' discovery')
                future = client.call_async(request)
                self.until(future.done,name+' response')
                return future.result()

            def map_status(self):
                return json.loads(self.call('map_status',Trigger.Request()).message)

            def snapshot(self,kind,revision,since,before):
                def matching():
                    return (len(self.geometry)>before and self.geometry[-1]['request']==kind and
                            self.geometry[-1]['map_revision']==revision and self.geometry[-1]['stamp_ns']>=since and
                            self.markers and self.markers[-1].markers and
                            stamp_ns(self.markers[-1].markers[0].header)==self.geometry[-1]['stamp_ns'])
                self.until(matching,kind+' matching geometry and markers')
                metadata, markers = self.geometry[-1], self.markers[-1]
                checked = validate_markers(metadata,markers,Marker)
                record = {'metadata':metadata,'markers':snapshot_record(markers),'marker_checks':checked}
                late_geometry, late_markers = [], []
                subscriptions = [self.create_subscription(String,'ground_edge_geometry',lambda m:late_geometry.append(json.loads(m.data)),durable),
                                 self.create_subscription(MarkerArray,'vis/ground_edge_checks',late_markers.append,durable)]
                try:
                    self.until(lambda:late_geometry and late_markers,'Late durable edge snapshot')
                    require(late_geometry[-1]==metadata and snapshot_record(late_markers[-1])==record['markers'],
                            'Late subscriber received a stale or incomplete directed edge snapshot')
                    record['late_subscriber_matches'] = True
                finally:
                    for subscription in subscriptions:self.destroy_subscription(subscription)
                return record

        node = Probe()
        parameters = {
            'map.resolution':RESOLUTION,'map.origin':list(ORIGIN),'map.dimensions':list(DIMENSIONS),
            'robot.type':'kGround','robot.size':[.2,.2,.2],
            'ground.max_ground_height':.8,'ground.robot_height':.6,
            'ground.max_inclination':UPHILL_LIMIT,'ground.max_negative_inclination':DOWNHILL_LIMIT,
            'ground.interpolate_projection_distance':False,'ground.ray_projection_mode':'voxel_surface',
            'global.enable':False,'global.background_enable':False,'mission.enabled':False,
            'planning.auto_global_planner_enable':False,'planning.adaptive_exploration':False,
            'planning.path_safety_enhance_enable':False,'planning.graph_mode':False,
            'planning.edge_min':.05,'planning.edge_max':4.,'planning.overshoot':OVERSHOOT,
            'planning.max_vertices':2,'planning.max_edges':100,'planning.max_attempts':3,
            'planning.path_length_penalty':0.,'planning.path_direction_penalty':0.,
            'planning.traverse_length_max':10.,'planning.traverse_time_max':100.,
            'sampling.min':[-.1,-.5,-.1],'sampling.max':[3.5,.5,1.3],
            'sampling.seed':42,'sampling.seed_mode':'fixed','visualization.planning_diagnostics':True,
            'sensor.range':1.,'sensor.horizontal_fov':math.pi/2,'sensor.vertical_fov':.5,
            'sensor.horizontal_resolution':.5,'sensor.vertical_resolution':.5,
            'gain.unknown':1.,'gain.free':.1,'gain.occupied':0.,
        }
        for axis in ('x','y','z','heading'):
            parameters[f'sampling.{axis}.distribution'] = 'constant'
            parameters[f'sampling.{axis}.constant'] = 3. if axis=='x' else 1. if axis=='z' else 0.
        command = ['ros2','run','gbplanner_local','local_planner_node','--ros-args','-r','__ns:='+node.get_namespace()]
        for key,value in parameters.items():command += ['-p',key+':='+json.dumps(value)]
        report.update(parameters=parameters,command=command)
        with output.with_suffix('.planner.log').open('x') as log:
            process = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        node.until(lambda:node.maps.get_subscription_count() and node.odometry.get_subscription_count(),'Inputs discovered')

        def publish_map(shape):
            cloud = PointCloud2()
            cloud.header.frame_id,cloud.header.stamp = 'map',node.get_clock().now().to_msg()
            cloud.height,cloud.width,cloud.point_step = 1,0 if shape=='empty' else math.prod(DIMENSIONS),16
            cloud.row_step,cloud.is_dense = cloud.width*16,True
            cloud.fields = [PointField(name=name,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(('x','y','z'))]
            cloud.fields.append(PointField(name='occupancy',offset=12,datatype=PointField.INT8,count=1))
            data = bytearray(cloud.row_step)
            offset = 0
            if shape!='empty':
                for iz in range(DIMENSIONS[2]):
                    for iy in range(DIMENSIONS[1]):
                        for ix in range(DIMENSIONS[0]):
                            x,y,z = [origin+(index+.5)*RESOLUTION for origin,index in zip(ORIGIN,(ix,iy,iz))]
                            height = .5*min(2.,max(0.,x+1.)) if shape=='ramp' else 0.
                            status = 100 if z<height else 0
                            if shape=='gap' and abs(x)<.75 and z<.1:status = -1
                            struct.pack_into('<fffbxxx',data,offset,x,y,z,status);offset += 16
            cloud.data = bytes(data)
            before = node.map_status()['map_revision']
            node.maps.publish(cloud)
            node.until(lambda:node.map_status()['map_revision']==before+1,shape+' accepted map')
            report['scenes'][shape] = {'sha256':hashlib.sha256(data).hexdigest(),'cells':cloud.width,'revision':before+1}
            return before+1

        revision = publish_map('ramp')
        odom = Odometry()
        odom.header.frame_id,odom.header.stamp = 'map',node.get_clock().now().to_msg()
        odom.pose.pose.position.x,odom.pose.pose.position.z,odom.pose.pose.orientation.w = -1.5,.6,1.
        node.odometry.publish(odom)
        node.until(lambda:json.loads(node.call('sensor_gain',Trigger.Request()).message).get('root',[0.])[0]==-1.5,'Measured root accepted')
        for shape in ('ramp','gap'):
            if shape=='gap':revision = publish_map('gap')
            for label,kind,service in (('local',PlannerSrv,'planner'),('ahead',PlanAhead,'planner_ahead')):
                request = kind.Request()
                request.header.frame_id,request.bound_mode = 'map',PlannerSrv.Request.K_EXACT_BOUND
                request.root_pose.position.x,request.root_pose.position.z,request.root_pose.orientation.w = -1.5,.6,1.
                if label=='ahead':request.arrival_time = 1.
                before,since,metric_before = len(node.geometry),node.get_clock().now().nanoseconds,len(node.metrics[label])
                response = node.call(service,request,kind)
                node.until(lambda:len(node.metrics[label])>metric_before,shape+' '+label+' metrics')
                record = node.snapshot(label,revision,since,before)
                report['phases'][shape+'_'+label] = record
                metric = node.metrics[label][-1]
                phase = metric if label=='local' else metric['plan']
                record.update(metrics=metric,path=[xyz(pose.position) for pose in response.path])
                geometry = record['metadata']['geometry']
                require(geometry['scope']=='ros2_directed_edge_admission' and geometry['enabled'] and
                        not geometry['partial'] and not geometry['truncated'] and geometry['dropped_records']==geometry['dropped_points']==0,
                        'Tiny completed fixture contains partial/capped capture')
                require(geometry['total_records']==len(geometry['records']) and
                        geometry['total_points']==sum(len(item['polyline']) for item in geometry['records']) and
                        all(item['total_points']==len(item['polyline']) and item['dropped_points']==0 for item in geometry['records']),
                        'Recorded geometry totals differ from actual stored points')
                if shape=='ramp':
                    record['geometry_checks'] = checked = validate_ramp(record['metadata'])
                    require(phase['success'] and phase['vertices']==2 and phase['edges']==1 and response.path,
                            'Admissible forward ramp did not produce the native route')
                    close(record['path'][0],[-1.5,0.,.6],'Executed ramp base start')
                    close(record['path'][-1],[1.5,0.,1.6],'Executed ramp base endpoint')
                    require(math.isclose(length(record['path']),checked['forward_length'],abs_tol=1e-7),
                            'Returned route length differs from the admitted projected edge')
                else:
                    record['geometry_checks'] = validate_gap(record['metadata'])
                    require(not phase['success'] and phase['vertices']==1 and phase['edges']==0 and
                            phase['collision_rejections']==3 and not response.path,
                            'Supported banks were connected through unknown support')
                if label=='local':
                    node.until(lambda:node.graphs and stamp_ns(node.graphs[-1].header)>=since,'Matching local graph')
                    graph = node.graphs[-1]
                    record['graph'] = {'vertices':[[v.id,*xyz(v.pose.position)] for v in graph.vertices],
                                       'edges':[[e.source_id,e.target_id,e.weight] for e in graph.edges]}
                    require(len(graph.vertices)==phase['vertices'] and len(graph.edges)==phase['edges'],'Graph/metric mismatch')

        before,since = len(node.geometry),node.get_clock().now().nanoseconds
        revision = publish_map('empty')
        cleared = node.snapshot('clear',revision,since,before)
        report['phases']['empty_map'] = cleared
        geometry = cleared['metadata']['geometry']
        require(not geometry['records'] and geometry['total_checks']==geometry['total_records']==geometry['total_points']==0 and
                len(cleared['markers'])==1 and not geometry['partial'] and not geometry['truncated'],
                'Empty map retained an old ground-edge display')
        report['status'] = 'passed'
    except Exception as error:
        report['error'],report['traceback'] = str(error),traceback.format_exc()
    finally:
        if process is not None:
            forced = False
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    forced = True;os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3.)
            report['cleanup'] = {'exit_code':process.returncode,'forced':forced}
            if forced or process.returncode!=0:report['status'] = 'failed'
        if node is not None:node.destroy_node()
        if initialized:rclpy.shutdown()
        report['wall_time_s'] = time.monotonic()-started
        output.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'status':report['status'],'output':str(output),'error':report.get('error')}))
    return 0 if report['status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Observe the upstream SMB dual lidar profile in native Fortress while idle.

Checks the actual robot description, TF, raw/adapted clouds, two mapper inputs
and live planner/mapper parameters. Creates no command publisher or planning,
map mutation, PCI or Action request. Natural initial settling is recorded.
"""
import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import traceback
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
from check_edgar_mapping import cloud_metadata, stamp_ns
from check_fortress_autonomy import shutdown_log_events


SENSORS = {
    'velodyne': {'frame':'velodyne', 'raw':'gazebo/velodyne_points_raw', 'adapted':'points/velodyne',
                 'width':440, 'height':16, 'minimum':.2, 'maximum':130., 'ray_maximum':131.,
                 'translation':[.23,0.,.6937], 'pitch':0., 'vertical_half_angle':math.radians(15.)},
    'os1': {'frame':'lidar_front', 'raw':'gazebo/os1_points_raw', 'adapted':'points/os1',
            'width':440, 'height':32, 'minimum':.7, 'maximum':100., 'ray_maximum':100.,
            'translation':[.3,-.3,.476], 'pitch':.5, 'vertical_half_angle':math.radians(16.611)},
}


def require(condition, message):
    if not condition:raise AssertionError(message)


def equivalent(actual, expected):
    if isinstance(expected, float):
        return isinstance(actual,(float,int)) and math.isfinite(actual) and abs(actual-expected)<1e-9
    if isinstance(expected, list):
        return isinstance(actual,list) and len(actual)==len(expected) and all(equivalent(a,b) for a,b in zip(actual,expected))
    return actual==expected


def shutdown_launch(process):
    """Let launch signal its children once; any emergency escalation fails QA."""
    if process is None:return {'started':False,'forced_kill':False}
    cleanup = {'started':True,'process_group':process.pid,'forced_kill':False,'emergency_sigterm':False}
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=9.)
        except subprocess.TimeoutExpired:pass
    def group_alive():
        try:os.killpg(process.pid,0);return True
        except ProcessLookupError:return False
    # A launch leader can exit before the final child; permit its ordinary
    # cleanup to finish before deciding whether emergency cleanup is needed.
    end = time.monotonic()+1.
    while group_alive() and time.monotonic()<end:time.sleep(.05)
    if group_alive():
        cleanup['emergency_sigterm'] = True
        try:os.killpg(process.pid,signal.SIGTERM)
        except ProcessLookupError:pass
        end = time.monotonic()+1.
        while group_alive() and time.monotonic()<end:time.sleep(.05)
    if group_alive():
        cleanup['forced_kill'] = True
        try:os.killpg(process.pid,signal.SIGKILL)
        except ProcessLookupError:pass
    try:cleanup['launch_returncode'] = process.wait(timeout=.5)
    except subprocess.TimeoutExpired:cleanup['launch_returncode'] = None
    cleanup['process_group_remaining'] = group_alive()
    return cleanup


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--wall-budget',type=float,default=120.)
    parser.add_argument('--observe-sim-seconds',type=float,default=2.)
    parser.add_argument('--renderer-prefix',type=Path,
                        help='Exercise the project renderer through the installed ROS launch')
    parser.add_argument('--gpu-rays-min-cubemap-size',type=int,choices=(128,256,512,1024),default=512)
    args = parser.parse_args()
    if args.renderer_prefix is not None:
        args.renderer_prefix=args.renderer_prefix.expanduser().resolve()
        if not args.renderer_prefix.is_dir():parser.error('--renderer-prefix must be an existing installation')
    if not math.isfinite(args.wall_budget) or not 30.<=args.wall_budget<=120.:
        parser.error('--wall-budget must be finite and between 30 and 120 seconds')
    if not math.isfinite(args.observe_sim_seconds) or not .5<=args.observe_sim_seconds<=5.:
        parser.error('--observe-sim-seconds must be between 0.5 and 5 seconds')
    output = args.output.resolve()
    log_path = output.with_suffix('.log')
    require(not output.exists() and not log_path.exists(),'Use a new output name; evidence will not be overwritten')
    output.parent.mkdir(parents=True,exist_ok=True)
    root = Path(__file__).resolve().parents[1]
    os.environ.setdefault('ROS_DOMAIN_ID',str(100+os.getpid()%80))
    started = time.monotonic()
    deadline = started+args.wall_budget-12.
    summary = {'status':'failed','scope':__doc__,'sensor_profile':'upstream',
        'domain':os.environ['ROS_DOMAIN_ID'],'wall_budget_s':args.wall_budget,
        'minimum_observation_sim_s':args.observe_sim_seconds,
        'renderer':{'selection':'project' if args.renderer_prefix is not None else 'inherit',
                    'prefix':str(args.renderer_prefix) if args.renderer_prefix is not None else None,
                    'minimum_cubemap_size':args.gpu_rays_min_cubemap_size
                        if args.renderer_prefix is not None else None},
        'motion_requested':False,'planning_requested':False,'action_or_pci_requests':0,
        'command_publisher_created':False,'map_mutation_requests':0,'checks':{},
        'native_equivalence_limit':'Original ray samples/FOV/mount/range and output gates are checked. '
            'Classic applies plugin noise between two range gates; Fortress performs native sensor noise before '
            'the final output gate. Exact noise/RNG/near-limit rejection parity is not claimed. OS1 uses native GPU rays.',
        'source_sha256':{}}
    process = node = log = None
    initialized = False
    try:
        import numpy as np
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry, Path as RosPath
        from rcl_interfaces.srv import GetParameters
        from rclpy.node import Node
        from rclpy.parameter import Parameter, parameter_value_to_python
        from rclpy.qos import DurabilityPolicy,QoSProfile,ReliabilityPolicy,qos_profile_sensor_data
        from rclpy.time import Time
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from tf2_ros import Buffer,TransformListener

        def xyz(message):
            require(message.width>0 and message.height>0 and message.row_step>=message.width*message.point_step and
                    len(message.data)==message.height*message.row_step,'Malformed cloud dimensions')
            values=[]
            for name in ('x','y','z'):
                fields=[f for f in message.fields if f.name==name]
                require(len(fields)==1 and fields[0].datatype==7 and fields[0].count==1,
                        'Cloud requires unique scalar FLOAT32 '+name)
                field=fields[0]
                require(field.offset+4<=message.point_step,'Invalid cloud field offset')
                values.append(np.ndarray((message.height,message.width),dtype='>f4' if message.is_bigendian else '<f4',
                    buffer=message.data,offset=field.offset,strides=(message.row_step,message.point_step)).reshape(-1))
            return np.column_stack(values)

        def describe(message,topic):
            points=xyz(message)
            finite=points[np.isfinite(points).all(axis=1)].astype(np.float64)
            data={**cloud_metadata(message,topic),'sha256':hashlib.sha256(message.data).hexdigest(),
                  'finite_points':len(finite),'nonfinite_points':len(points)-len(finite)}
            if len(finite):
                ranges=np.linalg.norm(finite,axis=1)
                data.update(xyz_min=finite.min(axis=0).tolist(),xyz_max=finite.max(axis=0).tolist(),
                    return_range_m=[float(ranges.min()),float(ranges.max())],
                    z_span_m=float(np.ptp(finite[:,2])),sample_xyz=finite[::max(1,len(finite)//12)][:12].tolist())
                # Rank of a centered 3-D cloud distinguishes actual spatial
                # structure from a tilted planar floor with nonzero Z span.
                eigenvalues=np.linalg.eigvalsh(np.cov(finite.T)) if len(finite)>3 else np.zeros(3)
                data['covariance_eigenvalues_m2']=eigenvalues.tolist()
                data['nonplanar_returns']=bool(eigenvalues[0]>1e-5 and data['z_span_m']>.05)
            return data

        def pair_check(raw,adapted,spec):
            require(stamp_ns(raw)==stamp_ns(adapted),'Cloud acquisition stamp changed')
            for name in ('width','height','point_step','row_step','fields','is_bigendian'):
                require(getattr(raw,name)==getattr(adapted,name),'Adapter changed '+name)
            require(adapted.header.frame_id==spec['frame'],'Wrong adapted sensor frame')
            a,b=xyz(raw),xyz(adapted)
            finite=np.isfinite(a).all(axis=1)
            ranges=np.linalg.norm(a.astype(np.float64),axis=1)
            invalid=finite & ((ranges<=spec['minimum']) | (ranges>=spec['maximum']))
            require(np.array_equal(a[finite & ~invalid],b[finite & ~invalid]),'Adapter changed an in-range XYZ')
            require(np.isnan(b[invalid]).all(),'Out-of-range finite XYZ did not become NaN')
            require(np.isfinite(b).all(axis=1).sum()==int((finite & ~invalid).sum()),'Adapter added or removed another return')
            # Permit exactly the XYZ bytes at invalid finite returns. All
            # original invalids, intensity/ring/padding and row padding stay.
            expected=bytearray(raw.data)
            offsets=[next(f.offset for f in raw.fields if f.name==name) for name in ('x','y','z')]
            for index in np.flatnonzero(invalid):
                row,column=divmod(int(index),raw.width)
                base=row*raw.row_step+column*raw.point_step
                for offset in offsets:expected[base+offset:base+offset+4]=adapted.data[base+offset:base+offset+4]
            require(bytes(expected)==bytes(adapted.data),'Adapter changed bytes outside invalid finite XYZ')
            metadata=describe(adapted,spec['adapted'])
            return {'stamp_ns':stamp_ns(raw),'raw_sha256':hashlib.sha256(raw.data).hexdigest(),
                'adapted_sha256':metadata['sha256'],'finite_raw':int(finite.sum()),
                'finite_preserved':int((finite & ~invalid).sum()),'finite_range_filtered':int(invalid.sum()),
                'raw_nonfinite_unchanged':int((~finite).sum()),'all_other_bytes_preserved':True,
                'adapted':metadata}

        class Probe(Node):
            def __init__(self):
                super().__init__('fortress_smb_sensors_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
                self.clock_ns=0
                self.counts={};self.stamps={};self.wall_stamps={};self.first={};self.latest={}
                self.cache={name:{'raw':{},'adapted':{}} for name in SENSORS}
                self.pairs={name:[] for name in SENSORS}
                self.odom=deque(maxlen=500);self.odom_count=0;self.commands=[];self.paths=0;self.statuses={}
                self.metrics={name:0 for name in ('planning_metrics','search_metrics','global_metrics')}
                self.tf=Buffer();self.listener=TransformListener(self.tf,self)
                self.create_subscription(Clock,'/clock',lambda m:setattr(self,'clock_ns',m.clock.sec*1000000000+m.clock.nanosec),qos_profile_sensor_data)
                for name,spec in SENSORS.items():
                    for kind in ('raw','adapted'):
                        self.create_subscription(PointCloud2,spec[kind],lambda m,n=name,k=kind:self.cloud(m,n,k),qos_profile_sensor_data)
                durable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
                for topic in ('voxel_map','accepted_voxel_map'):
                    self.create_subscription(PointCloud2,topic,lambda m,t=topic:self.map_cloud(m,t),durable)
                self.create_subscription(Odometry,'odometry',self.odometry,qos_profile_sensor_data)
                self.create_subscription(Twist,'cmd_vel',self.velocity,10)
                self.create_subscription(RosPath,'command_path',lambda m:setattr(self,'paths',self.paths+bool(m.poses)),durable)
                for topic in self.metrics:
                    self.create_subscription(String,topic,lambda m,t=topic:self.metric(t),durable)
                for topic in ('control_status','execution_status'):
                    self.create_subscription(String,topic,lambda m,t=topic:self.statuses.update({t:json.loads(m.data)}),durable)
                self.rpc_clients={name:self.create_client(Trigger,name) for name in
                    ('mapper_status','map_status','control_state','execution_state','global_status','gazebo_state')}
                self.parameter_clients={name:self.create_client(GetParameters,name+'/get_parameters') for name in
                    ('gbplanner_local','voxblox_mapper','fortress_state','robot_state_publisher')}

            def metric(self,name):self.metrics[name]+=1

            def cloud(self,message,name,kind):
                spec=SENSORS[name];topic=spec[kind];stamp=stamp_ns(message)
                require(stamp>0,'Unstamped '+topic)
                require(message.width==spec['width'] and message.height==spec['height'],'Wrong native ray organization: '+topic)
                self.record_cloud(message,topic)
                if len(self.pairs[name])>=12:return
                queues=self.cache[name];queues[kind][stamp]=message
                other='raw' if kind=='adapted' else 'adapted'
                if stamp in queues[other]:
                    self.pairs[name].append(pair_check(queues['raw'].pop(stamp),queues['adapted'].pop(stamp),spec))
                if len(queues[kind])>30:queues[kind].pop(next(iter(queues[kind])))

            def record_cloud(self,message,topic):
                self.counts[topic]=self.counts.get(topic,0)+1
                self.stamps.setdefault(topic,[]).append(stamp_ns(message))
                self.wall_stamps.setdefault(topic,[]).append(time.monotonic()-started)
                self.first.setdefault(topic,cloud_metadata(message,topic))
                self.latest[topic]=message

            def map_cloud(self,message,topic):
                if message.width:self.record_cloud(message,topic)

            def odometry(self,message):
                p,q=message.pose.pose.position,message.pose.pose.orientation
                t=message.twist.twist
                row={'stamp_ns':stamp_ns(message),'position':[p.x,p.y,p.z],'quaternion':[q.x,q.y,q.z,q.w],
                     'linear_velocity':[t.linear.x,t.linear.y,t.linear.z],
                     'angular_velocity':[t.angular.x,t.angular.y,t.angular.z]}
                require(all(math.isfinite(v) for k in ('position','quaternion','linear_velocity','angular_velocity') for v in row[k]),
                        'Nonfinite native odometry')
                self.odom.append(row);self.odom_count+=1

            def velocity(self,message):
                values=[message.linear.x,message.linear.y,message.linear.z,message.angular.x,message.angular.y,message.angular.z]
                require(all(math.isfinite(v) for v in values),'Nonfinite velocity output')
                if max(map(abs,values))>1e-9:self.commands.append({'sim_stamp_ns':self.clock_ns,'velocity':values})

            def until(self,predicate,label):
                while not predicate():
                    require(process.poll() is None,'Launch exited before '+label)
                    if time.monotonic()>=deadline:raise TimeoutError('Wall budget exhausted during '+label)
                    rclpy.spin_once(self,timeout_sec=.02)

            def call(self,client,request):
                self.until(client.service_is_ready,client.srv_name+' discovery')
                future=client.call_async(request)
                self.until(future.done,client.srv_name+' response')
                require(future.result() is not None,'Missing service response: '+client.srv_name)
                return future.result()

            def state(self,name):
                response=self.call(self.rpc_clients[name],Trigger.Request())
                return json.loads(response.message)

            def parameter_values(self,name,names):
                response=self.call(self.parameter_clients[name],GetParameters.Request(names=names))
                require(len(response.values)==len(names),'Incomplete parameters from '+name)
                return {key:parameter_value_to_python(value) for key,value in zip(names,response.values)}

            def statistics(self,topic):
                stamps=self.stamps.get(topic,[]);walls=self.wall_stamps.get(topic,[])
                data={'messages':len(stamps),'first_stamp_ns':stamps[0] if stamps else None,
                      'last_stamp_ns':stamps[-1] if stamps else None,'unique_stamps':len(set(stamps))}
                if len(stamps)>1:
                    delta=np.diff(stamps).astype(float)*1e-9
                    positive=delta[delta>0]
                    data.update(backward_stamps=int((delta<0).sum()),duplicate_stamps=int((delta==0).sum()),
                        observed_sim_rate_hz=(len(stamps)-1)/((stamps[-1]-stamps[0])*1e-9) if stamps[-1]>stamps[0] else None,
                        observed_wall_rate_hz=(len(walls)-1)/(walls[-1]-walls[0]) if walls[-1]>walls[0] else None,
                        acquisition_interval_s=[float(positive.min()),float(positive.max())] if len(positive) else None,
                        instantaneous_sim_rate_hz=[1./float(positive.max()),1./float(positive.min())] if len(positive) else None,
                        first_wall_s=walls[0],last_wall_s=walls[-1])
                return data

        rclpy.init(args=[]);initialized=True;node=Probe()
        source_paths=['src/gbplanner_gazebo/launch/edgar_fortress.launch.py',
            'src/gbplanner_gazebo/scripts/fortress_renderer.py',
            'src/gbplanner_gazebo/scripts/fortress_model.py','src/gbplanner_gazebo/scripts/fortress_smb_sensors.py',
            'src/gbplanner_gazebo/scripts/fortress_state_node.py','src/gbplanner_smb/config/upstream_sensors.yaml',
            'src/gbplanner_gazebo/models/smb/SOURCE.json','tools/check_fortress_smb_sensors.py']
        for relative in source_paths:
            path=root/relative
            if path.is_file():summary['source_sha256'][relative]=hashlib.sha256(path.read_bytes()).hexdigest()
        command=['ros2','launch','gbplanner_gazebo','edgar_fortress.launch.py',
                 'sensor_profile:=upstream','planning:=true','auto_home:=false','mapping:=true']
        if args.renderer_prefix is not None:
            command.extend(['renderer:=project','renderer_prefix:='+str(args.renderer_prefix),
                            'gpu_rays_min_cubemap_size:='+str(args.gpu_rays_min_cubemap_size)])
        summary['launch_command']=command
        log=log_path.open('x')
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=dict(os.environ))
        node.until(lambda:node.clock_ns>0 and node.odom and all(len(node.pairs[n])>=3 for n in SENSORS)
                   and all(t in node.latest for t in ('voxel_map','accepted_voxel_map')),'Native dual lidar and accepted map')
        observation_start=node.clock_ns

        expected_planner={'sensor.names':['velodyne'],'planning.exploration_sensors':['velodyne'],
            'sensor.velodyne.type':'kLidar','sensor.velodyne.range':20.,'sensor.velodyne.min_range':0.,
            'sensor.velodyne.center_offset':[.23,0.,0.],'sensor.velodyne.rotations':[0.,0.,0.],
            'sensor.velodyne.horizontal_fov':2.*math.pi,'sensor.velodyne.vertical_fov':math.pi/6.,
            'sensor.velodyne.horizontal_resolution':math.pi/36.,'sensor.velodyne.vertical_resolution':math.pi/36.,
            'global.auto_home':False,'ground.robot_height':.24,'ground.max_ground_height':.9337,
            'robot.size':[1.17,1.17,.84],'robot.center_offset':[0.,0.,.31]}
        expected_mapper={'input_sensors':['velodyne','os1'],'sensor.min_range':.5,'sensor.max_range':50.,
            'input.velodyne.topic':'points/velodyne','input.velodyne.frame':'velodyne',
            'input.os1.topic':'points/os1','input.os1.frame':'lidar_front'}
        expected_adapter={'sensor_names':['velodyne','os1']}
        for name,spec in SENSORS.items():
            for key,value in {'frame':spec['frame'],'input':spec['raw'],'output':spec['adapted'],
                              'output_min_range':spec['minimum'],'output_max_range':spec['maximum']}.items():
                expected_adapter['sensor.'+name+'.'+key]=value
        summary['parameters']={}
        for name,expected in [('gbplanner_local',expected_planner),('voxblox_mapper',expected_mapper),('fortress_state',expected_adapter)]:
            actual=node.parameter_values(name,list(expected));summary['parameters'][name]=actual
            require(all(equivalent(actual[k],v) for k,v in expected.items()),'Live parameters differ: '+name)
        summary['checks']['live_parameters']=True

        description=node.parameter_values('robot_state_publisher',['robot_description'])['robot_description']
        robot=ET.fromstring(description);summary['robot_description_sha256']=hashlib.sha256(description.encode()).hexdigest()
        summary['native_sensor_configuration']={};summary['sensor_tf']={}
        for name,spec in SENSORS.items():
            extensions=[g for g in robot.findall('gazebo') if g.get('reference')==spec['frame'] and g.find('sensor') is not None]
            require(len(extensions)==1,'Missing or duplicate native sensor '+name)
            sensor=extensions[0].find('sensor');lidar=sensor.find('lidar')
            require(sensor.get('type')=='gpu_lidar' and lidar is not None,'Sensor is not native gpu_lidar')
            config={'type':sensor.get('type'),'update_rate_hz':float(sensor.findtext('update_rate')),
                'horizontal_samples':int(lidar.findtext('scan/horizontal/samples')),
                'vertical_samples':int(lidar.findtext('scan/vertical/samples')),
                'horizontal_angles_rad':[float(lidar.findtext('scan/horizontal/'+k)) for k in ('min_angle','max_angle')],
                'vertical_angles_rad':[float(lidar.findtext('scan/vertical/'+k)) for k in ('min_angle','max_angle')],
                'ray_range_m':[float(lidar.findtext('range/'+k)) for k in ('min','max')],
                'noise_stddev_m':float(lidar.findtext('noise/stddev'))}
            summary['native_sensor_configuration'][name]=config
            require(config['horizontal_samples']==spec['width'] and config['vertical_samples']==spec['height'] and
                    equivalent(config['update_rate_hz'],10.) and
                    equivalent(config['ray_range_m'],[spec['minimum'],spec['ray_maximum']]),
                    'Native sampling/rate/ray range differs: '+name)
            require(equivalent(config['horizontal_angles_rad'],[-math.pi,math.pi]) and
                    equivalent(config['vertical_angles_rad'],[-spec['vertical_half_angle'],spec['vertical_half_angle']]),
                    'Wrong native FOV: '+name)
            node.until(lambda s=spec:node.tf.can_transform('base_link',s['frame'],Time()),'Sensor mount TF '+name)
            transform=node.tf.lookup_transform('base_link',spec['frame'],Time())
            t,q=transform.transform.translation,transform.transform.rotation
            actual={'translation':[t.x,t.y,t.z],'quaternion':[q.x,q.y,q.z,q.w]}
            expected_q=[0.,math.sin(spec['pitch']/2.),0.,math.cos(spec['pitch']/2.)]
            require(equivalent(actual['translation'],spec['translation']) and
                    (equivalent(actual['quaternion'],expected_q) or equivalent(actual['quaternion'],[-v for v in expected_q])),
                    'Wrong base-to-sensor transform: '+name)
            summary['sensor_tf'][name]=actual
        summary['checks']['native_configuration_and_tf']=True

        node.until(lambda:node.clock_ns-observation_start>=args.observe_sim_seconds*1e9 and
                   all(len(node.pairs[n])>=10 for n in SENSORS),'Stationary observation window')
        summary['final_states']={name:node.state(name) for name in node.rpc_clients}
        mapper=summary['final_states']['mapper_status'];accepted=summary['final_states']['map_status']
        require(mapper['backend']=='voxblox_tsdf' and mapper['known_cells']>0 and accepted['known_cells']>0,
                'Native TSDF or planner accepted map is empty')
        require(set(mapper['input_sources'])==set(SENSORS),'Mapper does not own exactly the two native sources')
        for name,spec in SENSORS.items():
            source=mapper['input_sources'][name]
            require(source['integrated_scans']>0 and source['topic'].lstrip('/')==spec['adapted'] and source['frame']==spec['frame'],
                    'Native cloud did not reach its mapper input: '+name)
            require(any(pair['adapted'].get('nonplanar_returns',False) for pair in node.pairs[name]),'No finite nonplanar returns: '+name)
            for kind in ('raw','adapted'):
                stats=node.statistics(spec[kind])
                require(stats['messages']>=10 and stats.get('backward_stamps',0)==0,'Missing or regressing scans: '+spec[kind])
        summary['checks']['both_native_inputs_integrated']=True
        summary['checks']['matched_clouds_and_nonplanar_returns']=True
        summary['observed_sensor_clouds']={spec[kind]:describe(node.latest[spec[kind]],spec[kind])
            for spec in SENSORS.values() for kind in ('raw','adapted')}
        control=summary['final_states']['control_state'];execution=summary['final_states']['execution_state']
        require(control['state']=='IDLE' and control['cycles']==0 and control['completed_actions']==0 and
                execution['accepted']==0 and not execution['active'] and node.paths==0 and not node.commands and
                not any(node.metrics.values()),'Stationary sensor check observed motion or planning work')
        require(summary['final_states']['global_status']['vertices']==0,'auto_home=false did not keep the global graph empty')
        summary['checks']['no_motion_planning_or_actions']=True
        for topic in ('voxel_map','accepted_voxel_map'):
            cloud=node.latest[topic];data=describe(cloud,topic)
            points=xyz(cloud);finite=points[np.isfinite(points).all(axis=1)].astype(np.float64)
            base=np.array(node.odom[-1]['position'])
            ranges=np.linalg.norm(finite-base,axis=1)
            data.update(coverage_origin=base.tolist(),maximum_distance_from_base_m=float(ranges.max()),
                        cells_beyond_6m=int((ranges>6.).sum()))
            summary.setdefault('map_coverage',{})[topic]=data
        summary['status']='passed'
    except Exception as error:
        summary['error']=type(error).__name__+': '+str(error)
        summary['traceback']=traceback.format_exc()
    finally:
        if node is not None:
            try:
                summary['clouds']={topic:{'statistics':node.statistics(topic),'first':node.first[topic],
                    'last':cloud_metadata(message,topic)} for topic,message in node.latest.items()}
                summary['matched_cloud_pairs']=node.pairs
                summary['odometry']={'messages':node.odom_count,'tail':list(node.odom)}
                summary['nonzero_velocity_commands']=node.commands
                summary['command_paths']=node.paths;summary['planning_metric_messages']=node.metrics
                summary['last_topic_statuses']=node.statuses
                summary['final_sim_stamp_ns']=node.clock_ns
            except Exception as error:
                summary['status']='failed';summary['evidence_error']=type(error).__name__+': '+str(error)
        summary['cleanup']=shutdown_launch(process)
        if log is not None:log.close()
        log_text=log_path.read_text() if log_path.exists() else ''
        deaths,escalations=shutdown_log_events(log_text)
        summary['node_exit_failures']=deaths
        summary['launch_signal_escalations']=escalations
        if args.renderer_prefix is not None:
            # Ignition colors individual streamed numbers, so ANSI sequences
            # can appear inside a single dimensions line.
            plain_log=re.sub(r'\x1b\[[0-9;]*m','',log_text)
            sizes=[tuple(map(int,row)) for row in re.findall(
                r'GBPlanner GPU rays internal cubemap (\d+)x(\d+), output (\d+)x(\d+)',plain_log)]
            summary['renderer']['logged_sizes']=sizes
            size=args.gpu_rays_min_cubemap_size
            summary['checks']['project_renderer_loaded']=all(
                (size,size,440,height) in sizes for height in (16,32))
            if not summary['checks']['project_renderer_loaded']:
                summary['status']='failed';summary.setdefault('error','Native renderer size was not confirmed for both lidars')
        cleanup=summary['cleanup']
        summary['checks']['clean_shutdown']=bool(cleanup.get('started') and cleanup.get('launch_returncode')==0 and
            not cleanup.get('forced_kill') and not cleanup.get('emergency_sigterm') and
            not cleanup.get('process_group_remaining') and not deaths and not escalations and
            'Traceback (most recent call last)' not in log_text)
        if not summary['checks']['clean_shutdown']:
            summary['status']='failed';summary.setdefault('error','Native launch or child process did not exit cleanly')
        if node is not None:node.destroy_node()
        if initialized and rclpy.ok():rclpy.shutdown()
        summary['wall_elapsed_s']=time.monotonic()-started
        summary['log_file']=str(log_path)
        output.write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'status':summary['status'],'output':str(output),'elapsed_s':summary['wall_elapsed_s'],
                      'error':summary.get('error')},ensure_ascii=False))
    return 0 if summary['status']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())

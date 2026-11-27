"""Independent estimated-B TF and native dual-lidar Voxblox experiment.

The caller freezes its declared initial map origin before using publish_pose.
This module never subscribes to simulator truth. No C control-point pose is
accepted: odometry must already describe physical base_link in map.
"""
from pathlib import Path
import copy
import hashlib
import json
import time

import numpy as np


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class MappingStage:
    def __init__(self, node, out, root, launch_process):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import TransformBroadcaster
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from std_srvs.srv import Trigger
        self.node,self.out,self.root,self.launch_process=node,Path(out),Path(root),launch_process
        self.odometry=node.create_publisher(Odometry,'/odometry',50)
        self.tf=TransformBroadcaster(node)
        self.TransformStamped=TransformStamped
        self.latest_odometry=None
        self.pose_messages=0
        self.cloud=None
        self.surface=None
        self.cloud_messages=0
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(PointCloud2,'/voxel_map',self._map,qos)
        node.create_subscription(PointCloud2,'/surface_pointcloud',self._surface,qos)
        self.status_client=node.create_client(Trigger,'/mapper_status')
        self.products_client=node.create_client(Trigger,'/publish_pointclouds')
        self.metadata={'scope':'Estimated planar base pose feeds original native dual lidar and Voxblox',
                       'truth_input':False,'mapper_uses_base_not_control_point':True,
                       'map_frame':'map','body_frame':'base_link','height_m':.24,
                       'height_source':'Declared flat-floor chassis geometry, not measured truth',
                       'icp_enabled':False,'is_slam':False,
                       'description':'Wheel/gyro dead reckoning plus TSDF mapping; no lidar correction or loop closure'}

    def publish_pose(self, pose):
        if pose.header.frame_id!='map' or pose.child_frame_id!='base_link':
            raise ValueError('Mapping requires the aligned physical B pose in map/base_link')
        self.latest_odometry=copy.deepcopy(pose)
        self.odometry.publish(pose)
        message=self.TransformStamped();message.header=pose.header;message.child_frame_id='base_link'
        p,q=pose.pose.pose.position,pose.pose.pose.orientation
        message.transform.translation.x=p.x;message.transform.translation.y=p.y;message.transform.translation.z=p.z
        message.transform.rotation=q
        self.tf.sendTransform(message)
        self.pose_messages+=1

    def _map(self,cloud):
        self.cloud=cloud;self.cloud_messages+=1

    def _surface(self,cloud):
        self.surface=cloud

    def start(self):
        import yaml
        from ament_index_python.packages import get_package_prefix
        sources=[self.root/'src/gbplanner_smb/config/edgar_exploration.yaml',
                 self.root/'src/gbplanner_smb/config/upstream_sensors.yaml']
        params={}
        for source in sources:params.update(yaml.safe_load(source.read_text())['voxblox_mapper']['ros__parameters'])
        params.update({'use_sim_time':True,'pointcloud_reliability':'reliable',
                       'map.snapshot_publish_period':1.0,'tf.cache_time_sec':30.0,
                       'update_mesh_every_n_sec':0.0,'enable_icp':False,
                       'clear_unknown_box_on_start':False,'esdf.clear_sphere_for_planning':False})
        config=self.out/'mapping.yaml'
        with config.open('x') as f:yaml.safe_dump({'voxblox_mapper':{'ros__parameters':params}},f,sort_keys=False)
        exe=Path(get_package_prefix('gbplanner_voxblox'))/'lib/gbplanner_voxblox/voxblox_mapper_node'
        self.metadata.update(configuration=str(config),source_sha256={str(p):digest(p) for p in sources},
                             executable=str(exe),executable_sha256=digest(exe),
                             source_module_sha256=digest(__file__))
        self.launch_process('mapper',[str(exe),'--ros-args','-r','__node:=voxblox_mapper','--params-file',str(config)])

    def call(self,client,request,label):
        self.node.spin_until(client.service_is_ready,label+' service')
        future=client.call_async(request)
        self.node.spin_until(future.done,label+' response')
        if future.exception():raise future.exception()
        return future.result()

    def status(self):
        from std_srvs.srv import Trigger
        reply=self.call(self.status_client,Trigger.Request(),'mapper status')
        return {'service_success':reply.success,**json.loads(reply.message)}

    def wait_initial_map(self):
        from std_srvs.srv import Trigger
        self.node.spin_until(self.status_client.service_is_ready,'mapper discovery')
        pending=[None];last_call=[-float('inf')];latest=[None]
        def ready():
            if pending[0] is not None and pending[0].done():
                result=pending[0].result();pending[0]=None
                latest[0]=json.loads(result.message)
                sources=latest[0].get('input_sources',{})
                if sources and all(x.get('integrated_scans',0)>0 for x in sources.values()) and self.cloud is not None:
                    return True
            if pending[0] is None and time.monotonic()-last_call[0]>=.5:
                pending[0]=self.status_client.call_async(Trigger.Request());last_call[0]=time.monotonic()
            return False
        self.node.spin_until(ready,'both native lidar streams integrated with estimated TF')
        self.metadata['initial_status']=latest[0]

    @staticmethod
    def cloud_arrays(cloud):
        # Preserve native PointCloud2 field types and row padding. These are
        # actual mapper products, not reconstructed or synthetic observations.
        types={1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}
        fields=[];formats=[];offsets=[]
        endian='>' if cloud.is_bigendian else '<'
        for field in cloud.fields:
            fields.append(field.name);formats.append(endian+types[field.datatype]);offsets.append(field.offset)
            if field.count!=1:raise ValueError('Expected scalar mapper product fields')
        dtype=np.dtype({'names':fields,'formats':formats,'offsets':offsets,'itemsize':cloud.point_step})
        rows=np.ndarray((cloud.height,cloud.width),dtype=dtype,buffer=bytes(cloud.data),
                        strides=(cloud.row_step,cloud.point_step)).reshape(-1)
        return {name:rows[name].copy() for name in fields}

    def snapshot(self, label=None):
        from std_srvs.srv import Trigger
        from gbplanner_voxblox.srv import FilePath
        reply=self.call(self.products_client,Trigger.Request(),'publish map products')
        self.node.duration(.3)
        target_dir = self.out if label is None else self.out / label
        if label is not None:
            target_dir.mkdir(parents=True, exist_ok=False)
        saved=target_dir/'estimated_map.vxblx'
        request=FilePath.Request();request.file_path=str(saved)
        client=self.node.create_client(FilePath,'/save_map')
        result=self.call(client,request,'save estimated TSDF layer')
        if not result.success:raise RuntimeError(result.message)
        products={}
        for name,cloud in (('voxel_map',self.cloud),('surface_pointcloud',self.surface)):
            if cloud is None:continue
            target=target_dir/(name+'.npz');np.savez_compressed(target,**self.cloud_arrays(cloud))
            stamp=cloud.header.stamp.sec+cloud.header.stamp.nanosec*1e-9
            products[name]={'file':str(target),'sha256':digest(target),'frame':cloud.header.frame_id,
                            'stamp_sim_s':stamp,'recorded_at_sim_s':self.node.clock_ns*1e-9,
                            'points':cloud.width*cloud.height,'fields':[f.name for f in cloud.fields]}
        metadata=copy.deepcopy(self.metadata)
        metadata.update(final_status=self.status(),products=products,
                             tsdf_layer={'path':str(saved),'sha256':digest(saved),'bytes':saved.stat().st_size},
                             estimated_pose_messages=self.pose_messages,voxel_map_messages=self.cloud_messages,
                             snapshot_service_success=reply.success,
                             mapper_subscriptions=[{'topic':t,'types':types} for t,types in self.node.get_subscriber_names_and_types_by_node('voxblox_mapper','/')],
                             odometry_publishers=[e.node_name for e in self.node.get_publishers_info_by_topic('/odometry')],
                             tf_publishers=[e.node_name for e in self.node.get_publishers_info_by_topic('/tf')])
        if label is None:
            self.metadata=metadata
            self.save()
        else:
            (target_dir/'mapping_summary.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
        return copy.deepcopy(metadata)

    def save(self):
        (self.out/'mapping_summary.json').write_text(json.dumps(self.metadata,ensure_ascii=False,indent=2)+'\n')

#!/usr/bin/env python3
"""Exercise native geofence policy with real planner/control/action processes.

Maps and odometry are a deterministic fixture. Execution uses the existing
kinematic topic consumer; this workflow does not claim a physical flight.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped
from planner_msgs.msg import RectangleShape, Graph
from planner_msgs.srv import PlannerGeofence, PciGeofence, ValidatePath
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from execution_demo import Consumer, pose
from local_demo import make_map, xyz


def rectangle(x, y, sx, sy):
    shape = RectangleShape()
    shape.center.x = float(x); shape.center.y = float(y)
    shape.size.x = float(sx); shape.size.y = float(sy)
    return shape


def segment_hits_rectangle(a, b, bounds, padding=.2):
    """Independent slab check for the axis-aligned robot footprint."""
    enter, leave = 0., 1.
    for axis in range(2):
        low, high = bounds[axis][0]-padding, bounds[axis][1]+padding
        delta = b[axis]-a[axis]
        if abs(delta)<1e-12:
            if not low <= a[axis] <= high:return False
        else:
            first, last = sorted(((low-a[axis])/delta, (high-a[axis])/delta))
            enter, leave = max(enter, first), min(leave, last)
            if enter > leave:return False
    return True


class GeofenceDemo(Consumer):
    def __init__(self):
        super().__init__()
        self.fences = self.create_client(PlannerGeofence, 'gbplanner/geofence')
        self.pci = self.create_client(PciGeofence, 'pci_geofence')
        self.clear = self.create_client(Trigger, 'gbplanner/clear_untraversable_zones')
        self.fence_state = self.create_client(Trigger, 'geofence_status')
        self.toggle = self.create_client(SetBool, 'gbplanner/enable_untraversable_polygon_subscriber')
        self.validate = self.create_client(ValidatePath, 'validate_path')
        self.controller = self.create_client(Trigger, 'control_state')
        for client in [self.fences,self.pci,self.clear,self.fence_state,self.toggle,self.validate,self.controller]:
            assert client.wait_for_service(timeout_sec=15), client.srv_name
        self.polygons = self.create_publisher(PolygonStamped, '/traversability_estimation/untraversable_polygon', 10)
        durable = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers=[];self.graphs=[];self.updates=[]
        self.create_subscription(MarkerArray,'vis/geofence',self.markers.append,durable)
        self.create_subscription(Graph,'local_graph',self.graphs.append,durable)
        self.create_subscription(String,'map_updates',lambda m:self.updates.append(json.loads(m.data)),durable)
        self.tf=StaticTransformBroadcaster(self)

    def fence_status(self):return json.loads(self.call(self.fence_state,Trigger.Request()).message)
    def control_status(self):return json.loads(self.call(self.controller,Trigger.Request()).message)

    def add(self, rectangles, frame='map', pci=False):
        request=PciGeofence.Request() if pci else PlannerGeofence.Request()
        request.header.frame_id=frame;request.rectangles=rectangles
        return self.call(self.pci if pci else self.fences,request).success

    def path_valid(self,points):
        request=ValidatePath.Request();request.header.frame_id='map'
        request.path=[pose((*point,0.)) for point in points]
        return self.call(self.validate,request)

    def polygon(self,points,frame='map'):
        self.spin_until(lambda:self.polygons.get_subscription_count()>0)
        message=PolygonStamped();message.header.frame_id=frame
        for point in points:message.polygon.points.append(Point32(x=float(point[0]),y=float(point[1]),z=0.))
        self.polygons.publish(message)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,default=Path('results/geofence/summary.json'))
    args=parser.parse_args();processes=[];node=None;report={'status':'failed'}
    try:
        for command in [
            ['ros2','run','gbplanner_local','local_planner_node','--ros-args','-p','global.enable:=true','-p','planning.geofence_checking_enable:=true',
             '-p','geofence.areas:=[staging]', '-p','geofence.staging.center:=[3.0, 3.0, 0.0]',
             '-p','geofence.staging.size:=[0.5, 0.5, 0.0]'],
            ['ros2','run','gbplanner_local','control_node.py'],
            ['ros2','run','gbplanner_execution','path_follower_node.py','--ros-args','-p','yaw_rate:=1.0']]:
            processes.append(subprocess.Popen(command,start_new_session=True))
        rclpy.init();node=GeofenceDemo();free=make_map(known=True,wall=False)
        node.publish_map(free);node.settle()
        assert node.fence_status()['areas']==1
        assert not node.path_valid([[3.,2.5,0.],[3.,3.5,0.]]).valid
        assert node.call(node.clear,Trigger.Request()).success
        for x in [-2.,-1.,0.,1.,2.]:
            node.position=(x,0.,0.,0.);node.move_odometry(node.position[:3])
        original_route,metric=node.global_call(homing=True);assert metric['success'],metric
        original_path=xyz(original_route.path);graph_before=node.status();before=node.fence_status()
        assert node.add([rectangle(.5,0.,.3,8.)],pci=True)
        status=node.fence_status();blocked,metric=node.global_call(homing=True)
        assert not blocked.path and metric['reason']=='global_target_unreachable',metric
        assert not node.path_valid([[-2.,0.,0.],[2.,0.,0.]]).valid
        assert status['observation_revision']==before['observation_revision'] and status['map_revision']>before['map_revision']
        assert node.revision()['known_cells']==19200
        node.spin_until(lambda:node.control_status()['map_revision']==status['map_revision'])
        assert node.control_status()['observation_revision']==before['observation_revision']
        blocked_graph=node.status();assert blocked_graph['suspended_edges']>0
        invalid_before=node.fence_status()
        assert not node.add([rectangle(-3.,3.,.5,.5),rectangle(1.,1.,-1.,1.)])
        invalid_after=node.fence_status()
        assert invalid_after['areas']==invalid_before['areas'] and invalid_after['revision']==invalid_before['revision']
        assert node.call(node.clear,Trigger.Request()).success
        restored,metric=node.global_call(homing=True);assert metric['success'],metric
        assert xyz(restored.path)==original_path
        graph_restored=node.status();assert graph_restored['edges']==graph_before['edges']

        # Test transformed polygons, subscriber toggling and missing-TF rejection.
        transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='fence_frame'
        transform.header.stamp=node.get_clock().now().to_msg();transform.transform.translation.x=1.
        transform.transform.translation.y=2.;transform.transform.rotation.z=math.sin(math.pi/4)
        transform.transform.rotation.w=math.cos(math.pi/4);node.tf.sendTransform(transform)
        deadline=time.monotonic()+.3
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
        assert node.add([rectangle(0.,0.,2.,.4)],frame='fence_frame',pci=True)
        assert not node.path_valid([[1.,.4,0.],[1.,3.5,0.]]).valid
        assert node.path_valid([[-2.,0.,0.],[-1.,0.,0.]]).valid
        assert node.call(node.clear,Trigger.Request()).success
        before=node.fence_status()
        assert not node.add([rectangle(0.,0.,1.,1.)],frame='missing_frame')
        assert node.fence_status()['areas']==0
        request=SetBool.Request();request.data=False;assert node.call(node.toggle,request).success
        node.polygon([[.7,1.7],[1.3,1.7],[1.3,2.3],[.7,2.3]])
        node.spin_until(lambda:node.fence_status()['ignored_polygons']>before['ignored_polygons'])
        assert node.fence_status()['areas']==0
        request.data=True;assert node.call(node.toggle,request).success
        node.polygon([[-.3,-.3],[.3,-.3],[.3,.3],[-.3,.3]],frame='fence_frame')
        node.spin_until(lambda:node.fence_status()['areas']==1)
        assert not node.path_valid([[1.,1.2,0.],[1.,2.8,0.]]).valid
        node.spin_until(lambda:node.markers and len(node.markers[-1].markers)==2)
        assert node.call(node.clear,Trigger.Request()).success

        # Local RRG must also respect the motion-only polygon restriction.
        node.position=(-2.,0.,0.,0.);node.move_odometry(node.position[:3])
        node.publish_map(make_map(known=False,wall=False))
        bounds=((-0.6,0.2),(-.6,.6))
        assert node.add([rectangle(-.2,0.,.8,1.2)])
        count=len(node.graphs);local,local_metric=node.plan();assert local_metric['success'],local_metric
        node.spin_until(lambda:len(node.graphs)>count)
        graph=node.graphs[-1];vertices={v.id:(v.pose.position.x,v.pose.position.y) for v in graph.vertices}
        for edge in graph.edges:
            assert not segment_hits_rectangle(vertices[edge.source_id],vertices[edge.target_id],bounds)
        local_path=xyz(local.path)
        assert node.path_valid(local_path).valid
        assert node.call(node.clear,Trigger.Request()).success

        # Adding a policy area ahead of a moving action must stop it without a
        # new occupancy scan. Clear it, then execute the same route successfully.
        node.publish_map(free);node.settle();node.position=(-2.,0.,0.,0.)
        node.move_odometry(node.position[:3]);node.settle()
        handle=node.send(node.goal([node.position,(2.,0.,0.,0.)]));assert handle.accepted
        node.spin_until(lambda:node.state()['state']=='EXECUTING' and node.position[0]>-1.7)
        observation=node.fence_status()['observation_revision']
        assert node.add([rectangle(.5,0.,.3,3.)],pci=True)
        stopped=node.result(handle);assert stopped.status==GoalStatus.STATUS_ABORTED
        stop_state=node.state();stop_position=list(node.position)
        assert stop_state['hold_commands']>=1 and stop_position[0]<.15, (stop_state,stop_position)
        assert node.fence_status()['observation_revision']==observation
        assert node.call(node.clear,Trigger.Request()).success
        node.settle()
        handle=node.send(node.goal([node.position,(2.,0.,0.,0.)]));assert handle.accepted
        completed=node.result(handle);assert completed.status==GoalStatus.STATUS_SUCCEEDED
        assert math.dist(node.position[:3],(2.,0.,0.))<.05 and not node.errors,node.errors
        report={'status':'passed','scope':'Native planner + PCI geofence forwarding + action execution with kinematic fixture',
                'original_path':original_path,'blocked_graph':blocked_graph,'restored_graph':graph_restored,
                'local_plan':local_metric,'local_path':local_path,'local_checked_edges':len(graph.edges),
                'stopped_position':stop_position,'execution_at_stop':stop_state,'execution_final':node.state(),
                'geofence_status':node.fence_status(),
                'checks':{'startup_rectangle_loaded':True,'rectangle_pci_header_forwarded':True,'global_route_blocked_and_restored':True,
                          'transactional_batch_rejection':True,'transformed_rectangle_and_polygon':True,
                          'missing_transform_rejected':True,'subscriber_toggle':True,'local_graph_avoids_geofence':True,
                          'geofence_edit_not_new_observation':True,'policy_change_stops_active_action':True,
                          'clear_allows_subsequent_execution':True,'marker_published':True}}
    except Exception as error:
        report['error']=str(error)
        if node:report['position']=list(node.position)
        raise
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for process in reversed(processes):
            if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+chr(10))
        print(json.dumps(report,indent=2))


if __name__=='__main__':main()

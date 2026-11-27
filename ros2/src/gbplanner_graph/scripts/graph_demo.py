#!/usr/bin/env python3
"""Exercise native ROS 2 graph transport and search on a known graph fixture."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.serialization import serialize_message, deserialize_message
from nav_msgs.msg import Odometry, Path as PathMsg
from planner_msgs.msg import Graph, Vertex, Edge
from planner_msgs.srv import PlannerSearch, PlannerHoming, PlannerSetHomingPos, PlannerStringTrigger
from planner_msgs.action import PathFollower

def pose(x, y=0.0):
    from geometry_msgs.msg import Pose
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.orientation.w = 1.0
    return p

def run(output):
    rclpy.init()
    node = rclpy.create_node('graph_demo')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    graph_pub = node.create_publisher(Graph, 'graph', qos)
    odom_pub = node.create_publisher(Odometry, 'odometry', 10)
    paths = []
    subscription = node.create_subscription(PathMsg, 'path', paths.append, qos)
    clients = {name: node.create_client(kind, name) for name, kind in (
        ('planner_search', PlannerSearch), ('planner_homing', PlannerHoming),
        ('planner_set_homing_pos', PlannerSetHomingPos),
        ('graph_save', PlannerStringTrigger), ('graph_load', PlannerStringTrigger))}
    def spin(seconds=0.25):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    def call(name, request):
        future = clients[name].call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or future.result() is None:
            raise RuntimeError('Service failed: ' + name)
        return future.result()
    def search(a, b, frame='map', bound=PlannerSearch.Request.K_NO_BOUND):
        req = PlannerSearch.Request()
        req.header.frame_id = frame
        req.source, req.target = a, b
        req.bound_mode = bound
        return call('planner_search', req)
    try:
        for name, client in clients.items():
            if not client.wait_for_service(timeout_sec=15):
                raise RuntimeError('Service unavailable: ' + name)
        graph = Graph()
        graph.header.frame_id = 'map'
        # Two routes to the same goal. Sparse IDs exercise external graph input.
        for index, x, y in [(0,0,0),(10,1,0),(20,2,0),(30,1,1),(40,9,9)]:
            v = Vertex(); v.id = index; v.pose = pose(x,y); graph.vertices.append(v)
        for a,b,cost in [(0,10,1.0),(10,20,1.0),(0,30,math.sqrt(2)),(30,20,math.sqrt(2))]:
            e = Edge();e.source_id=a;e.target_id=b;e.weight=cost;graph.edges.append(e)
        deadline = time.monotonic()+10
        while graph_pub.get_subscription_count() == 0 and time.monotonic()<deadline:
            spin(0.1)
        graph_pub.publish(graph)
        spin(0.5)
        answer = search(pose(0),pose(2))
        assert answer.success
        points = [[p.position.x,p.position.y,p.position.z] for p in answer.path]
        assert points == [[0.0,0.0,0.0],[1.0,0.0,0.0],[2.0,0.0,0.0]], points
        assert not search(pose(0),pose(9,9)).success
        assert not search(pose(0),pose(2),frame='odom').success
        assert not search(pose(0),pose(2),bound=PlannerSearch.Request.K_EXTENDED_BOUND).success
        assert not search(pose(100),pose(2)).success
        odom=Odometry();odom.header.frame_id='map';odom.pose.pose=pose(0)
        for _ in range(5):odom_pub.publish(odom);spin(0.05)
        assert call('planner_set_homing_pos',PlannerSetHomingPos.Request()).success
        odom.pose.pose=pose(2)
        for _ in range(5):odom_pub.publish(odom);spin(0.05)
        homing=PlannerHoming.Request();homing.header.frame_id='map'
        home=call('planner_homing',homing)
        assert [p.position.x for p in home.path] == [2.0,1.0,0.0]
        file_request=PlannerStringTrigger.Request()
        file_request.message='/tmp/gbplanner-demo-graph-'+str(os.getpid())+'.cdr'
        assert call('graph_save',file_request).success
        assert call('graph_load',file_request).success
        assert search(pose(0),pose(2)).success
        # Action transport schema is migrated; execution server is a later stage.
        goal=PathFollower.Goal()
        assert deserialize_message(serialize_message(goal),PathFollower.Goal) == goal
        spin(0.2)
        assert paths and paths[-1].header.frame_id == 'map'
        result = {'status':'passed','scope':'graph subsystem integration',
                  'vertices':len(graph.vertices),'edges':len(graph.edges),
                  'path':points,'path_length_m':2.0,'homing_x':[2,1,0],
                  'disconnected_rejected':True,'wrong_frame_rejected':True,
                  'unsupported_collision_mode_rejected':True,
                  'outside_snap_radius_rejected':True,'cdr_reload':True,
                  'path_topic_received':True,'action_schema_roundtrip':True}
        if output:
            target=Path(output);target.parent.mkdir(parents=True,exist_ok=True)
            target.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(result,ensure_ascii=False,indent=2))
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--start-node',action='store_true')
    parser.add_argument('--output')
    args=parser.parse_args()
    process=None
    try:
        if args.start_node:
            process=subprocess.Popen(['ros2','launch','gbplanner_graph','graph.launch.py'],start_new_session=True)
        run(args.output)
    finally:
        if process and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)

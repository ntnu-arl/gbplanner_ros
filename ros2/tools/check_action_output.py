#!/usr/bin/env python3
"""Short ROS interface check with an independent mock PathFollower driver."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = '124'
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/gbplanner_execution/scripts'))
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.task import Future
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from planner_msgs.action import PathFollower
from planner_msgs.srv import ContinuePath, ValidatePath
from path_follower_node import pose


class Probe(Node):
    def __init__(self):
        super().__init__('external_action_probe')
        self.current = (0., 0., 0., 0.)
        self.mode = 'success'; self.active = None; self.received = []
        self.feedback = []; self.cancel_count = 0; self.state = {}
        self.map_published = False
        self.validation_count = 0
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.odom = self.create_publisher(Odometry, 'odometry', 10)
        self.maps = self.create_publisher(String, 'map_updates', durable)
        self.create_subscription(String, 'execution_status',
                                 lambda msg: self.state.update(json.loads(msg.data)), durable)
        self.create_service(ValidatePath, 'validate_path', self.validate)
        self.server = ActionServer(self, PathFollower, 'pci_output_path',
                                  execute_callback=self.execute,
                                  goal_callback=self.accept, cancel_callback=self.cancel)
        self.client = ActionClient(self, PathFollower, 'path_follower')
        self.continuation = self.create_client(ContinuePath, 'continue_path')
        self.create_timer(.01, self.tick)

    def validate(self, request, response):
        self.validation_count += 1
        response.valid = True; response.reason = 'mock_free_map'; response.map_revision = 1
        return response

    def accept(self, request):
        self.received.append(request)
        return GoalResponse.REJECT if self.mode == 'reject' else GoalResponse.ACCEPT

    def cancel(self, handle):
        self.cancel_count += 1
        return CancelResponse.ACCEPT

    async def execute(self, handle):
        done = Future()
        self.active = dict(handle=handle, done=done, started=time.monotonic(), mode=self.mode)
        return await done

    def tick(self):
        active = self.active
        if active:
            handle = active['handle']; elapsed = time.monotonic()-active['started']
            feedback = PathFollower.Feedback()
            feedback.remaining_waypoints = 17; feedback.dist_to_goal = .31
            feedback.estimated_time_remaining = 42.0
            handle.publish_feedback(feedback)
            result = PathFollower.Result()
            if handle.is_cancel_requested:
                handle.canceled(); self.active = None; active['done'].set_result(result)
            elif active['mode'] == 'abort' and elapsed > .08:
                handle.abort(); self.active = None; active['done'].set_result(result)
            elif active['mode'] in ('success', 'early_success') and elapsed > .08:
                if active['mode'] == 'success':
                    endpoint = handle.request.path[-1].pose
                    self.current = (endpoint.position.x, endpoint.position.y, endpoint.position.z,
                                    2*math.atan2(endpoint.orientation.z, endpoint.orientation.w))
                handle.succeed(); result.success = True
                self.active = None; active['done'].set_result(result)
        odom = Odometry(); odom.header.frame_id = 'map'
        odom.header.stamp = self.get_clock().now().to_msg(); odom.pose.pose = pose(self.current)
        self.odom.publish(odom)
        message = String(); message.data = json.dumps({'map_revision': 1, 'known_cells': 100})
        self.maps.publish(message)
        self.map_published = True

    def until(self, predicate, seconds=5.):
        deadline = time.monotonic()+seconds
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError(str(self.state))
            rclpy.spin_once(self, timeout_sec=.01)

    def spin_for(self, seconds):
        until = time.monotonic()+seconds
        self.until(lambda: time.monotonic() >= until)

    def goal(self, mode):
        self.mode = mode; self.feedback.clear()
        goal = PathFollower.Goal(); goal.manual_path = True; goal.bound_mode = 4; goal.vel_max = .4
        for state in (self.current, (self.current[0]+.12, *self.current[1:])):
            stamped = PoseStamped(); stamped.header.frame_id = 'map'; stamped.pose = pose(state)
            goal.path.append(stamped)
        future = self.client.send_goal_async(goal, feedback_callback=lambda msg: self.feedback.append(msg.feedback))
        self.until(future.done)
        handle = future.result(); assert handle.accepted, self.state
        return handle, handle.get_result_async()


def main():
    started = time.monotonic(); node = None; process = None
    summary = {'status': 'failed', 'scope': 'native ROS action bridge with mock driver and free-map validator; no robot dynamics'}
    with tempfile.TemporaryDirectory(prefix='action_output_') as temporary:
        with open(Path(temporary)/'follower.log', 'w+') as log:
            try:
                process = subprocess.Popen(['ros2', 'run', 'gbplanner_execution', 'path_follower_node.py',
                    '--ros-args', '-p', 'output_type:=kAction', '-p', 'sample_period:=0.02',
                    '-p', 'execution_grace:=2.0'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                rclpy.init(args=[]); node = Probe()
                node.until(lambda: node.client.server_is_ready() and node.continuation.service_is_ready()
                           and node.map_published)
                node.spin_for(.2)
                node.spin_for(.1)
                handle, result = node.goal('success'); node.until(result.done)
                reply = result.result()
                assert reply.status == GoalStatus.STATUS_SUCCEEDED and reply.result.success, node.state
                forwarded = node.received[-1]
                assert forwarded.manual_path and forwarded.bound_mode == 4 and abs(forwarded.vel_max-.4)<1e-9
                assert len(forwarded.path)>2 and all(p.header.frame_id == 'map' for p in forwarded.path)
                assert any(f.estimated_time_remaining == 42. for f in node.feedback), node.feedback
                assert node.state['trajectory_commands'] == 0 and node.state['command_subscribers'] == 0
                summary['success'] = {'sampled_poses': len(forwarded.path), 'tracker_eta_forwarded': 42.,
                                      'bounds_speed_manual_preserved': True, 'topic_consumer_required': False}

                # A successful driver result cannot replace observed arrival.
                handle, result = node.goal('early_success')
                node.until(lambda: node.state.get('reason') == 'external_success_waiting_for_odometry')
                node.spin_for(.1); assert not result.done()
                endpoint = node.received[-1].path[-1].pose.position
                node.current = (endpoint.x, endpoint.y, endpoint.z, 0.)
                node.until(result.done); assert result.result().result.success
                summary['arrival'] = 'waited for stationary endpoint odometry after early success'

                for mode in ('reject', 'abort'):
                    handle, result = node.goal(mode); node.until(result.done)
                    assert result.result().status == GoalStatus.STATUS_ABORTED and not result.result().result.success
                    summary[mode] = 'outer action aborted'

                handle, result = node.goal('hold'); node.until(lambda: node.active is not None)
                request = ContinuePath.Request(); request.header.frame_id = 'map'
                request.goal_id = handle.goal_id.uuid
                future = node.continuation.call_async(request); node.until(future.done)
                assert not future.result().accepted and future.result().reason == 'external_action_cannot_continue'
                cancel = handle.cancel_goal_async(); node.until(cancel.done); node.until(result.done)
                assert result.result().status == GoalStatus.STATUS_CANCELED, node.state
                summary['cancel'] = {'forwarded': node.cancel_count, 'terminal_result_and_stop_observed': True,
                                     'continuation': future.result().reason}

                # SIGINT must cancel the independent driver before DDS exits.
                handle, result = node.goal('hold'); node.until(lambda: node.active is not None)
                before = node.cancel_count
                os.killpg(process.pid, signal.SIGINT)
                node.until(lambda: process.poll() is not None)
                assert node.cancel_count == before+1 and node.active is None
                summary['shutdown'] = 'SIGINT forwarded cancel and received terminal result'
                summary['validation_calls'] = node.validation_count
                summary['status'] = 'passed'
            except Exception as error:
                summary['error'] = f'{type(error).__name__}: {error}'
                log.flush(); log.seek(0); summary['node_log'] = log.read()[-4000:]
            finally:
                if process and process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    if node:
                        try: node.until(lambda: process.poll() is not None, 3.)
                        except TimeoutError: os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3.)
                if node: node.destroy_node()
                if rclpy.ok(): rclpy.shutdown()
                summary['elapsed_s'] = round(time.monotonic()-started, 3)
                summary['processes_stopped'] = process is None or process.poll() is not None
    output = ROOT/'results/action_output/summary.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(summary, ensure_ascii=False))
    return 0 if summary['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Check source kTopic follower Actions against a paused/slowed ROS clock.

Uses real ROS Actions, clock/odometry messages and published trajectories.
Odometry and successful map validation are explicit protocol fixtures, not
vehicle physics or collision evidence. No installed files are modified.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=workspace/'results/topic_trajectory_clock/native.json')
    args = parser.parse_args()
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', '174')
    source = workspace/'src/gbplanner_execution/scripts/path_follower_node.py'
    result = {'status':'failed', 'scope':__doc__, 'checks':{}, 'measurements':{},
              'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest()}
    started = time.monotonic()
    deadline = started+30.
    process, node = None, None
    initialized = False
    try:
        import rclpy
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import Pose, PoseStamped
        from nav_msgs.msg import Odometry
        from planner_msgs.action import PathFollower
        from planner_msgs.srv import ContinuePath, ValidatePath
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.task import Future
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from trajectory_msgs.msg import MultiDOFJointTrajectory

        rclpy.init(args=[])
        initialized = True
        namespace = '/topic_clock_'+str(os.getpid())
        node = rclpy.create_node('probe', namespace=namespace)
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        group = ReentrantCallbackGroup()
        clocks = node.create_publisher(Clock, '/clock', 10)
        odometry = node.create_publisher(Odometry, 'odometry', 10)
        maps = node.create_publisher(String, 'map_updates', durable)
        missions = node.create_publisher(String, 'mission_status', durable)
        client = ActionClient(node, PathFollower, 'path_follower')
        state_client = node.create_client(Trigger, 'execution_state')
        continue_client = node.create_client(ContinuePath, 'continue_path')
        trajectories, statuses, feedback = [], [], []
        node.create_subscription(MultiDOFJointTrajectory, 'command/trajectory', trajectories.append, 10)
        node.create_subscription(String, 'execution_status', lambda m:statuses.append(json.loads(m.data)), durable)
        held_validation = []
        hold_validation = False

        async def validate(request, response):
            if hold_validation:
                held = Future()
                held_validation.append(held)
                await held
            response.valid, response.reason, response.map_revision = True, 'protocol_fixture', 1
            return response
        node.create_service(ValidatePath, 'validate_path', validate, callback_group=group)
        sim, x = 10., 0.

        def stamp_ns(stamp):
            return int(stamp.sec)*1000000000+int(stamp.nanosec)

        def trajectory_x(message):
            elapsed = sim-stamp_ns(message.header.stamp)*1e-9
            previous_t = 0.
            previous_x = message.points[0].transforms[0].translation.x
            for point in message.points:
                next_t = stamp_ns(point.time_from_start)*1e-9
                next_x = point.transforms[0].translation.x
                if elapsed <= next_t:
                    fraction = min(1., max(0., (elapsed-previous_t)/(next_t-previous_t)))
                    return previous_x+fraction*(next_x-previous_x)
                previous_t, previous_x = next_t, next_x
            return previous_x

        def pump(seconds=.03, rate=0., follow=False):
            nonlocal sim, x
            stop = time.monotonic()+seconds
            previous = next_publish = time.monotonic()
            while time.monotonic()<stop:
                now = time.monotonic()
                require(now<deadline, 'Clock probe exceeded wall deadline')
                if process is not None:
                    require(process.poll() is None, 'Follower exited unexpectedly')
                sim += rate*(now-previous)
                previous = now
                if now >= next_publish:
                    message = Clock()
                    nanos = round(sim*1e9)
                    message.clock.sec, message.clock.nanosec = divmod(nanos, 1000000000)
                    clocks.publish(message)
                    if follow and trajectories:
                        x = trajectory_x(trajectories[-1])
                    observed = Odometry()
                    observed.header.frame_id, observed.child_frame_id = 'map', 'base_link'
                    observed.header.stamp = message.clock
                    observed.pose.pose.position.x = x
                    observed.pose.pose.orientation.w = 1.
                    odometry.publish(observed)
                    maps.publish(String(data=json.dumps({'map_revision':1, 'known_cells':100})))
                    next_publish = now+.01
                rclpy.spin_once(node, timeout_sec=.003)

        def until(condition, label, rate=0., follow=False):
            limit = min(deadline, time.monotonic()+5.)
            while not condition():
                require(time.monotonic()<limit, label)
                pump(rate=rate, follow=follow)

        def wait(future):
            until(future.done, 'ROS response timed out')
            return future.result()

        def status():
            return json.loads(wait(state_client.call_async(Trigger.Request())).message)

        def goal(length=.2):
            pump(.08, rate=.2)
            request = PathFollower.Goal()
            for target in (x, x+length):
                item = PoseStamped()
                item.header.frame_id = 'map'
                item.pose.position.x, item.pose.orientation.w = target, 1.
                request.path.append(item)
            handle = wait(client.send_goal_async(request,
                feedback_callback=lambda m:feedback.append({'eta':float(m.feedback.estimated_time_remaining),
                    'remaining_waypoints':int(m.feedback.remaining_waypoints), 'wall_s':time.monotonic()-started})))
            require(handle.accepted, 'Source follower rejected goal')
            return handle, handle.get_result_async()

        def continuation(handle, generation, endpoint, budget=False):
            request = ContinuePath.Request()
            request.header.frame_id = 'map'
            request.goal_id, request.expected_generation = handle.goal_id.uuid, generation
            request.budget_enabled, request.required_time_after_root = budget, 20. if budget else 0.
            for target in (endpoint, endpoint+.2):
                item = Pose()
                item.position.x, item.orientation.w = target, 1.
                request.path.append(item)
            return wait(continue_client.call_async(request))

        command = [sys.executable, str(source), '--ros-args', '-r', '__ns:='+namespace,
                   '-p', 'use_sim_time:=true', '-p', 'sample_period:=0.05',
                   '-p', 'speed:=1.0', '-p', 'smooth_yaw:=false',
                   '-p', 'odom_timeout:=0.5', '-p', 'execution_grace:=0.25',
                   '-p', 'validation_timeout:=0.3']
        result['command'] = command
        with output.with_suffix('.log').open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True, env=dict(os.environ))
        until(lambda:client.server_is_ready() and state_client.service_is_ready() and
                    continue_client.service_is_ready(), 'Follower discovery')
        until(lambda:status()['odometry_freshness']['fresh'] and status()['known_cells']>0,
              'Follower input readiness', rate=.1)

        # Paused wall time exceeds the whole route plus endpoint grace.
        handle, done = goal()
        until(lambda:status()['state']=='EXECUTING', 'Initial topic dispatch')
        initial = status()
        initial_command = trajectories[-1]
        require(initial['topic_timing']['header_stamp_ns']==stamp_ns(initial_command.header.stamp),
                'Stored origin differs from actual command header')
        paused_eta = feedback[-1]['eta'] if feedback else None
        pause_start = time.monotonic()
        pump(.85)
        paused = status()
        require(not done.done() and paused['state']=='EXECUTING', 'Paused clock expired running Action')
        require(paused['topic_timing']['elapsed_s']==initial['topic_timing']['elapsed_s'],
                'Paused trajectory elapsed advanced')
        require(abs(feedback[-1]['eta']-(initial['topic_timing']['duration_s']-
                    initial['topic_timing']['elapsed_s']))<1e-5, 'Paused ETA advanced with wall time')
        result['checks']['paused_elapsed_eta_and_grace']=True
        result['measurements']['paused']={'wall_s':time.monotonic()-pause_start,
            'initial':initial['topic_timing'], 'paused':paused['topic_timing'], 'eta':feedback[-1]['eta']}
        pump(.5, rate=.1, follow=True)
        slowed = status()
        advance = slowed['topic_timing']['elapsed_s']-paused['topic_timing']['elapsed_s']
        require(.03<advance<.09 and not done.done(), 'Low-RTF trajectory did not use ROS seconds')
        result['checks']['low_rtf_uses_ros_elapsed']=True
        result['measurements']['slow_elapsed_delta_s']=advance
        until(done.done, 'Resumed trajectory did not finish', rate=2., follow=True)
        require(done.result().status==GoalStatus.STATUS_SUCCEEDED and done.result().result.success,
                'Resumed original Action failed')
        result['checks']['same_action_resumes_to_observed_endpoint']=True

        # With a frozen simulation, age a near-limit budget in wall time only.
        # Correct ROS age accepts it; subtracting wall age would reject it.
        handle, done = goal()
        until(lambda:status()['state']=='EXECUTING', 'Budget route dispatch')
        before = status()
        timing = before['topic_timing']
        eta = timing['duration_s']-timing['elapsed_s']
        missions.publish(String(data=json.dumps({'enabled':True, 'valid':True,
                            'remaining_s':20.+eta+.1})))
        pump(.8)
        reply = continuation(handle, 0, before['endpoint'][0], budget=True)
        require(reply.accepted and reply.generation==1, 'Frozen-clock continuation budget was rejected: '+reply.reason)
        committed = status()
        require(committed['topic_timing']['header_stamp_ns']==stamp_ns(trajectories[-1].header.stamp) and
                committed['topic_timing']['elapsed_s']==0., 'Continuation did not reset to published ROS origin')
        result['checks']['continuation_ros_origin_and_budget_age']=True
        result['measurements']['budget']={'eta_s':eta, 'remaining_s':20.+eta+.1,
            'paused_wall_before_request_s':.8, 'reply':reply.reason, 'after_commit':committed['topic_timing']}
        pump(1.3)
        stale = continuation(handle, 1, committed['endpoint'][0], budget=True)
        require(not stale.accepted and stale.reason=='continuation_budget_unavailable',
                'Paused clock disabled wall communication-health deadline')
        require(not done.done(), 'Paused replacement trajectory expired')
        result['checks']['mission_communication_deadline_remains_steady']=True
        until(done.done, 'Continued trajectory did not finish', rate=2., follow=True)
        require(done.result().status==GoalStatus.STATUS_SUCCEEDED, 'Continued Action failed')

        # Existing high-water rejection remains in front of timing/arrival;
        # frozen /clock also does not freeze the stop confirmation deadline.
        handle, done = goal()
        until(lambda:status()['state']=='EXECUTING', 'Rewind route dispatch')
        high_water = sim
        sim -= .4
        pump(.7)
        require(done.done() and done.result().status==GoalStatus.STATUS_ABORTED,
                'Clock regression did not abort through steady stop deadline')
        rewound = status()
        require(rewound['reason']=='odometry_lost:hold_unconfirmed', 'Unexpected rewind result')
        result['checks']['rewind_guard_and_steady_stop_deadline']=True
        result['measurements']['rewind_reason']=rewound['reason']

        # A held validator is a communication failure, not a motion timer.
        sim = high_water+.1
        pump(.1, rate=.1)
        hold_validation = True
        handle, done = goal()
        until(lambda:bool(held_validation), 'Held validator was not called')
        pump(.45)
        require(done.done() and done.result().status==GoalStatus.STATUS_ABORTED and
                status()['reason']=='validation_timeout', 'Paused clock froze validation timeout')
        result['checks']['validation_deadline_remains_steady']=True
        hold_validation = False
        for pending in held_validation:
            pending.set_result(True)
        pump(.05)
        result['stats']=status()
        result['status']='passed'
    except Exception as error:
        result['error']=type(error).__name__+': '+str(error)
        if 'statuses' in locals():
            result['last_status_event']=statuses[-1] if statuses else None
    finally:
        if process is not None:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=4.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2.)
            result['process_exit']=process.returncode
            if process.returncode!=0:
                result['status']='failed'
                result.setdefault('error','Follower did not exit cleanly')
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        result['wall_elapsed_s']=time.monotonic()-started
        output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({key:result.get(key) for key in ('status','checks','error','wall_elapsed_s')}))
    return 0 if result['status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())

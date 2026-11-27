#!/usr/bin/env python3
"""Exercise real tracking Actions with a deliberately paused ROS simulation clock.

This is a clock-contract check with synthetic odometry, not vehicle physics.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID', str(120+os.getpid()%50))
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped, Twist
    from std_msgs.msg import String
    from planner_msgs.action import PathFollower
    from std_srvs.srv import Trigger
    from check_edgar_mapping import stop_process_tree

    result = {'status': 'failed', 'scope': 'Synthetic ROS clock/odometry, real tracking Actions', 'checks': {}}
    started = time.monotonic()
    rclpy.init(args=[])
    node = rclpy.create_node('sim_clock_probe')
    clock = node.create_publisher(Clock, '/clock', 10)
    odometry = node.create_publisher(Odometry, 'odometry', 10)
    action = ActionClient(node, PathFollower, 'pci_output_path')
    durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    events, commands = [], []
    node.create_subscription(String, 'tracking_status', lambda m: events.append(json.loads(m.data)), durable)
    node.create_subscription(Twist, 'cmd_vel', lambda m: commands.append(
        {'wall_s': time.monotonic()-started, 'v': m.linear.x, 'w': m.angular.z}), 10)
    sim = 10.
    deadline = started+30.
    def require(test, message):
        if not test: raise AssertionError(message)
    def pump(duration, advance=False, position=None, fixed_stamp=None):
        nonlocal sim
        stop = time.monotonic()+duration
        previous = time.monotonic()
        next_publish = previous
        while time.monotonic() < stop:
            now = time.monotonic()
            require(now < deadline, 'Clock-contract wall budget exhausted')
            if advance: sim += now-previous
            previous = now
            if now >= next_publish:
                message = Clock(); message.clock.sec = int(sim); message.clock.nanosec = int((sim-int(sim))*1e9)
                clock.publish(message)
                if position is not None:
                    stamp = sim if fixed_stamp is None else fixed_stamp
                    m=Odometry();m.header.frame_id='map';m.child_frame_id='base_link'
                    m.header.stamp.sec=int(stamp);m.header.stamp.nanosec=int((stamp-int(stamp))*1e9)
                    m.pose.pose.position.x=position;m.pose.pose.orientation.w=1.
                    odometry.publish(m)
                next_publish=now+.02
            rclpy.spin_once(node,timeout_sec=.005)
    def wait(future, position=None):
        while not future.done(): pump(.02, advance=position is not None, position=position)
        return future.result()
    def goal(start=0.):
        pump(.15,advance=True,position=start)
        g=PathFollower.Goal()
        for x in (start,start+1.):
            p=PoseStamped();p.header.frame_id='map';p.pose.position.x=x;p.pose.orientation.w=1.;g.path.append(p)
        handle=wait(action.send_goal_async(g),start)
        require(handle.accepted, 'Tracking goal was not accepted')
        return handle.get_result_async()

    log=args.output.with_suffix('.log').open('w')
    process=subprocess.Popen(['ros2','run','gbplanner_tracking','pure_pursuit_node','--ros-args',
        '-p','use_sim_time:=true','-p','odom_timeout:=0.5'],stdout=log,stderr=subprocess.STDOUT,
        start_new_session=True)
    try:
        while not action.server_is_ready(): pump(.05,advance=True,position=0.)
        future=goal()
        pump(.15,advance=True,position=0.)
        require(any(abs(c['v'])>0 for c in commands), 'No initial movement command')
        pause_started=time.monotonic()-started
        pump(.75)
        require(not future.done(), 'Paused simulation aborted a live Action')
        require(events[-1].get('odometry_paused') is True, 'Tracker did not expose its observation pause')
        tail=[c for c in commands if c['wall_s']>pause_started+.6]
        require(tail and all(c['v']==0 and c['w']==0 for c in tail), 'Old odometry kept a nonzero command alive')
        result['checks']['paused_clock_stops_without_aborting']=True
        resumed_at=len(commands)
        pump(.2,advance=True,position=.3)
        require(any(c['v']>0 for c in commands[resumed_at:]), 'New odometry did not resume control')
        pump(.2,advance=True,position=1.)
        require(wait(future,1.).result.success, 'Resumed Action did not reach measured endpoint')
        result['checks']['fresh_observation_resumes_same_action']=True

        future=goal(1.)
        pump(.75,advance=True)
        require(future.done() and not future.result().result.success, 'Advancing clock without odometry failed to abort')
        result['checks']['advancing_clock_still_aborts_stale_odometry']=True

        future=goal(1.)
        repeated=sim
        pump(.75,advance=True,position=1.,fixed_stamp=repeated)
        require(future.done() and not future.result().result.success, 'Duplicate samples renewed liveness')
        result['checks']['duplicate_source_stamp_does_not_renew']=True

        future=goal(1.)
        sim-=1.
        pump(.15,position=1.)
        require(future.done() and not future.result().result.success, 'Backward clock did not revoke stale pose')
        result['checks']['backwards_clock_rejected']=True
        result['status']='passed'
    except Exception as error:
        import traceback
        result.update(error=str(error),traceback=traceback.format_exc())
    finally:
        process.send_signal(signal.SIGINT)
        try: process.wait(timeout=5.)
        except subprocess.TimeoutExpired: pass
        result['cleanup']=stop_process_tree(process)
        result['events']=events;result['commands']=commands
        result['wall_elapsed_s']=time.monotonic()-started
        args.output.write_text(json.dumps(result,indent=2)+'\n')
        log.close();node.destroy_node();rclpy.shutdown()
    print(json.dumps({k:result.get(k) for k in ('status','checks','error','wall_elapsed_s')}))
    return result['status']!='passed'

if __name__=='__main__':
    raise SystemExit(main())

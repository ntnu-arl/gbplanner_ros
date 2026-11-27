#!/usr/bin/env python3
"""Directed ROS shutdown checks for PCI; synthetic callback timing, no vehicle physics."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

root=Path(__file__).resolve().parents[1]
source=root/'src/gbplanner_local/scripts/control_node.py'
out=root/'results/control_shutdown'
out.mkdir(parents=True,exist_ok=True)
os.environ['ROS_DOMAIN_ID']='188'
result={'status':'failed','source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'scope':'Native ROS2 source-node signal teardown; controlled callback timing, no vehicle simulation.',
        'cases':{}}
started=time.monotonic()
rclpy.init(args=[])
probe=Node('shutdown_probe')
process=None
try:
    for signum in (signal.SIGINT,signal.SIGTERM):
        name=signum.name.lower()
        ns='/control_shutdown_'+name
        client=probe.create_client(Trigger,ns+'/control_state')
        publisher=probe.create_publisher(Odometry,ns+'/odometry',10)
        log_path=out/(name+'.log')
        with log_path.open('w') as log:
            process=subprocess.Popen([sys.executable,str(source),'--ros-args','-r','__ns:='+ns],stdout=log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+8.
            while not client.service_is_ready():
                assert process.poll() is None,'node exited before discovery'
                assert time.monotonic()<deadline,'service discovery timeout'
                rclpy.spin_once(probe,timeout_sec=.02)
            future=client.call_async(Trigger.Request())
            while not future.done():
                assert time.monotonic()<deadline,'service response timeout'
                odom=Odometry();odom.header.frame_id='map'
                odom.header.stamp=probe.get_clock().now().to_msg();odom.pose.pose.orientation.w=1.
                publisher.publish(odom)
                rclpy.spin_once(probe,timeout_sec=.005)
            assert future.result().success
            stopped=time.monotonic()
            process.send_signal(signum)
            if process.poll() is None:process.send_signal(signum)
            code=process.wait(timeout=4.)
        text=log_path.read_text()
        assert code==0,(name,code,text)
        assert not any(s in text for s in ['Destroyable','never retrieved','Traceback','Exception ignored']),text
        result['cases'][name]={'status':'passed','exit_code':code,'shutdown_s':time.monotonic()-stopped,
                               'real_control_state_response':True}
        process=None
        probe.destroy_client(client);probe.destroy_publisher(publisher)

    wrapper='''import os,sys,time
sys.path.insert(0, {scripts!r})
import control_node as control
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
class Instrumented(control.PlannerControl):
    def __init__(self):
        super().__init__()
        self.probe_group=ReentrantCallbackGroup()
        self.suspend_timer=self.create_timer(.01,self.suspend,callback_group=self.probe_group)
        self.slow_timer=self.create_timer(.02,self.slow,callback_group=self.probe_group)
    async def suspend(self):
        self.suspend_timer.cancel()
        print('SUSPENDED',flush=True)
        try:await Future()
        finally:print('COROUTINE_CLOSED',flush=True)
    def slow(self):
        self.slow_timer.cancel()
        print('CALLBACK_ENTER',flush=True)
        time.sleep(.25)
        assert self.context.ok(), 'context destroyed during callback'
        print('CALLBACK_END '+str(self.get_clock().now().nanoseconds),flush=True)
        if os.environ.get('INJECT_FAILURE')=='1':raise RuntimeError('deliberate_callback_failure')
    def destroy_node(self):
        print('NODE_DESTROY',flush=True)
        return super().destroy_node()
control.PlannerControl=Instrumented
control.main()
'''.format(scripts=str(source.parent))
    with tempfile.TemporaryDirectory(prefix='control-shutdown-') as directory:
        instrumented=Path(directory)/'instrumented.py';instrumented.write_text(wrapper)
        for fault in (False,True):
            name='last_callback_error' if fault else 'active_and_suspended_callbacks'
            log_path=out/(name+'.log')
            with log_path.open('w') as log:
                process=subprocess.Popen([sys.executable,str(instrumented)],stdout=log,stderr=subprocess.STDOUT,
                    env=dict(os.environ,INJECT_FAILURE='1' if fault else '0'))
                deadline=time.monotonic()+8.
                while not all(marker in log_path.read_text() for marker in ['SUSPENDED','CALLBACK_ENTER']):
                    assert process.poll() is None,'instrumented node exited early'
                    assert time.monotonic()<deadline,'callback entry timeout'
                    time.sleep(.005)
                stopped=time.monotonic();process.send_signal(signal.SIGINT)
                time.sleep(.02)
                if process.poll() is None:process.send_signal(signal.SIGTERM)
                code=process.wait(timeout=4.)
            text=log_path.read_text()
            assert text.index('CALLBACK_END')<text.index('NODE_DESTROY'),text
            assert text.index('COROUTINE_CLOSED')<text.index('NODE_DESTROY'),text
            assert not any(s in text for s in ['Destroyable','never retrieved','Exception ignored']),text
            if fault:
                assert code!=0 and 'RuntimeError: deliberate_callback_failure' in text,(code,text)
            else:assert code==0 and 'Traceback' not in text,(code,text)
            result['cases'][name]={'status':'passed','exit_code':code,'shutdown_s':time.monotonic()-stopped,
                'callback_finished_before_destroy':True,'suspended_coroutine_closed':True,
                'failure_propagated':fault}
            process=None
    result['status']='passed'
finally:
    if process is not None and process.poll() is None:
        process.kill();process.wait()
    probe.destroy_node();rclpy.shutdown()
    result['elapsed_s']=time.monotonic()-started
    (out/'native.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

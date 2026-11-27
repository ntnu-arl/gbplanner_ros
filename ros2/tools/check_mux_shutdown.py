#!/usr/bin/env python3
"""Check VelocityMux SIGINT/SIGTERM shutdown under real ROS message traffic.

Each source mux forwards a live joy command while receiving legal execution
and tracking status. Shutdown must publish zero and exit normally. The fixture
provides only idle-state/PCI-stop peers; no simulator or robot is started.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

sys.dont_write_bytecode=True


def require(condition,message):
    if not condition:raise AssertionError(message)


def main():
    root=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    output=args.output.resolve()
    logs={name:output.with_name(output.stem+'_'+name+'.log') for name in ('sigint','sigterm')}
    require(not any(p.exists() for p in [output,*logs.values()]),'Use a new result name')
    output.parent.mkdir(parents=True,exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID',str(100+os.getpid()%80))
    source=root/'src/gbplanner_smb/scripts/velocity_mux_node.py'
    started=time.monotonic();deadline=started+20.
    summary={'status':'failed','scope':__doc__,'cases':{},'domain':os.environ['ROS_DOMAIN_ID'],
        'mux_source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'fixture_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    process=probe=None;initialized=False;current=None
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from planner_msgs.srv import PciStop
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy,QoSProfile,ReliabilityPolicy
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        rclpy.init(args=[]);initialized=True
        durable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        idle={'execution':{'state':'IDLE','active':False,'accepted':0,'goal_id':None,
                           'output_in_flight':False,'continuation_pending':False},
              'tracking':{'state':'IDLE','active':False,'accepted':0,'goal_id':None,
                          'last_command':{'linear_x':0.,'angular_z':0.}},
              'control':{'state':'IDLE','auto':False}}

        class Probe(Node):
            def __init__(self,name):
                self.namespace='/mux_shutdown_'+str(os.getpid())+'_'+name
                super().__init__('probe',namespace=self.namespace)
                self.commands=[];self.states=[];self.messages=0;self.stop_calls=0
                self.joy=self.create_publisher(Twist,'joy_teleop/cmd_vel',10)
                self.layers={k:self.create_publisher(String,k+'_status',durable) for k in ('execution','tracking')}
                self.create_subscription(Twist,'cmd_vel',self.velocity,100)
                self.create_subscription(String,'velocity_mux_status',lambda m:self.states.append(json.loads(m.data)),durable)
                self.create_service(PciStop,'pci_stop',self.stop)
                for name in idle:
                    self.create_service(Trigger,name+'_state',lambda req,res,n=name:self.layer_state(res,n))
                self.query=self.create_client(Trigger,'velocity_mux_state')
                # Keep normal service deadlines. 200 Hz legal message traffic
                # exercises take_message while the real main handles signals.
                self.timer=self.create_timer(.005,self.publish)

            def publish(self):
                message=Twist();message.linear.x=.12;message.angular.z=.04
                self.joy.publish(message)
                for name,publisher in self.layers.items():publisher.publish(String(data=json.dumps(idle[name])))
                self.messages+=1

            def velocity(self,message):
                self.commands.append({'received_s':time.monotonic()-started,
                    'linear_x':message.linear.x,'angular_z':message.angular.z})

            def stop(self,_request,response):
                self.stop_calls+=1;response.success=True;return response

            def layer_state(self,response,name):
                response.success=True;response.message=json.dumps(idle[name]);return response

            def until(self,predicate,label,expect_alive=True):
                while not predicate():
                    if expect_alive:require(process.poll() is None,'Mux exited before '+label)
                    if time.monotonic()>=deadline:raise TimeoutError(label)
                    rclpy.spin_once(self,timeout_sec=.005)

            def status(self):
                self.until(self.query.service_is_ready,'Mux status discovery')
                future=self.query.call_async(Trigger.Request())
                self.until(future.done,'Mux status reply')
                response=future.result();require(response is not None and response.success,'Mux status failed')
                return json.loads(response.message)

        for name,signum in [('sigint',signal.SIGINT),('sigterm',signal.SIGTERM)]:
            current={'status':'failed','signal':name.upper(),'log_file':str(logs[name])}
            summary['cases'][name]=current
            probe=Probe(name)
            command=[sys.executable,str(source),'--ros-args','-r','__ns:='+probe.namespace]
            current['command']=command
            with logs[name].open('x') as log:
                process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,
                                         start_new_session=True,env=dict(os.environ))
            probe.until(lambda:probe.joy.get_subscription_count()>0 and
                        all(p.get_subscription_count()>0 for p in probe.layers.values()),'Input subscriptions')
            probe.until(lambda:sum(c['linear_x']==.12 and c['angular_z']==.04 for c in probe.commands)>=5 and
                        probe.stop_calls>0,'Forwarded joy traffic and PCI stop response')
            probe.until(lambda:probe.status()['stop_acknowledged'],'Mux accepted PCI stop acknowledgement')
            before=probe.status()
            require(before['forwarded']>=5 and before['selected_source']=='joy' and before['stop_acknowledged'] and
                    before['last_command']=={'linear_x':.12,'angular_z':.04},'Mux did not actually forward the live command')
            require(before['execution_goal_id'] is None and before['tracking_goal_id'] is None,'Fixture introduced a goal')
            current['before_shutdown']=before
            current['signal_sent_s']=time.monotonic()-started
            process.send_signal(signum)
            probe.until(lambda:process.poll() is not None,'Normal signal exit',expect_alive=False)
            current['exit_code']=process.returncode
            # Drain the final DDS write after the child exits; load publishers
            # remain live through shutdown and are not used to force a zero.
            drain_end=time.monotonic()+.1
            probe.until(lambda:time.monotonic()>=drain_end,'Final zero delivery',expect_alive=False)
            after=[c for c in probe.commands if c['received_s']>=current['signal_sent_s']]
            require(after and after[-1]['linear_x']==0. and after[-1]['angular_z']==0.,'Shutdown zero was not received')
            text=logs[name].read_text()
            require(process.returncode==0 and 'Traceback' not in text and 'RuntimeError' not in text,
                    'Signal shutdown did not exit cleanly')
            current.update(status='passed',shutdown_s=time.monotonic()-started-current['signal_sent_s'],
                input_messages_per_topic=probe.messages,stop_service_calls=probe.stop_calls,
                output_commands=probe.commands,post_signal_commands=after,
                final_zero_received=True,forced=False,traceback_in_log=False)
            probe.destroy_node();probe=None;process=None
        summary['status']='passed'
    except Exception as error:
        summary['error']=type(error).__name__+': '+str(error)
        summary['traceback']=traceback.format_exc()
        if current is not None and probe is not None:
            current.update(output_commands=probe.commands,last_mux_state=probe.states[-1] if probe.states else None,
                           input_messages_per_topic=probe.messages,stop_service_calls=probe.stop_calls)
    finally:
        if process is not None:
            forced=False
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=2.)
                except subprocess.TimeoutExpired:
                    forced=True;os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=1.)
            if current is not None:current.update(cleanup_exit_code=process.returncode,forced=forced)
        if probe is not None:probe.destroy_node()
        if initialized and rclpy.ok():rclpy.shutdown()
        summary['elapsed_s']=time.monotonic()-started
        output.write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'status':summary['status'],'cases':list(summary['cases']),'output':str(output),
                      'elapsed_s':summary['elapsed_s'],'error':summary.get('error')}))
    return 0 if summary['status']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())

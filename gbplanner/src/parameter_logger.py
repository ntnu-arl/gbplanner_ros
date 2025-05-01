#!/usr/bin/env python3

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

rospy.init_node("parameter_logger")
param_pub = rospy.Publisher("parameter_logs", DiagnosticStatus, queue_size=10)


log_duration = 30
log_freq = 1
rate = rospy.Rate(log_freq)
for i in range(0, log_duration*log_freq):
  parameter_log_msg = DiagnosticStatus()
  for param_name in rospy.get_param_names():
    param_entry = KeyValue()
    param_entry.key = param_name
    param_entry.value = str(rospy.get_param(param_name))
    parameter_log_msg.values.append(param_entry)
  param_pub.publish(parameter_log_msg)
  rate.sleep()
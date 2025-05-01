#!/usr/bin/env python3

import numpy as np
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

t = rospy.Duration()
# t.to_sec()

def trajCb(traj):
    tp = MultiDOFJointTrajectoryPoint
    # tp.transforms
    for i in range(1,len(traj.points)):
        p1_v = traj.points[i].transforms[0].translation
        p0_v = traj.points[i-1].transforms[0].translation
        p1 = np.array([p1_v.x, p1_v.y, p1_v.z])
        p0 = np.array([p0_v.x, p0_v.y, p0_v.z])
        seg_len = np.linalg.norm(p1 - p0)
        dt = traj.points[i].time_from_start.to_sec() - traj.points[i-1].time_from_start.to_sec()
        print("seg len: ", seg_len, " dt: ", dt, " end speed: ", seg_len/dt)
    print("------------------")


rospy.init_node('traj_listener', anonymous=True)

rospy.Subscriber("/firefly/command/trajectory", MultiDOFJointTrajectory, trajCb)

rospy.spin()
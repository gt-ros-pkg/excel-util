#! /usr/bin/python

import numpy as np
import rospy
import moveit_msgs

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cubic_spline_interp import CubicSpline
from moveit_msgs.msg import DisplayTrajectory

class SplineTraj(object):
    def __init__(self, splines):
        self.splines = splines

def from_trajectory_msg(msg):
    t, q, qd, qdd = [], [], [], []
    for point in msg.points:
        t.append(point.time_from_start.to_sec())
        q.append(point.positions)
        qd.append(point.velocities)
        qdd.append(point.accelerations)
    q, qd, qdd = np.array(q), np.array(qd), np.array(qdd)
    splines = []
    qlen = len(q[0])
    for i in range(qlen):
        splines.append(CubicSpline(np.array(t), q[:,i], qd[:,i], qdd[:,i]))
    return SplineTraj(splines)

def from_display_trajectory_msg(msg):
    trajectory = msg.trajectory[0].joint_trajectory
    return trajectory

def callback(data):
    rospy.loginfo("New trajectory")
    trajectory = from_display_trajectory_msg(data)
    traj_splines = from_trajectory_msg(trajectory)
    while True:
	snum = input("Please enter joint number:")
	try:
	    num = int(snum)	    
	    if (num<=6)&(num>=0):
		traj_splines.splines[num].view()
	        break
	    else:
		raise ValueError("Unvalid number")
	except ValueError:
	    print("This is not a valid decimal number")

def main():
    rospy.init_node("traj_printer")
    print("0 rail\n1 shoulder_pan\n2 shoulder_lift\n3 elbow\n4 wrist_1\n5 wrist_2\n6 wrist_3")
    rospy.Subscriber("move_group/display_planned_path", DisplayTrajectory, callback)
    rospy.spin()

if __name__ == "__main__":
    main()


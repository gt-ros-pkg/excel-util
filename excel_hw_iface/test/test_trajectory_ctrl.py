#!/usr/bin/python

import copy
import sys

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float64MultiArray
import rospy
import actionlib

from excel_hw_iface.excel_interface import ExcelInterface

def main():
    rospy.init_node("test_traj_ctrl")

    excel = ExcelInterface(timeout=0.)
    act_cli = excel.vel_jnt_traj_act_cli
    if not act_cli.wait_for_server(rospy.Duration.from_sec(2.)):
        rospy.logerr("Can't find action server")
        return

    q_cur = excel.get_q()
    point0 = JointTrajectoryPoint()
    point0.positions = q_cur.tolist()
    point0.velocities = 7*[0.0]
    point0.accelerations = 7*[0.0]
    points = []
    for i in range(3):
        points.append(copy.deepcopy(point0))

    if False:
        DELTA_T = 3.0
        DELTA_Q = 0.6
        JNT_IND = 0

        points[0].positions[JNT_IND] = q_cur[JNT_IND] + DELTA_Q
        points[0].time_from_start = rospy.Duration.from_sec(DELTA_T)

        points[1].positions[JNT_IND] = q_cur[JNT_IND] - DELTA_Q
        points[1].time_from_start = rospy.Duration.from_sec(3*DELTA_T)

        points[2].positions[JNT_IND] = q_cur[JNT_IND] 
        points[2].time_from_start = rospy.Duration.from_sec(4*DELTA_T)

    if True:
        DELTA_T = 1.3
        DELTA_Q = [-0.4, -0.3, -0.3, 0.4, -0.5, -0.6, -0.8]

        points[0].positions = (q_cur + DELTA_Q).tolist()
        points[0].time_from_start = rospy.Duration.from_sec(DELTA_T)

        points[1].positions = (q_cur - DELTA_Q).tolist()
        points[1].time_from_start = rospy.Duration.from_sec(3*DELTA_T)

        points[2].positions = q_cur.tolist()
        points[2].time_from_start = rospy.Duration.from_sec(4*DELTA_T)

    fjt = FollowJointTrajectoryGoal()
    fjt.trajectory.header.stamp = rospy.Time()
    fjt.trajectory.joint_names = excel.joint_names
    fjt.trajectory.points = points

    act_cli.send_goal(fjt)
    rospy.loginfo("Starting trajectory")
    rospy.sleep(1.5)
    act_cli.wait_for_result()
    rospy.loginfo("Trajectory complete")

if __name__ == "__main__":
    main()

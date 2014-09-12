#!/usr/bin/python

import copy
import sys

from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from moveit_msgs.msg import DisplayTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float64MultiArray
import rospy
import actionlib

from excel_hw_iface.excel_interface import ExcelInterface

def main():
    rospy.init_node("test_traj_sub_ctrl")
    rospy.Subscriber("move_group/display_planned_path", DisplayTrajectory, callback)
    rospy.spin()
    
def from_display_trajectory_msg(msg):
    trajectory = msg.trajectory[0].joint_trajectory
    return trajectory    

def callback(data):
    rospy.loginfo("New trajectory")
    trajectory = from_display_trajectory_msg(data)


    excel = ExcelInterface()
    act_cli = excel.vel_jnt_traj_act_cli
    if not act_cli.wait_for_server(rospy.Duration.from_sec(2.)):
        rospy.logerr("Can't find action server")
        return
    
    fjt = FollowJointTrajectoryGoal()
    fjt.trajectory = trajectory

    act_cli.send_goal(fjt)
    rospy.loginfo("Starting trajectory")
    rospy.sleep(1.5)
    act_cli.wait_for_result()
    rospy.loginfo("Trajectory complete")

if __name__ == "__main__":
    main()

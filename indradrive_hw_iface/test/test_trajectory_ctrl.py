#!/usr/bin/python

import copy
import sys

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float64MultiArray
import rospy
import actionlib

def main():
    rospy.init_node("test_traj_ctrl")
    act_cli = actionlib.SimpleActionClient(
            '/vel_trajectory_ctrl/follow_joint_trajectory', 
            FollowJointTrajectoryAction)
    if not act_cli.wait_for_server(rospy.Duration.from_sec(1.)):
        rospy.logerr("Can't find action server")
        return

    jtp1 = JointTrajectoryPoint()
    jtp1.positions = [-0.1]
    jtp1.velocities = [0.0]
    jtp1.accelerations = [0.0]
    jtp1.time_from_start = rospy.Duration.from_sec(5.)

    jtp2 = JointTrajectoryPoint()
    jtp2.positions = [0.3]
    jtp2.velocities = [0.1]
    jtp2.accelerations = [0.0]
    jtp2.time_from_start = rospy.Duration.from_sec(10.)

    jtp3 = JointTrajectoryPoint()
    jtp3.positions = [0.1]
    jtp3.velocities = [0.0]
    jtp3.accelerations = [0.0]
    jtp3.time_from_start = rospy.Duration.from_sec(15.)

    fjt = FollowJointTrajectoryGoal()
    # fjt.trajectory.header.stamp = rospy.Time.now()+rospy.Duration(5.0)
    fjt.trajectory.header.stamp = rospy.Time()
    fjt.trajectory.joint_names = ['rail_joint']
    fjt.trajectory.points = [jtp1, jtp2, jtp3]

    act_cli.send_goal(fjt)
    rospy.loginfo("Starting trajectory")
    rospy.sleep(0.0)
    act_cli.wait_for_result()
    rospy.loginfo("Trajectory complete")
    print act_cli.get_result()
    print act_cli.get_state()

if __name__ == "__main__":
    main()

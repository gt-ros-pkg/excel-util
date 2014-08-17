#!/usr/bin/python

import copy
import sys

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float64MultiArray
import rospy
import actionlib
from sensor_msgs.msg import JointState


def main():
    rospy.init_node("test_traj_ctrl")
    act_cli = actionlib.SimpleActionClient(
            '/vel_trajectory_ctrl/follow_joint_trajectory', 
            FollowJointTrajectoryAction)
    if not act_cli.wait_for_server(rospy.Duration.from_sec(1.)):
        rospy.logerr("Can't find action server")
        return
    js_msg = rospy.wait_for_message('/joint_states', JointState)
    pos_i = js_msg.position[0]

    jtp1 = JointTrajectoryPoint()
    jtp1.positions = [pos_i-0.5]
    jtp1.velocities = [0.0]
    jtp1.accelerations = [0.0]
    jtp1.time_from_start = rospy.Duration.from_sec(4.0)

    jtp2 = JointTrajectoryPoint()
    jtp2.positions = [pos_i+0.5]
    jtp2.velocities = [0.0]
    jtp2.accelerations = [0.0]
    jtp2.time_from_start = rospy.Duration.from_sec(12.)

    jtp3 = JointTrajectoryPoint()
    jtp3.positions = [pos_i]
    jtp3.velocities = [0.0]
    jtp3.accelerations = [0.0]
    jtp3.time_from_start = rospy.Duration.from_sec(16.)

    fjt = FollowJointTrajectoryGoal()
    # fjt.trajectory.header.stamp = rospy.Time.now()+rospy.Duration(5.0)
    fjt.trajectory.header.stamp = rospy.Time()
    fjt.trajectory.joint_names = ['rail_joint']
    fjt.trajectory.header.stamp = rospy.Time()
    fjt.trajectory.points = [jtp1, jtp2, jtp3]

    while not rospy.is_shutdown():
        act_cli.send_goal(fjt)
        rospy.loginfo("Starting trajectory")
        rospy.sleep(0.0)
        act_cli.wait_for_result()
        rospy.loginfo("Trajectory complete")
        print act_cli.get_result()
        print act_cli.get_state()
        rospy.sleep(rospy.Duration(1.))

if __name__ == "__main__":
    main()

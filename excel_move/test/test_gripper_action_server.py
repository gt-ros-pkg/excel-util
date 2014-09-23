#! /usr/bin/env python

import roslib
import rospy
import actionlib
import sys

from control_msgs.msg import GripperCommandAction, GripperCommandGoal

def main(is_close):
    rospy.init_node("test_gripper_action_server")
    act_cli = actionlib.SimpleActionClient('gripper_controller/gripper_action', GripperCommandAction)
    act_cli.wait_for_server()
    if is_close:
        close_goal = GripperCommandGoal()
        close_goal.command.position = 0.0
        close_goal.command.max_effort = 1000.0
        act_cli.send_goal_and_wait(close_goal)
        rospy.loginfo("Gripper closed.")
    else:
        open_goal = GripperCommandGoal()
        open_goal.command.position = 1.0
        open_goal.command.max_effort = 1000.0
        act_cli.send_goal_and_wait(open_goal)
        rospy.loginfo("Gripper open.")

if __name__ == '__main__':
    if len(sys.argv) > 1:
        is_close = sys.argv[1] == "close"
    main(is_close)

#! /usr/bin/python

import sys

import actionlib
import rospy
from excel_bins.msg import MoveBinAction, MoveBinGoal


def main():
    rospy.init_node('bin_manager')

    act_goal = MoveBinGoal()
    act_goal.bin_id = int(sys.argv[1])
    act_goal.x_target = float(sys.argv[2])
    act_goal.y_target = float(sys.argv[3])
    act_goal.r_target = float(sys.argv[4])
    act_goal.size = "large"

    print "Waiting for action server 'move_bin_to_target' ..."
    bin_move_ac = actionlib.SimpleActionClient('move_bin_to_target', MoveBinAction)
    bin_move_ac.wait_for_server()
    print "Found action server."
    print "Sending action goal:", act_goal
    outcome = bin_move_ac.send_goal_and_wait(act_goal)
    return outcome == actionlib.GoalStatus.SUCCEEDED

if __name__ == "__main__":
    main()

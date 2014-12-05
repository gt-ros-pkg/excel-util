#! /usr/bin/python

import sys

import actionlib
import rospy
from excel_move.msg import ScanningAction, ScanningGoal


def main():
    rospy.init_node('test_scanning_as')

    act_goal = ScanningGoal()
    # act_goal.good_bins = ["333", "555", "999"]
    # act_goal.bad_bins = ["111", "777"]
    act_goal.good_bins = ["111", "555"]
    act_goal.bad_bins = ["333", "777", "999"]
   # act_goal.good_bins = ["333", "555", "999"]
   # act_goal.bad_bins = ["111", "777"]

    print "Waiting for action server 'scan_parts' ..."
    scanning_ac = actionlib.SimpleActionClient('scan_parts', ScanningAction)
    scanning_ac.wait_for_server()
    print "Found action server."
    print "Sending action goal:", act_goal
    outcome = scanning_ac.send_goal_and_wait(act_goal)
    result = scanning_ac.get_result()
    print "Outcome:", outcome
    print "Result:", result

if __name__ == "__main__":
    main()

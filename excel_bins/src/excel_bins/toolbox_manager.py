#! /usr/bin/python

import sys
import yaml
import numpy as np

import actionlib
import rospy
from roslaunch.substitution_args import resolve_args
from ar_tag_manager import ARTagManager
from excel_bins.msg import MoveBinAction, MoveBinGoal
from excel_move.msg import ScanningAction, ScanningGoal
from std_msgs.msg import Int32, String, Bool, Int8, Int8MultiArray

from ur_py_utils.ur_controller_manager import URControllerManager

def load_ws_setup(filename):
    f = file(resolve_args(filename), 'r')
    yaml_data = yaml.load(f)
    slots = yaml_data['slots']
    bin_home_slots = yaml_data['bin_home_slots']
    hum_ws_slots = yaml_data['hum_ws_slots']
    bin_barcode_ids = yaml_data['bin_barcode_ids']
    tags_list = yaml_data['tags_list']
    f.close()
    return slots, tags_list, bin_home_slots, hum_ws_slots, bin_barcode_ids

class ToolboxDeliverer(object):
    def __init__(self, ar_man):
        self.ar_man = ar_man
        self.occupied_sub = rospy.Subscriber('/human/workspace/occupied', Bool, self.occupied_cb)
        self.is_occupied = False 

        print "Waiting for action server 'move_bin_to_target' ..."
        self.bin_move_ac = actionlib.SimpleActionClient('move_bin_to_target', MoveBinAction)
        self.bin_move_ac.wait_for_server()
        print "Found action server."

    def occupied_cb(self, msg):
        self.is_occupied = msg.data

    def prepare_toolbox(self):
        display_message_pub = rospy.Publisher('/display/message', String, latch=True)
        scan_status_pub = rospy.Publisher('/display/scanning_status', Int8, latch=True)
        bins_required_pub = rospy.Publisher('/display/bins_required', Int8MultiArray, latch=True) # must be 2 or 3
        parts_ready_pub = rospy.Publisher('/display/parts_ready', Bool, latch=True)
        system_blocked_pub = rospy.Publisher('/display/system_blocked', Bool, latch=True)

        display_message_pub.publish("Toolbox demo starting")
        scan_status_pub.publish(0)
        print "Taking the robot to the first pose"
        parts_ready_pub.publish(True)
        first_toolbox_goal = MoveBinGoal()
        first_toolbox_goal.bin_id = -10
        first_toolbox_goal.x_target = 1.5
        first_toolbox_goal.y_target = 1.6
        first_toolbox_goal.r_target = -90
        display_message_pub.publish("Taking the toolbox to the first pose")

        self.ar_man.not_track_cb(Int32(100))
        outcome = self.bin_move_ac.send_goal_and_wait(first_toolbox_goal)
        self.ar_man.not_track_cb(Int32(-100))
        return outcome == actionlib.GoalStatus.SUCCEEDED

    def remove_toolbox(self):
        display_message_pub = rospy.Publisher('/display/message', String, latch=True)
        scan_status_pub = rospy.Publisher('/display/scanning_status', Int8, latch=True)
        bins_required_pub = rospy.Publisher('/display/bins_required', Int8MultiArray, latch=True) # must be 2 or 3
        parts_ready_pub = rospy.Publisher('/display/parts_ready', Bool, latch=True)
        system_blocked_pub = rospy.Publisher('/display/system_blocked', Bool, latch=True)

        display_message_pub.publish("Putting the toolbox back on the table")
        scan_status_pub.publish(0)
        print "Taking the robot to the first pose"
        parts_ready_pub.publish(True)
        first_toolbox_goal = MoveBinGoal()
        first_toolbox_goal.bin_id = -11
        first_toolbox_goal.x_target = 0.5
        first_toolbox_goal.y_target = 0.2
        first_toolbox_goal.r_target = 0.0

        self.ar_man.not_track_cb(Int32(100))
        outcome = self.bin_move_ac.send_goal_and_wait(first_toolbox_goal)
        self.ar_man.not_track_cb(Int32(-100))

        return outcome == actionlib.GoalStatus.SUCCEEDED

def main():
    rospy.init_node('toolbox_manager')
    ws_setup_fn = '$(find excel_bins)/src/excel_bins/bin_workspace_setup.yaml'
    slots, tags_list, bin_home_slots, hum_ws_slots, bin_barcode_ids = load_ws_setup(ws_setup_fn)
    print "Slots:", slots
    print "Bin homes:", bin_home_slots
    print "Human workspace slots:", hum_ws_slots
    print "Bin barcode IDs:", bin_barcode_ids

    IS_SIMULATION = False
    
    ar_man = ARTagManager(slots, tags_list)

    cman = URControllerManager()

    cman.start_joint_controller('vel_pva_trajectory_ctrl')
    rospy.sleep(1.)
    print "Slot states:", ar_man.get_real_bin_slot_states()

    toolbox_deliverer = ToolboxDeliverer(ar_man)
    toolbox_deliverer.prepare_toolbox()

    rospy.sleep(0.1)
    cman.start_joint_controller('vel_cart_pos_ctrl')
    raw_input("Press enter to remove toolbox")
    cman.start_joint_controller('vel_pva_trajectory_ctrl')
    rospy.sleep(0.1)

    toolbox_deliverer.remove_toolbox()

if __name__ == "__main__":
    main()

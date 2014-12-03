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

class BinManager(object):
    def __init__(self, ar_man, hum_ws_slots, bin_home_slots={}, is_sim=False):
        self.ar_man = ar_man
        self.hum_ws_slots = hum_ws_slots
        self.bin_home_slots = bin_home_slots
        self.is_sim = is_sim

        # self.bin_not_tracking_pub = rospy.Publisher('/bin_not_tracking', Int32)
        if not self.is_sim:
            print "Waiting for action server 'move_bin_to_target' ..."
            self.bin_move_ac = actionlib.SimpleActionClient('move_bin_to_target', MoveBinAction)
            self.bin_move_ac.wait_for_server()
            print "Found action server."

            print "\n"*3
            print "        RUNNING IN FOR REAL"
            print "\n"*3
        else:
            print "\n"*3
            print "        RUNNING IN SIMULATION"
            print "\n"*3

    # bin_order is a list of bin ids needed for this order
    def deliver_bin_order(self, bin_order):
        print "Delivering bin order:", bin_order
        while not rospy.is_shutdown():
            hum_ws_start_bins = self.ar_man.get_real_filled_slots(self.hum_ws_slots)
            all_bins = self.ar_man.get_real_filled_slots()
            bins_to_deliver = []
            bins_to_remove = []
            for order_bin_id in bin_order:
                r = rospy.Rate(1)
                while order_bin_id not in all_bins and not rospy.is_shutdown():
                    print "Bin", order_bin_id, "is not currently in the workspace, will continue once it is"
                    r.sleep()
                if order_bin_id not in hum_ws_start_bins:
                    # bin not in the workspace, we need to deliver
                    bins_to_deliver.append(order_bin_id)
            for ws_bin_id in hum_ws_start_bins:
                if ws_bin_id not in bin_order:
                    # bin in workspace but not in order, need to remove
                    bins_to_remove.append(ws_bin_id)

            if len(bins_to_deliver) == 0 and len(bins_to_remove) == 0:
                print "Completed bin order:", bin_order
                return

            print "Delivering bins:", bins_to_deliver
            print "Removing bins:", bins_to_remove

            while (len(bins_to_deliver) > 0 or len(bins_to_remove) > 0) and not rospy.is_shutdown():

                if len(bins_to_remove) > 0:
                    bin_to_remove = bins_to_remove[0]
                    remove_to_slot = self.get_remove_to_slot(bin_to_remove)

                    print "Removing bin", bin_to_remove, "to slot", remove_to_slot
                    if not self.move_bin_to_slot(bin_to_remove, remove_to_slot):
                        rospy.logerr("Failed moving bin to slot, press enter to continue")
                        sys.stdin.readline()

                    # confirm that the bin is no longer in the human workspace
                    if bin_to_remove not in self.ar_man.get_real_filled_slots(self.hum_ws_slots):
                        bins_to_remove.remove(bin_to_remove)
                    else:
                        print "Bin", bin_to_remove, "still in workspace, this shouldn't happen"

                if len(bins_to_deliver) > 0:
                    #empty_hum_ws_slots = self.ar_man.get_real_empty_slots(self.hum_ws_slots)
                    empty_hum_ws_slots = self.ar_man.empty_slots(self.hum_ws_slots)
                    bin_to_deliver = bins_to_deliver[0]
                    empty_hum_ws_slot = empty_hum_ws_slots[0]

                    print "Delivering bin", bin_to_deliver, "to slot", empty_hum_ws_slot
                    if not self.move_bin_to_slot(bin_to_deliver, empty_hum_ws_slot):
                        rospy.logerr("Failed moving bin to slot, press enter to continue")
                        sys.stdin.readline()

                    # confirm that the bin is in the human workspace
                    if bin_to_deliver in self.ar_man.get_real_filled_slots(self.hum_ws_slots):
                        bins_to_deliver.remove(bin_to_deliver)
                    else:
                        print "Bin", bin_to_deliver, "not in workspace, this shouldn't happen"

    def move_bin_to_slot(self, bin_id, slot_id):
        r = rospy.Rate(1)
        while bin_id not in self.ar_man.get_real_filled_slots() and not rospy.is_shutdown():
            print "Bin", bin_id, "is not currently in the workspace, will continue once it is"
            r.sleep()

        slot_pose = self.ar_man.bin_slots[slot_id]
        act_goal = MoveBinGoal()
        act_goal.bin_id = bin_id
        act_goal.x_target = slot_pose[0][0]
        act_goal.y_target = slot_pose[0][1]
        act_goal.r_target = np.rad2deg(slot_pose[1][2])
        act_goal.size = "large"
        print "Sending action goal:", act_goal
        if not self.is_sim:
            # self.bin_not_tracking_pub.publish(bin_id)
            self.ar_man.not_track_cb(Int32(bin_id))
            outcome = self.bin_move_ac.send_goal_and_wait(act_goal)
            self.ar_man.not_track_cb(Int32(-bin_id))
            # self.ar_man.set_double_bin_location(bin_id, slot_pose)
            self.ar_man.set_real_bin_location(bin_id, slot_pose)
            # self.bin_not_tracking_pub.publish(-bin_id)
            return outcome == actionlib.GoalStatus.SUCCEEDED
        else:
            print "Manually move bin", bin_id, "to slot", slot_id, "and hit enter when done"
            sys.stdin.readline()
            return True

    def get_remove_to_slot(self, bin_to_remove):
        remove_to_slot = -100
        if bin_to_remove in self.bin_home_slots:
            remove_to_slot = self.bin_home_slots[bin_to_remove]
        #empty_rob_ws_slots = self.ar_man.get_real_empty_slots(self.hum_ws_slots, invert_set=True)
        empty_rob_ws_slots = self.ar_man.empty_slots(self.hum_ws_slots, invert_set=True)
        print "Current empty robot workspace slots:", empty_rob_ws_slots
        if remove_to_slot not in empty_rob_ws_slots:
            print "Slot", remove_to_slot, "not an empty slot, picking a random empty slot"
            remove_to_slot = empty_rob_ws_slots[0]
        return remove_to_slot

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

def load_bin_orders(filename):
    f = file(resolve_args(filename), 'r')
    yaml_data = yaml.load(f)
    bin_orders = yaml_data['bin_orders']
    f.close()
    return bin_orders

class ToolboxDeliverer(object):
    def __init__(self):
        self.occupied_sub = rospy.Subscriber('/human/workspace/occupied', Bool, self.occupied_cb)
        self.is_occupied = False 

    def occupied_cb(self, msg):
        self.is_occupied = msg.data

    def prepare_toolbox(self, bin_man, bin_orders):
        display_message_pub = rospy.Publisher('/display/message', String, latch=True)
        scan_status_pub = rospy.Publisher('/display/scanning_status', Int8, latch=True)
        bins_required_pub = rospy.Publisher('/display/bins_required', Int8MultiArray, latch=True) # must be 2 or 3
        parts_ready_pub = rospy.Publisher('/display/parts_ready', Bool, latch=True)
        system_blocked_pub = rospy.Publisher('/display/system_blocked', Bool, latch=True)

        display_message_pub.publish("Toolbox demo starting")
        scan_status_pub.publish(0)
        while not rospy.is_shutdown():
            print "Taking the robot to the first pose"
            parts_ready_pub.publish(True)
            first_toolbox_goal = MoveBinGoal()
            first_toolbox_goal.bin_id = -10
            first_toolbox_goal.x_target = 1.5
            first_toolbox_goal.y_target = 1.6
            first_toolbox_goal.r_target = -90
            display_message_pub.publish("Taking the toolbox to the first pose")

	    bin_man.ar_man.not_track_cb(Int32(100))
            outcome = bin_man.bin_move_ac.send_goal_and_wait(first_toolbox_goal)
	    bin_man.ar_man.not_track_cb(Int32(-100))
	    return outcome == actionlib.GoalStatus.SUCCEEDED

    def remove_toolbox(self, bin_man, bin_orders):
        display_message_pub = rospy.Publisher('/display/message', String, latch=True)
        scan_status_pub = rospy.Publisher('/display/scanning_status', Int8, latch=True)
        bins_required_pub = rospy.Publisher('/display/bins_required', Int8MultiArray, latch=True) # must be 2 or 3
        parts_ready_pub = rospy.Publisher('/display/parts_ready', Bool, latch=True)
        system_blocked_pub = rospy.Publisher('/display/system_blocked', Bool, latch=True)

        display_message_pub.publish("Putting the toolbox back on the table")
        scan_status_pub.publish(0)
        while not rospy.is_shutdown():
            print "Taking the robot to the first pose"
            parts_ready_pub.publish(True)
            first_toolbox_goal = MoveBinGoal()
            first_toolbox_goal.bin_id = -11
            first_toolbox_goal.x_target = 0.5
            first_toolbox_goal.y_target = 0.2
            first_toolbox_goal.r_target = 0.0

	    bin_man.ar_man.not_track_cb(Int32(100))
            outcome = bin_man.bin_move_ac.send_goal_and_wait(first_toolbox_goal)
            bin_man.ar_man.not_track_cb(Int32(-100))

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
    bin_man = BinManager(ar_man, hum_ws_slots, bin_home_slots, IS_SIMULATION)

    rospy.sleep(1.)
    print "Slot states:", ar_man.get_real_bin_slot_states()

    bin_orders = load_bin_orders('$(find excel_bins)/src/excel_bins/bin_orders1.yaml')
    print "Bin orders:", bin_orders
    toolbox_deliverer = ToolboxDeliverer()
    toolbox_deliverer.prepare_toolbox(bin_man, bin_orders)

    rospy.sleep(1.)
    toolbox_deliverer.remove_toolbox(bin_man, bin_orders)

if __name__ == "__main__":
    main()

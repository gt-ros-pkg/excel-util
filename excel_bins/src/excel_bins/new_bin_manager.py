#! /usr/bin/python

import sys
import yaml
import numpy as np

import actionlib
import rospy
from roslaunch.substitution_args import resolve_args
from excel_bins.msg import MoveBinAction, MoveBinGoal
from excel_move.msg import ScanningAction, ScanningGoal
from std_msgs.msg import Int32, String, Bool, Int8, Int8MultiArray

from excel_bins.bin_perception import BinPerception, SlotManager

from ur_py_utils.ur_controller_manager import URControllerManager

class BinManager(object):
    def __init__(self, bin_percep, slot_man, is_sim=False):
        self.bin_percep = bin_percep
        self.slot_man = slot_man
        self.is_sim = is_sim

        self.not_moving_dist = 0.01

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

    def wait_for_bins_in_scene(self, bin_ids):
        bins_in_scene = self.bin_percep.list_bins_in_scene()
        bins_not_in_scene = []
        for bin_id in bin_ids:
            if bin_id not in bins_in_scene:
                bins_not_in_scene.append(bin_id)
        if len(bins_not_in_scene) == 0:
            return

        last_bin_xy = [np.array([-1000,0.]) for i in range(len(bin_ids))]
        for i, bin_id in enumerate(bin_ids):
            cur_bin_pos, cur_bin_quat = self.bin_percep.get_bin_pose(bin_id)
            if cur_bin_pos is not None:
                last_bin_xy[i] = cur_bin_pos[:2]
        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            bins_moving = []
            bins_not_in_scene = []
            for i, bin_id in enumerate(bin_ids):
                cur_bin_pos, cur_bin_quat = self.bin_percep.get_bin_pose(bin_id)
                if cur_bin_pos is not None:
                    dist_moved = np.linalg.norm(cur_bin_pos[:2] - last_bin_xy[i])
                    print bin_id, "Moved dist", dist_moved
                    if dist_moved > self.not_moving_dist:
                        bins_moving.append(bin_id)
                    last_bin_xy[i] = cur_bin_pos[:2]
                else:
                    bins_not_in_scene.append(bin_id)
            if len(bins_moving) == 0 and len(bins_not_in_scene) == 0:
                return

            print "Bins", bins_not_in_scene, "are not currently in the workspace"
            print "Bins", bins_moving, "are still moving"
            print "Will continue once bins are present and not moving"
            r.sleep()

    # assumes this bin_order is in the scene
    def get_bins_to_deliver(self, bin_order):
        hum_ws_bins = self.bin_percep.list_bins_in_workspace()
        bins_to_deliver = []
        for order_bin_id in bin_order:
            if order_bin_id not in hum_ws_bins:
                # bin not in the workspace, we need to deliver
                bins_to_deliver.append(order_bin_id)
        return bins_to_deliver

    def is_bin_order_complete(self, bin_order):
        bins_to_remove = self.get_bins_to_remove(bin_order)
        bins_to_deliver = self.get_bins_to_deliver(bin_order)
        return len(bins_to_remove) == 0 and len(bins_to_deliver) == 0

    # assumes this bin_order is in the scene
    def get_bins_to_remove(self, bin_order):
        hum_ws_bins = self.bin_percep.list_bins_in_workspace()
        bins_to_remove = []
        for ws_bin_id in hum_ws_bins:
            if ws_bin_id not in bin_order:
                # bin in workspace but not in order, need to remove
                bins_to_remove.append(ws_bin_id)
        return bins_to_remove

    # bin_order is a list of bin ids needed for this order
    def deliver_bin_order(self, bin_order):
        print "Delivering bin order:", bin_order
        self.wait_for_bins_in_scene(bin_order)
        while not self.is_bin_order_complete(bin_order) and not rospy.is_shutdown():
            bins_to_remove = self.get_bins_to_remove(bin_order)
            fail_deliver = False
            fail_remove = False
            if len(bins_to_remove) > 0:
                bin_to_remove = bins_to_remove[0]
                is_still_holding_bin = False

                while not rospy.is_shutdown():
                    # NOTE update bins now since we are about to look for an empty slot and immediately
                    # plan movement to that slot. This call should not be repeated until after
                    # the move_bin_to_slot method is called, since that call depends on updating
                    # the scene during its call and that the space it selects will still be empty
                    # at the end of its move NOTE
                    self.bin_percep.planning_scene_update_bins()

                    # look for an empty slot to remove to
                    remove_to_slot = self.slot_man.get_remove_to_slot(bin_to_remove)

                    if remove_to_slot is not None:
                        print "Removing bin", bin_to_remove, "to slot", remove_to_slot
                        success, is_still_holding_bin = self.move_bin_to_slot(bin_to_remove, remove_to_slot, is_still_holding_bin)
                        if not success:
                            if is_still_holding_bin:
                                rospy.logerr("Failed moving bin to slot but still holding bin, will try to replace")
                                rospy.sleep(1.)
                                continue
                            else:
                                rospy.logerr("Failed moving bin to slot but not holding bin, will just skip")
                                rospy.sleep(1.)
                                break
                        else:
                            rospy.loginfo("Success moving bin")
                            break
                    else:
                        print "\nNo empty robot ws slots found\n"
                        fail_remove = True
                        break

            bins_to_deliver = self.get_bins_to_deliver(bin_order)
            if len(bins_to_deliver) > 0:
                bin_to_deliver = bins_to_deliver[0]
                is_still_holding_bin = False

                while not rospy.is_shutdown():
                    # NOTE update bins now since we are about to look for an empty slot and immediately
                    # plan movement to that slot. This call should not be repeated until after
                    # the move_bin_to_slot method is called, since that call depends on updating
                    # the scene during its call and that the space it selects will still be empty
                    # at the end of its move NOTE
                    self.bin_percep.planning_scene_update_bins()

                    # look for an empty slot to deliver to
                    deliver_to_slot = self.slot_man.get_deliver_to_slot(bin_to_deliver)

                    if deliver_to_slot is not None:
                        print "Delivering bin", bin_to_deliver, "to slot", deliver_to_slot
                        success, is_still_holding_bin = self.move_bin_to_slot(bin_to_deliver, deliver_to_slot, is_still_holding_bin)
                        if not success:
                            if is_still_holding_bin:
                                rospy.logerr("Failed moving bin to slot but still holding bin, will try to replace")
                                rospy.sleep(1.)
                                continue
                            else:
                                rospy.logerr("Failed moving bin to slot but not holding bin, will just skip")
                                rospy.sleep(1.)
                                break
                        else:
                            rospy.loginfo("Success moving bin")
                            break
                    else:
                        print "\nNo empty human ws slots found\n"
                        fail_deliver = True
                        break

            if fail_deliver and fail_remove:
                print "\nIs collision checker running?\n"
                rospy.sleep(1.)

    def move_bin_to_slot(self, bin_id, slot_id, is_holding_bin):

        slot_pose = self.slot_man.get_slot_pose(slot_id)
        act_goal = MoveBinGoal()
        act_goal.bin_id = bin_id
        act_goal.x_target = slot_pose[0][0]
        act_goal.y_target = slot_pose[0][1]
        act_goal.r_target = np.rad2deg(slot_pose[1][2])
        act_goal.is_holding_bin = is_holding_bin
        act_goal.size = "large"
        print "Sending action goal:", act_goal
        if not self.is_sim:
            self.bin_percep.pause_bin_tracking(bin_id, pause=True)
            outcome = self.bin_move_ac.send_goal_and_wait(act_goal)
            result = self.bin_move_ac.get_result()
            if result.success:
                self.bin_percep.force_bin_pose(bin_id, slot_pose)
                self.bin_percep.pause_bin_tracking(bin_id, pause=False)
            return result.success, result.is_still_holding_bin
            # return outcome == actionlib.GoalStatus.SUCCEEDED
        else:
            print "Manually move bin", bin_id, "to slot", slot_id, "and hit enter when done"
            sys.stdin.readline()
            return True, False

def load_ws_setup(filename):
    f = file(resolve_args(filename), 'r')
    yaml_data = yaml.load(f)
    slots = yaml_data['slots']
    bin_home_slots = yaml_data['bin_home_slots']
    hum_ws_slots = yaml_data['hum_ws_slots']
    bin_barcode_ids = yaml_data['bin_barcode_ids']
    tags_list = yaml_data['tags_list']
    bin_tag_pairs = yaml_data['bin_tag_pairs']
    f.close()
    return slots, tags_list, bin_home_slots, hum_ws_slots, bin_barcode_ids, bin_tag_pairs

def load_bin_orders(filename):
    f = file(resolve_args(filename), 'r')
    yaml_data = yaml.load(f)
    bin_orders = yaml_data['bin_orders']
    f.close()
    return bin_orders

class BinDelivererScanner(object):
    def __init__(self, bin_man, is_sim=False):
        self.bin_man = bin_man
        self.is_sim = is_sim

        self.occupied_sub = rospy.Subscriber('/human/workspace/occupied', Bool, self.occupied_cb)
        self.is_occupied = False 
        self.scanning_ended = False
        self.display_message_pub = rospy.Publisher('/display/message', String, latch=True)
        self.scan_status_pub = rospy.Publisher('/display/scanning_status', Int8, latch=True)
        self.bins_required_pub = rospy.Publisher('/display/bins_required', Int8MultiArray, latch=True) # must be 2 or 3
        self.parts_ready_pub = rospy.Publisher('/display/parts_ready', Bool, latch=True)
        self.system_blocked_pub = rospy.Publisher('/display/system_blocked', Bool, latch=True)
        self.scanned_parts_pub = rospy.Publisher('/display/scanned_parts', Int8, latch=True)

        if not is_sim:
            self.scanning_ac = actionlib.SimpleActionClient('scan_parts', ScanningAction)
            print "Waiting for action server 'scan_parts' ..."
            self.scanning_ac.wait_for_server()
            print "Found action server."

    def occupied_cb(self, msg):
        self.is_occupied = msg.data
        if (self.scanning_ended and not self.is_occupied):
            self.scanned_parts_pub.publish(0)
            self.scan_status_pub.publish(0)
            self.scanning_ended = False

    def deliver_bin_orders(self, bin_orders, bin_barcode_ids):
        if not self.is_sim:
            cman = URControllerManager()
            cman.start_joint_controller('vel_pva_trajectory_ctrl')

        self.display_message_pub.publish("Demo starting")
        self.scan_status_pub.publish(0)

        if not self.is_sim:
            # move robot to home position
            home_pose_act_goal = MoveBinGoal()
            home_pose_act_goal.bin_id = -12
            self.bin_man.bin_move_ac.send_goal_and_wait(home_pose_act_goal)

        while not rospy.is_shutdown():
            for bin_order in bin_orders:
                self.display_message_pub.publish("Fetching Part Numbers: {" + 
                                            ", ".join([str(b) for b in bin_order]) + "}...")
                self.bins_required_pub.publish(data=bin_order)
                self.parts_ready_pub.publish(False)

                if False:
                    rospy.sleep(5.0)
                else:
                    self.bin_man.deliver_bin_order(bin_order)

                print "Moving to home position"
                self.parts_ready_pub.publish(True)

                home_act_goal = MoveBinGoal()
                wait_time = 4.0
                if self.is_occupied:
                    home_act_goal.bin_id = -6
                    self.display_message_pub.publish("Workspace occupied, Moving to home position bis")
                    wait_time = 0.5
                else:
                    home_act_goal.bin_id = -5
                    self.display_message_pub.publish("Moving to home position")

                if not self.is_sim:
                    self.bin_man.bin_move_ac.send_goal_and_wait(home_act_goal)

                self.display_message_pub.publish("Waiting for associate")
                self.scan_status_pub.publish(-1)

                if True: 
                    r = rospy.Rate(0.5)
                    while not rospy.is_shutdown():
                        print "Waiting for human to occupy workspace"
                        if self.is_occupied:
                            break
                        r.sleep()
                    rospy.sleep(wait_time)
                    
                    print "Starting first scan"


                    self.scanned_parts_pub.publish(0)
                    self.display_message_pub.publish("Starting first scan")
                    act_goal = ScanningGoal()
                    act_goal.good_bins = [bin_barcode_ids[bid] for bid in bin_order]
                    for bc_id in bin_barcode_ids.values():
                        if bc_id not in act_goal.good_bins:
                            act_goal.bad_bins.append(bc_id)
                    print "Scanning goal:", act_goal
 
                    if not self.is_sim:
                        outcome = self.scanning_ac.send_goal_and_wait(act_goal)
                        scanning_result = self.scanning_ac.get_result().result
                        scanned_bins = self.scanning_ac.get_result().scanned
                    else:
                        scanning_result = 0
                    
                    if scanning_result == 0:
                        print "Scanning successful, going to next bin order"
                        self.scan_status_pub.publish(1)
                        self.scanning_ended = True
                        continue
                    elif scanning_result == -1:

                        print "1st scanning_result", scanning_result

                        #good elements set
                        second_list=[]
                        for bi in range(len(scanned_bins)):
                            if scanned_bins[bi]==0:
                                second_list.append(act_goal.good_bins[bi])
                                
                        act_goal.good_bins = second_list

                        rospy.sleep(0.1)
                        print "Starting second scan"

                        self.display_message_pub.publish("Starting second scan")
                        outcome = self.scanning_ac.send_goal_and_wait(act_goal)
                        scanning_result = self.scanning_ac.get_result().result
                        scanned_bins = self.scanning_ac.get_result().scanned

                        print "2nd scanning_result", scanning_result

                        if scanning_result == 0:
                            self.scan_status_pub.publish(1)
                            print "Scanning successful, going to next bin order"
                            self.scanning_ended = True
                            continue
                        elif scanning_result == -1:

                            #good elements set
                            third_list=[]
                            for bi in range(len(scanned_bins)):
                                if scanned_bins[bi]==0:
                                    third_list.append(act_goal.good_bins[bi])
                                
                            act_goal.good_bins = third_list

                            print "Waiting for human to exit"
                            self.system_blocked_pub.publish(True)
                            self.display_message_pub.publish("Finish assembly and get out of the workspace")
                            r = rospy.Rate(1)
                            while not rospy.is_shutdown():
                                print "Waiting for human to get out of workspace"
                                if not self.is_occupied:
                                    break
                                r.sleep()
                            self.system_blocked_pub.publish(False)

                            self.display_message_pub.publish("Starting third scan")
                            outcome = self.scanning_ac.send_goal_and_wait(act_goal)
                            scanning_result = self.scanning_ac.get_result().result
                            print "3rd scanning_result", scanning_result

                            if scanning_result == 0:
                                self.scan_status_pub.publish(1)
                                print "Scanning successful, going to next bin order"
                                self.scanning_ended = True
                                continue
                            elif scanning_result == -1:
                                self.scan_status_pub.publish(0)
                                self.display_message_pub.publish("MISSING PART")
                                print "Still missing parts, exiting"
                                return
                            elif scanning_result == -2:
                                self.scan_status_pub.publish(0)
                                self.display_message_pub.publish("BAD PART")
                                print "\n"* 5
                                print "              WRONG PART"
                                print "\n"* 5
                                return
                            
                        elif scanning_result == -2:
                            self.scan_status_pub.publish(0)
                            self.display_message_pub.publish("BAD PART")
                            print "\n"* 5
                            print "              WRONG PART"
                            print "\n"* 5
                            return

                    elif scanning_result == -2:
                        self.scan_status_pub.publish(0)
                        self.display_message_pub.publish("BAD PART")
                        print "\n"* 5
                        print "              WRONG PART"
                        print "\n"* 5
                        return

                elif True:
                    rospy.sleep(3.0)
                else:
                    print "Order complete, (scanning now), press enter to continue"
                    sys.stdin.readline()
                
            print "Starting at the beginning"

def main():
    rospy.init_node('bin_manager')
    ws_setup_fn = '$(find excel_bins)/src/excel_bins/new_bin_workspace_setup.yaml'
    slots, tags_list, bin_home_slots, hum_ws_slots, bin_barcode_ids, bin_tag_pairs = load_ws_setup(ws_setup_fn)
    print "Slots:", slots
    print "Bin homes:", bin_home_slots
    print "Human workspace slots:", hum_ws_slots
    print "Bin barcode IDs:", bin_barcode_ids
    print "Bin tag pairs:", bin_tag_pairs

    IS_SIMULATION = False
    
    slot_man = SlotManager(slots, hum_ws_slots, bin_home_slots)
    bin_percep = BinPerception(bin_tag_pairs)

    bin_man = BinManager(bin_percep, slot_man, IS_SIMULATION)

    rospy.sleep(2.0)
    bin_percep.clear_all_bins_from_scene()
    bin_percep.planning_scene_update_bins()

    bin_orders = load_bin_orders('$(find excel_bins)/src/excel_bins/bin_orders1.yaml')
    print "Bin orders:", bin_orders
    bin_deliverer = BinDelivererScanner(bin_man, IS_SIMULATION)
    bin_deliverer.deliver_bin_orders(bin_orders, bin_barcode_ids)

if __name__ == "__main__":
    main()

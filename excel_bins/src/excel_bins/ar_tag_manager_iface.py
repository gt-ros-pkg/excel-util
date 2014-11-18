#! /usr/bin/python

import numpy as np
import yaml
import math
from scipy.spatial import KDTree
from threading import RLock
from collections import deque
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from roslaunch.substitution_args import resolve_args
from excel_servers.srv import BinLocationEmpty

import rospy
from hrl_geom.pose_converter import PoseConv
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from moveit_msgs.msg import CollisionObject

def create_slot_tree(bin_slots):
    pos_data = np.zeros((len(bin_slots),3))
    for i, bid in enumerate(sorted(bin_slots.keys())):
        pos_data[i,:] = bin_slots[bid][0]
    return KDTree(pos_data)

def load_bin_slots(filename):
    f = file(resolve_args(filename), 'r')
    bin_slots = yaml.load(f)['data']
    f.close()
    return bin_slots

class ARTagManagerInterface(object):
    def __init__(self, bin_slots, available_bins=None):
        # distance from marker to slot which can be considered unified
        self.ar_unification_thresh = 0.3
        self.bin_slots = bin_slots
        self.available_bins = available_bins
        if len(bin_slots) > 0:
            self.slot_tree = create_slot_tree(bin_slots)
        else:
            self.slot_tree = None
        self.camera_pose = np.mat(np.eye(4))
        self.ar_poses = {}
        self.lock = RLock()
        self.slots_pub = rospy.Publisher("/bin_slots", PoseArray, latch=True)
        self.publish_poses(self.slots_pub, [bin_slots[slot] for slot in bin_slots])
        
        self.real_bin_poses = {}

    def publish_poses(self, pub, poses):
        pose_arr = PoseArray()
        frame = "/base_link"
        for pose in poses:
            pose_msg = PoseConv.to_pose_msg(pose)
            pose_arr.poses.append(pose_msg)
        pose_arr.header.frame_id = frame
        pose_arr.header.stamp = rospy.Time.now()
        pub.publish(pose_arr)

    # forces bin location from a pose in the base_link frame
    def set_bin_location(self, mid, pose, cur_time=0.):
        with self.lock:
            # cur_time = rospy.get_time()
            self.ar_poses[mid].clear()
            self.ar_poses[mid].append([cur_time, PoseConv.to_pose_msg(pose)])

    def set_real_bin_location(self, mid, pose, cur_time=0.):
        self.ar_poses[mid].clear()
        self.ar_poses[mid+10].clear()
        self.real_bin_poses[mid] = pose

    def get_available_bins(self):
        with self.lock:
            bins = []
            for mid in self.ar_poses:
                if (len(self.ar_poses[mid]) > 0 and 
                        (self.available_bins is None or mid in self.available_bins)):
                    bins.append(mid)
            return bins

    def clean_ar_pose(self, ar_pose):
        ar_pose_mat = self.camera_pose * PoseConv.to_homo_mat(ar_pose)
        ang = np.arctan2(ar_pose_mat[1,0], ar_pose_mat[0,0])   
        if ang<-math.pi:
            ang += 2*math.pi
        if ang>math.pi:
            ang -= 2*math.pi
        return (ar_pose_mat[:3,3].T.A[0], 
                [0., 0., ang])

    def get_bin_pose(self, bin_id):
        with self.lock:
            if bin_id not in self.ar_poses:
                print "Bin ID %d not found!" % bin_id
                return None, None
            pos_list, rot_list = [], []
            for cur_time, pose in self.ar_poses[bin_id]:
                pos, rot = self.clean_ar_pose(pose)
                pos_list.append(pos)
                rot_list.append(rot)
            
            # Making sure we use an odd number of values for median computation
            if len(pos_list) > 1 and len(pos_list) % 2 == 0:
                rot_list.pop()
            
            med_pos, med_rot = np.median(pos_list,0), np.median(rot_list,0)
            return (med_pos.tolist(), med_rot.tolist())

    def get_all_bin_poses(self):
        with self.lock:
            bin_data = {}
            for bin_id in self.get_available_bins():
                bin_pose = self.get_bin_pose(bin_id)
                bin_data[bin_id] = [bin_pose[0], bin_pose[1]]
            return bin_data

    def get_bin_slot(self, slot_id):
        return self.bin_slots[slot_id]

    def get_slot_ids(self):
        return sorted(self.bin_slots.keys())

    def get_bin_slot_states(self):
        bin_poses = self.get_all_bin_poses()
        bin_ids = sorted(bin_poses.keys())
        bin_pos_data = np.array([bin_poses[bin_id][0] for bin_id in bin_ids])
        if len(bin_pos_data) != 0:
            dists, inds = self.slot_tree.query(bin_pos_data, k=1, 
                                               distance_upper_bound=self.ar_unification_thresh)
        else:
            dists, inds = [], []

        slot_states = [-1] * len(self.slot_tree.data)
        missing_bins = []
        for i, ind in enumerate(inds):
            bin_id = bin_ids[i]
            if ind == len(slot_states):
                missing_bins.append(bin_id)
                continue
            slot_states[ind] = bin_id
        return slot_states, missing_bins

    def get_filled_slots(self, slots_to_check=None, invert_set=False):
        if slots_to_check is None:
            slots_to_check = self.bin_slots.keys()
        slot_states, _ = self.get_bin_slot_states()
        slot_ids = self.get_slot_ids()
        bins = []
        for ind, slot_state in enumerate(slot_states):
            if slot_state != -1:
                slot_in_set = slot_ids[ind] in slots_to_check
                if np.logical_xor(not slot_in_set, not invert_set):
                    bins.append(slot_state)
        return sorted(bins)

    def get_empty_slots(self, slots_to_check=None, invert_set=False):
        if slots_to_check is None:
            slots_to_check = self.bin_slots.keys()
        slot_states, _ = self.get_bin_slot_states()
        slot_ids = self.get_slot_ids()
        empty_slots = []
        for ind, slot_state in enumerate(slot_states):
            if slot_state == -1:
                slot_in_set = slot_ids[ind] in slots_to_check
                if np.logical_xor(not slot_in_set, not invert_set):
                    empty_slots.append(slot_ids[ind])
        return empty_slots

    def get_random_empty_slot(self, slots_to_check=None, invert_set=False):
        empty_slots = self.get_empty_slots(slots_to_check, invert_set)
        print 'get_random_slot empty_slots', empty_slots
        if len(empty_slots) == 0:
            return None, None
        rand_slot = empty_slots[np.random.randint(len(empty_slots))]
        rand_slot_loc = self.get_bin_slot(rand_slot)
        return rand_slot, rand_slot_loc

    def get_random_bin(self, slots_to_check=None, invert_set=False):
        bins = self.get_filled_slots(slots_to_check, invert_set)
        print 'get_random_bin filled_slots', bins
        if len(bins) == 0:
            return None, None
        rand_bin = bins[np.random.randint(len(bins))]
        rand_bin_loc = self.get_bin_pose(rand_bin)
        return rand_bin, rand_bin_loc
    
    #### NEW FONCTIONS RELATED TO BINS AND NOT TAGS
    
    def get_real_bin_poses(self):
        with self.lock:
            real_bin_data = {}
            for bin_id in self.real_bin_poses:
                pos,rot  = self.clean_ar_pose(self.real_bin_poses[bin_id])
                real_bin_data[bin_id] = [[pos[0],pos[1],pos[2]],[rot[0],rot[1],rot[2]]]
            return real_bin_data
        
    def get_real_filled_slots(self, slots_to_check=None, invert_set=False):
        if slots_to_check is None:
            slots_to_check = self.bin_slots.keys()
        slot_states, _ = self.get_real_bin_slot_states()
        slot_ids = self.get_slot_ids()
        bins = []
        for ind, slot_state in enumerate(slot_states):
            if slot_state != -1:
                slot_in_set = slot_ids[ind] in slots_to_check
                if np.logical_xor(not slot_in_set, not invert_set):
                    bins.append(slot_state)
        return sorted(bins)

    def get_real_empty_slots(self, slots_to_check=None, invert_set=False):
        if slots_to_check is None:
            slots_to_check = self.bin_slots.keys()
        slot_states, _ = self.get_real_bin_slot_states()
        slot_ids = self.get_slot_ids()
        empty_slots = []
        for ind, slot_state in enumerate(slot_states):
            if slot_state == -1:
                slot_in_set = slot_ids[ind] in slots_to_check
                if np.logical_xor(not slot_in_set, not invert_set):
                    empty_slots.append(slot_ids[ind])
        return empty_slots
    
    def get_real_bin_slot_states(self):
        bin_poses = self.get_real_bin_poses()
        bin_ids = sorted(bin_poses.keys())
        bin_pos_data = np.array([bin_poses[bin_id][0] for bin_id in bin_ids])
        if len(bin_pos_data) != 0:
            dists, inds = self.slot_tree.query(bin_pos_data, k=1, 
                                               distance_upper_bound=self.ar_unification_thresh)
        else:
            dists, inds = [], []

        slot_states = [-1] * len(self.slot_tree.data)
        missing_bins = []
        for i, ind in enumerate(inds):
            bin_id = bin_ids[i]
            if ind == len(slot_states):
                missing_bins.append(bin_id)
                continue
            slot_states[ind] = bin_id
        return slot_states, missing_bins
     
        
    #### NEW FONCTIONS RELATED TO BINS AND NOT TAGS
    def empty_slots(self):
	rospy.wait_for_service('is_bin_location_empty')
	isBinLocationEmpty = rospy.ServiceProxy('is_bin_location_empty', BinLocationEmpty)
	
	collision_object = CollisionObject()
	collision_object.header.frame_id = "table_link"
	collision_object.id = "large_bin"  #need to be "small_bin" to load a small bin
	collision_object.operation = collision_object.ADD	

	slots_ids = self.get_slot_ids()	

	empty_slots = []

	for slot_id in slots_ids:
	    # clearing the collision object
	    collision_object.meshes = []
	    collision_object.mesh_poses = []

	    # specifying the bin's pose
	    pose = self.get_bin_slot(slot_id)
	    pose_msg = Pose()
	    pose_msg.position.x = pose[0][0]
	    pose_msg.position.y = pose[0][1]
	    pose_msg.position.z = pose[0][2]
	    quat = quaternion_from_euler(pose[1][0], pose[1][1], pose[1][2])
	    pose_msg.orientation.x = quat[0]
	    pose_msg.orientation.y = quat[1]
	    pose_msg.orientation.z = quat[2]
	    pose_msg.orientation.w = quat[3]
	    collision_object.mesh_poses.append(pose_msg) 
	    
	    # call the service to determine if the slot is available
	    res = isBinLocationEmpty(collision_object)
	    if res.empty:
		empty_slots.append(slot_id)

	return empty_slots

    def filled_slots(self):
	return false



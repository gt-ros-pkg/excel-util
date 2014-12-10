#! /usr/bin/python

import numpy as np
from collections import deque
from threading import RLock
import yaml
import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from ar_track_alvar.msg import AlvarMarkers, AlvarMarker
from hrl_geom.pose_converter import PoseConv
from ar_tag_manager_iface import ARTagManagerInterface, load_bin_slots
from roslaunch.substitution_args import resolve_args
from std_msgs.msg import Int8MultiArray, Int32

from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import *
from moveit_msgs.srv import *
from excel_bins.srv import UpdateBins, UpdateBinsResponse

from excel_bins.msg import Bins, Bin 
from excel_servers.srv import BinLocationEmpty

class BinPerception(object):
    def __init__(self, bin_tag_pairs):
        self.bin_tag_pairs = bin_tag_pairs
        self.tag_bin_ids = {}
        for bin_id in bin_tag_pairs:
            self.tag_bin_ids[bin_tag_pairs[bin_id][0]] = (bin_id, 0)
            self.tag_bin_ids[bin_tag_pairs[bin_id][1]] = (bin_id, 1)

        self.table_height = 0.88
        self.workspace_y_thresh = 0.95
        self.bin_small_height = 0.13
        self.bin_large_height = 0.18
        self.toolbox_height = 0.18
        self.large_bin_mesh_fn = "/home/hyatt/dev/hydro_ws/src/excel_util/excel_bins/meshes/bin_large.stl"

        self.tag_pose_hist = {}
        # FILTER SIZE HAS TO BE ODD because of the median
        self.filter_size = 5
        self.bins_not_tracking = []
        self.forced_bin_poses = {}

        self.lock = RLock()

        self.ar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_marker_cb, queue_size=1)
        self.plan_scene_iface = PlanningSceneInterface()
        self.get_plan_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        self.update_scene_srv = rospy.Service('/planning_scene_update_bins', UpdateBins, self.planning_scene_update_bins_srv)

    def ar_marker_cb(self, msg):
        if self.lock.acquire(blocking=0):
            cur_time = rospy.get_time()
            for marker in msg.markers:
                if marker.id not in self.tag_bin_ids:
                    # we don't know which bin this tag belongs to, ignore it
                    continue
                bin_id_for_tag, bin_pair_ind = self.tag_bin_ids[marker.id]
                # other_tag_id = self.bin_tag_pairs[bin_id_for_tag][1-bin_pair_ind]
                if bin_id_for_tag in self.bins_not_tracking:
                    # don't track tags for this bin
                    continue

                if marker.id not in self.tag_pose_hist:
                    self.tag_pose_hist[marker.id] = deque()

                if len(self.tag_pose_hist[marker.id]) == self.filter_size:
                    # if len(self.tag_pose_hist[other_tag_id]) > 0:
                    #     other_tag_min_time = self.tag_pose_hist[other_tag_id][0][0]
                    #     # cur_tag_max_time = self.tag_pose_hist[marker.id][-1][0]
                    #     time_ahead = cur_time - other_tag_min_time
                    #     if time_ahead > 4.:
                    #         continue
                    # else:
                    #     continue
                    self.tag_pose_hist[marker.id].popleft()

                self.tag_pose_hist[marker.id].append([cur_time, marker.pose])

            self.lock.release()

    def get_ar_tag_pose(self, tag_id):
        with self.lock:
            if tag_id not in self.tag_pose_hist:
                return None, None, None
            if len(self.tag_pose_hist[tag_id]) == 0:
                return None, None, None

            pos_list, rot_list, time_list = [], [], []
            for cur_time, pose in self.tag_pose_hist[tag_id]:
                pos, rot = self.clean_ar_pose(pose)
                pos_list.append(pos)
                rot_list.append(rot)
                time_list.append(cur_time)

            # Making sure we use an odd number of values for median computation
            if len(pos_list) > 1 and len(pos_list) % 2 == 0:
                rot_list.pop()

            med_pos, med_rot, med_time = np.median(pos_list,0), np.median(rot_list,0), np.median(time_list,0)
            return (med_pos.tolist(), med_rot.tolist(), med_time.tolist())

    def clean_ar_pose(self, ar_pose):
        ar_pose_mat = PoseConv.to_homo_mat(ar_pose)
        ang = np.arctan2(ar_pose_mat[1,0], ar_pose_mat[0,0])   
        if ang<-math.pi:
            ang += 2*math.pi
        if ang>math.pi:
            ang -= 2*math.pi
        return (ar_pose_mat[:3,3].T.A[0], [0., 0., ang])

    def get_bin_pose(self, bin_id):
        tag_id_1 = self.bin_tag_pairs[bin_id][0]
        tag_id_2 = self.bin_tag_pairs[bin_id][1]
        tag_pos_1, tag_quat_1, tag_time_1 = self.get_ar_tag_pose(tag_id_1)
        tag_pos_2, tag_quat_2, tag_time_2 = self.get_ar_tag_pose(tag_id_2)

        if tag_time_1 is not None and tag_time_2 is not None:
            diff_time = abs(tag_time_1 - tag_time_2)
            if diff_time > 2.:
                print "\nDifference in times greater than 2 seconds found for bin", bin_id
                print "One tag is probably occluded"
            
        if tag_pos_1 is None or tag_pos_2 is None:
            # we don't have a history for both tags
            if bin_id in self.forced_bin_poses:
                # we have a manually set bin pose
                return self.forced_bin_poses[bin_id]
            else:
                return (None, None)
        bin_pos = (np.array(tag_pos_1) + np.array(tag_pos_2)) / 2.
        bin_pos[2] = self.table_height

        tag_pos_diff = (np.array(tag_pos_1) - np.array(tag_pos_2))
        ang = np.arctan2(tag_pos_diff[0], tag_pos_diff[1])

        return (bin_pos, [0, 0, -ang])

    def list_bins_in_scene(self):
        bin_ids_with_dups = [self.tag_bin_ids[tag_id][0] for tag_id in self.tag_pose_hist]
        bin_ids_no_dups = sorted(list(set(bin_ids_with_dups)))
        bins_in_scene = []
        for bin_id in bin_ids_no_dups:
            bin_pos, bin_ang = self.get_bin_pose(bin_id)
            if bin_pos is not None:
                bins_in_scene.append(bin_id)
        return bins_in_scene

    def list_bins_in_workspace(self):
        bin_ids_workspace = []
        for bin_id in self.list_bins_in_scene():
            bin_pos, bin_quat = self.get_bin_pose(bin_id)
            if bin_pos[1] > self.workspace_y_thresh:
                bin_ids_workspace.append(bin_id)
        return bin_ids_workspace

    def force_bin_pose(self, bin_id, bin_pose):
        bin_pose[0][2] = self.table_height
        tag_id_1 = self.bin_tag_pairs[bin_id][0]
        tag_id_2 = self.bin_tag_pairs[bin_id][1]
        if tag_id_1 not in self.tag_pose_hist:
            self.tag_pose_hist[tag_id_1] = deque()
        self.tag_pose_hist[tag_id_1].clear()
        if tag_id_2 not in self.tag_pose_hist:
            self.tag_pose_hist[tag_id_2] = deque()
        self.tag_pose_hist[tag_id_2].clear()
        self.forced_bin_poses[bin_id] = bin_pose

    def planning_scene_update_bins(self, bins_to_ignore=[]):
        bins_not_in_scene = self.bins_not_in_planning_scene()
        for bin_id in self.list_bins_in_scene():
            if bin_id in bins_to_ignore:
                continue
            bin_id_name = 'bin#%d' % bin_id
            bin_pose = PoseConv.to_pose_stamped_msg('/table_link', self.get_bin_pose(bin_id))
            if bin_id in bins_not_in_scene:
                self.plan_scene_iface.add_mesh(bin_id_name, bin_pose, self.large_bin_mesh_fn)
                print "\n"*2
                print "Adding %s to the scene" % bin_id_name
                print "\n"*2
            else:
                move_obj = CollisionObject()
                move_obj.id = bin_id_name
                move_obj.header.frame_id = '/table_link'
                move_obj.operation = move_obj.MOVE
                move_obj.mesh_poses.append(bin_pose.pose)
                self.plan_scene_iface._pub_co.publish(move_obj)

    def planning_scene_update_bins_srv(self, req):
        self.planning_scene_update_bins(req.bins_to_ignore)
        return UpdateBinsResponse()

    def bins_not_in_planning_scene(self):
        gps_req = GetPlanningSceneRequest()
        gps_req.components.components = gps_req.components.WORLD_OBJECT_NAMES
        plan_scene = self.get_plan_scene(gps_req)
        obj_ids = [obj.id for obj in plan_scene.scene.world.collision_objects]
        
        bin_ids = self.list_bins_in_scene()
        bins_not_in_scene = []
        for bin_id in bin_ids:
            bin_id_name = 'bin#%d' % bin_id
            if bin_id_name not in obj_ids:
                bins_not_in_scene.append(bin_id)
        return bins_not_in_scene
    
    def clear_bin_from_scene(self, bin_id):
        bin_id_name = 'bin#%d' % bin_id
        remove_obj = CollisionObject()
        remove_obj.id = bin_id_name
        remove_obj.operation = remove_obj.REMOVE
        self.plan_scene_iface._pub_co.publish(remove_obj)

    def clear_all_bins_from_scene(self):
        bins_not_in_scene = self.bins_not_in_planning_scene()
        for bin_id in self.bin_tag_pairs:
            if bin_id not in bins_not_in_scene:
                self.clear_bin_from_scene(bin_id)

    def pause_bin_tracking(self, bin_id, pause=True):
        if pause:
            self.bins_not_tracking.append(bin_id)
        else:
            self.bins_not_tracking.remove(bin_id)

class SlotManager(object):
    def __init__(self, slots, hum_ws_slots, bin_home_slots):
        self.slots = slots
        self.hum_ws_slots = hum_ws_slots
        self.bin_home_slots = bin_home_slots

        self.is_bin_loc_empty_srv = rospy.ServiceProxy('is_bin_location_empty', BinLocationEmpty)
        print "Waiting for is_bin_location_empty to be available"
        self.is_bin_loc_empty_srv.wait_for_service()
        print "is_bin_location_empty found!"

    def get_empty_slots(self, use_hum_ws_slots):
        test_col_objs = []
        test_slot_ids = []
        for slot_id in self.slots:
            if use_hum_ws_slots:
                if slot_id not in self.hum_ws_slots:
                    continue
            else:
                if slot_id in self.hum_ws_slots:
                    continue

            test_col_obj = CollisionObject()
            test_col_obj.header.frame_id = "table_link"
            test_col_obj.id = "large_bin"  #need to be "small_bin" to load a small bin
            test_col_obj.operation = test_col_obj.ADD

            # specifying the bin's pose
            pose_msg = PoseConv.to_pose_msg(self.slots[slot_id])
            test_col_obj.mesh_poses.append(pose_msg) 

            # add the collision object to the array
            test_col_objs.append(test_col_obj)
            test_slot_ids.append(slot_id)

        # call the service to determine if the slot is available
        try:
            res = self.is_bin_loc_empty_srv(test_col_objs)
        except Exception as e:
            print "\n"*3
            print "isBinLocationEmpty failed:", e
            print "\n"*3
            return []

        empty_slots = []
        for i, slot_id in enumerate(test_slot_ids):
            if res.empty[i]:
                empty_slots.append(slot_id)
        return empty_slots

    def get_remove_to_slot(self, bin_to_remove):
        remove_to_slot = -100
        if bin_to_remove in self.bin_home_slots:
            remove_to_slot = self.bin_home_slots[bin_to_remove]
        empty_rob_ws_slots = self.get_empty_slots(use_hum_ws_slots=False)
        if len(empty_rob_ws_slots) == 0:
            return None
        print "Current empty robot workspace slots:", empty_rob_ws_slots
        if remove_to_slot not in empty_rob_ws_slots:
            print "Slot", remove_to_slot, "not an empty slot, picking first empty slot"
            remove_to_slot = empty_rob_ws_slots[0]
        return remove_to_slot

    def get_deliver_to_slot(self, bin_to_deliver):
        empty_hum_ws_slots = self.get_empty_slots(use_hum_ws_slots=True)
        if len(empty_hum_ws_slots) == 0:
            return None
        return empty_hum_ws_slots[0]

    def get_slot_pose(self, slot_id):
        return self.slots[slot_id]

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

def main():
    rospy.init_node('bin_perception')

    ws_setup_fn = '$(find excel_bins)/src/excel_bins/bin_workspace_setup.yaml'
    slots, tags_list, bin_home_slots, hum_ws_slots, bin_barcode_ids = load_ws_setup(ws_setup_fn)
    print "Slots:", slots
    print "Bin homes:", bin_home_slots
    print "Human workspace slots:", hum_ws_slots
    print "Bin barcode IDs:", bin_barcode_ids

    slot_manager = SlotManager(slots, bin_home_slots, hum_ws_slots)

    # find the slot ids likely to be in the human workspace
    # hum_ws_slots = []
    # for slot_id in bin_slots:
    #     # > 0.95 (other side of rail), < 1.5 (human work table)
    #     if bin_slots[slot_id][0][1] > 0.95 and bin_slots[slot_id][0][0] < 1.5:
    #         hum_ws_slots.append(slot_id)

    # tags_list = [1,11,2,12,3,13,4,14,5,15,6,16,7,17,8,18,9,19,100,101]
    bin_tag_pairs = { 1: [1, 11], 
                      3: [3, 13], 
                      5: [5, 15], 
                      7: [7, 17],
                      9: [9, 19] }
    bin_percep = BinPerception(bin_tag_pairs)
    rospy.sleep(1.0)
    bin_percep.clear_all_bins_from_scene()

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        # print "tag 1", bin_percep.get_ar_tag_pose(1)
        if False:
            print "bin 3", bin_percep.get_bin_pose(3)
            print "list_bins_in_scene", bin_percep.list_bins_in_scene()
            print "list_bins_in_workspace", bin_percep.list_bins_in_workspace()
            print "bins_not_in_planning_scene", bin_percep.bins_not_in_planning_scene()
            bin_percep.planning_scene_update_bins()
            print "get_empty_slots hum_ws", slot_manager.get_empty_slots(True)
            print "get_empty_slots rob_ws", slot_manager.get_empty_slots(False)
        if True:
            print "list_bins_in_scene", bin_percep.list_bins_in_scene()
        r.sleep()

if __name__ == "__main__":
    main()

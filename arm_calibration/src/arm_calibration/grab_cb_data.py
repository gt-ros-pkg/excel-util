#! /usr/bin/python

import numpy as np
import yaml
import sys
import cv

import rospy
import rosbag
import tf
from sensor_msgs.msg import Image, PointCloud2, CameraInfo

from hrl_geom.pose_converter import PoseConv
from cv_bridge import CvBridge, CvBridgeError
from camera_calibration.calibrator import ChessboardInfo, MonoCalibrator
from sensor_msgs.point_cloud2 import read_points, create_cloud, create_cloud_xyz32

class DataListener(object):
    def __init__(self, is_kinect, bridge, calib):
        self.is_kinect = is_kinect
        self.cur_img = None
        self.cur_pc = None
        self.cam_info = None
        self.cur_corners = None
        self.bridge = bridge
        self.calib = calib
        self.cam_sub = rospy.Subscriber("/camera", Image, self.sub_img)
        if self.is_kinect:
            self.pc_sub = rospy.Subscriber("/pc", PointCloud2, self.sub_pc)
        else:
            self.cam_info_sub = rospy.Subscriber("/camera_info", CameraInfo, self.sub_info)
        self.vis_pub = rospy.Publisher("/cb_img_raw", Image)
        print "Waiting for image/PC"
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.cur_img is not None:
                if self.is_kinect:
                    if self.cur_pc is not None:
                        break
                else:
                    if self.cam_info is not None:
                        break
            r.sleep()
        print "Received image/PC"

    def sub_img(self, img):
        self.cur_img = img
        prev_num_corners = len(self.calib.good_corners)
        rv = self.calib.handle_msg(img)

        # publish new image with drawn corners
        new_img = self.bridge.cv_to_imgmsg(rv.scrib, "rgb8")
        new_img.header = img.header
        self.vis_pub.publish(new_img)
        # if not has_corners:
        #     # self.cur_corners = None
        #     return
        if len(self.calib.good_corners) > prev_num_corners:
            self.cur_corners = self.calib.good_corners[-1][0]

    def sub_pc(self, pc):
        self.cur_pc = pc

    def sub_info(self, cam_info):
        self.cam_info = cam_info

    def wait_for_new(self):#, timeout):

        # last_img_id = self.cur_img.header.seq
        # if self.is_kinect:
        #     last_other_id = self.cur_pc.header.seq
        # else:
        #     last_other_id = self.cam_info.header.seq
        r = rospy.Rate(10)
        # start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if self.cur_corners:
                corners = self.cur_corners
                self.cur_corners = None
                return corners
            # if rospy.get_time() - start_time > timeout:
            #     print "Timed out"
            #     return None
            # if last_img_id != self.cur_img.header.seq:
            #     if self.is_kinect:
            #         cur_id = self.cur_pc.header.seq
            #     else:
            #         cur_id = self.cam_info.header.seq
            #     if last_other_id != cur_id:
            #         return self.cur_corners
            r.sleep()

def main():
    if len(sys.argv) < 5:
        print 'grab_cbs_auto <rowsxcols (8x6)> <squaredim millimeters> <is kinect 0/1> output_bag.bag'
        return
    rospy.init_node("grab_cbs")

    is_kinect = int(sys.argv[3])

    # load cb stuff
    rowsxcols = sys.argv[1].split('x')
    chessboard = ChessboardInfo(int(rowsxcols[0]), int(rowsxcols[1]), float(sys.argv[2])/1000.)
    print chessboard.n_cols
    print chessboard.n_rows
    print chessboard.dim
    calib = MonoCalibrator([chessboard])
    bridge = CvBridge()

    tf_list = tf.TransformListener()
    rospy.sleep(1.)
    l = DataListener(is_kinect, bridge, calib)

    cb_knowns = []
    for j in range(chessboard.n_cols):
        for i in range(chessboard.n_rows):
            cb_knowns.append((chessboard.dim*i, chessboard.dim*j, 0.0))
        
    bag = rosbag.Bag(sys.argv[4], 'w')
    def shutdown_hook():
        bag.close()
    rospy.on_shutdown(shutdown_hook)
    i = 0
    while not rospy.is_shutdown():

        # tries = 0
        while not rospy.is_shutdown(): # and tries < 3:
            corners = l.wait_for_new() #5.)
            if corners is None:
                print "No corners detected"
                tries += 1
                continue
            corners_2d = np.uint32(np.round(corners)).tolist()
            corners_3d = []
            if is_kinect:
                for x,y,z in read_points(l.cur_pc, field_names=['x', 'y', 'z'], uvs=corners_2d):
                    corners_3d.append((x,y,z))
                frame_id = l.cur_pc.header
            else:
                obj_pts = cv.fromarray(np.array(cb_knowns))
                img_pts = cv.fromarray(np.array(corners))
                K = cv.fromarray(np.reshape(l.cam_info.K,(3,3)))
                D = cv.fromarray(np.array([l.cam_info.D]))
                R_vec = cv.fromarray(np.zeros((3,1)))
                t = cv.fromarray(np.zeros((3,1)))
                cv.FindExtrinsicCameraParams2(obj_pts, img_pts, K, D, R_vec, t)
                R_mat = cv.fromarray(np.zeros((3,3)))
                cv.Rodrigues2(R_vec, R_mat)
                T = PoseConv.to_homo_mat(np.mat(np.asarray(t)).T.A.tolist(), 
                                         np.mat(np.asarray(R_mat)).A.tolist())
                cb_knowns_mat = np.vstack((np.mat(cb_knowns).T, np.mat(np.ones((1, len(cb_knowns))))))
                corners_3d = np.array((T * cb_knowns_mat)[:3,:].T)
                frame_id = l.cur_img.header
            print corners_3d
            if np.any(np.isnan(corners_3d)):
                print "Pointcloud malformed"
                tries += 1
                continue
            now = rospy.Time.now()
            corners_pc = create_cloud_xyz32(frame_id, corners_3d)
            tf_list.waitForTransform('/base_link', '/wrist_3_link', now, rospy.Duration(1.))
            try:
                pose = tf_list.lookupTransform('/base_link', '/wrist_3_link', now)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "TF screwed up..."
                continue
            bag.write("/pose", PoseConv.to_pose_stamped_msg('/base_link', pose), now)
            bag.write("/pc", corners_pc, now)
            print "Wrote pose/CB to bag file"
            break
        i += 1
        # if raw_input("Press enter to continue, type 'q' to quit: ") == "q":
        #      break
    # bag.close()

if __name__ == "__main__":
    main()

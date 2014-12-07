#! /usr/bin/env python

import numpy as np

import roslib
import rospy

from geometry_msgs.msg import PoseStamped, PoseArray

class BinFollow(object):

    def __init__(self):
        self.est_pose_sub = rospy.Subscriber('/human/estimated/pose', PoseArray, self.est_pose_cb)
        self.set_point = rospy.wait_for_message('/vel_cart_pos_ctrl/cur_pose', PoseStamped)
        self.cmd_pose_pub = rospy.Publisher('/vel_cart_pos_ctrl/command_pose', PoseStamped)
        rospy.loginfo("Found current Cartesian pose, starting")

    def est_pose_cb(self, msg):
        if msg.poses[0].position.x != msg.poses[0].position.x:
            return
        self.set_point.pose.position.x = np.clip(msg.poses[0].position.x, 0.3, 2.3)
        self.set_point.pose.position.y = np.clip(msg.poses[0].position.y-0.8, 1.2, 1.8)
        self.cmd_pose_pub.publish(self.set_point)

def main():
    rospy.init_node("bin_follow")
    bf = BinFollow()
    rospy.spin()

if __name__ == '__main__':
    main()

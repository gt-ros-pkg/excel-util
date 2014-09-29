#!/usr/bin/env python

import copy
import numpy as np
from collections import deque

import rospy
from tf import TransformListener
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3Stamped, WrenchStamped

from pykdl_utils.kdl_kinematics import create_kdl_kin

HIST_LEN = 10

class TorqueErrorFix(object):
    def __init__(self):
        self.kin = create_kdl_kin("/base_link", "/ee_link")
        self.joint_names = self.kin.get_joint_names()
        self.des_i = None

        self.torque_err_hist = [deque() for i in range(6)]

        self.force_err_pub = rospy.Publisher("wrench_err", WrenchStamped)
        self.desired_i_sub = rospy.Subscriber("/mode_state_pub/desired_i", 
                                              Float64MultiArray, self.desired_i_cb)
        self.js_sub = rospy.Subscriber("/joint_states", JointState, self.joint_states_cb)

    def desired_i_cb(self, msg):
        self.des_i = msg.data

    def joint_states_cb(self, js):
        if self.des_i is None:
            return
        q, qd, eff = self.kin.extract_joint_state(js, self.joint_names)
        if q is None or eff is None:
            rospy.logwarn("Cannot extract JointState")
            return
        J = self.kin.jacobian(q)

        for i in range(6):
            # self.torque_err_hist[i].append(self.des_i[i]-eff[i])
            self.torque_err_hist[i].append(-eff[i])
            if len(self.torque_err_hist[i]) > HIST_LEN:
                self.torque_err_hist[i].popleft()
        if len(self.torque_err_hist[i]) != HIST_LEN:
            return

        torque_err = np.mat(np.zeros((len(q),1)))
        # for i in [0, 1, 2, 3, 4, 5]:
        for i in [0, 5]:
            torque_err[i] = np.mean(self.torque_err_hist[i])

        try:
            force_err = 10.0*np.linalg.solve(J.T, torque_err)
        except np.linalg.LinAlgError as e:
            print "Jacobian singular"
            print e
            return

        ws_err = WrenchStamped()
        ws_err.header = js.header
        ws_err.header.frame_id = "base_link"
        ws_err.wrench.force.x = force_err[0]
        ws_err.wrench.force.y = force_err[1]
        ws_err.wrench.force.z = force_err[2]
        ws_err.wrench.torque.x = force_err[3]
        ws_err.wrench.torque.y = force_err[4]
        ws_err.wrench.torque.z = force_err[5]
        self.force_err_pub.publish(ws_err)

def main():
    rospy.init_node("torque_err_fix")
    tef = TorqueErrorFix()
    rospy.spin()

if __name__ == "__main__":
    main()

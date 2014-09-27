#!/usr/bin/env python

import copy
import numpy as np

import rospy
from tf import TransformListener
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3Stamped, WrenchStamped

from pykdl_utils.kdl_kinematics import create_kdl_kin

class TorqueErrorFix(object):
    def __init__(self):
        self.kin = create_kdl_kin("/base_link", "/ee_link")
        self.joint_names = self.kin.get_joint_names()

        self.force_err_pub = rospy.Publisher("wrench_err", WrenchStamped)
        self.js_sub = rospy.Subscriber("/joint_states", JointState, self.joint_states_cb)

    def joint_states_cb(self, js):
        q, qd, eff = self.kin.extract_joint_state(js, self.joint_names)
        if q is None or eff is None:
            rospy.logwarn("Cannot extract JointState")
            return
        J = self.kin.jacobian(q)
        torque_err = np.mat(np.zeros((len(q),1)))

        torque_err[0] = eff[0]

        try:
            force_err = np.linalg.solve(J.T, torque_err)
        except np.linalg.LinAlgError as e:
            print "Jacobian singular"
            print e
            return

        ws_err = WrenchStamped()
        ws_err.header = js.header
        ws_err.header.frame_id = "/ee_link"
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

#!/usr/bin/python

import numpy as np

import rospy
from rospy import ROSException
import actionlib
from sensor_msgs.msg import JointState
from ur_py_utils.arm_iface import ArmInterface
from indradrive_hw_iface.drive_status_ctrl import DriveStatusControl
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class ExcelInterface(object):
    RAIL_JOINT_NAME = 'table_rail_joint'

    def __init__(self, js_prefix='', config_prefix='/config_fwd_ctrl'):
        self.js_prefix = js_prefix
        self.joint_state_inds = None

        self.joint_names = [ExcelInterface.RAIL_JOINT_NAME] + ArmInterface.JOINT_NAMES

        self.arm = ArmInterface(js_prefix, config_prefix)
        self.rail = DriveStatusControl(topic_prefix='/excel_ctrl_man_xeno/')

        self.vel_jnt_traj_act_cli = actionlib.SimpleActionClient(
                '/vel_trajectory_ctrl/follow_joint_trajectory', 
                FollowJointTrajectoryAction)

    def get_joint_state(self, timeout=1.1):
        try:
            js = rospy.wait_for_message(self.js_prefix + '/joint_states', JointState, timeout)
        except ROSException as e:
            rospy.logwarn('get_joint_state timed out after %1.1f s' % timeout)
            return None
        if self.joint_state_inds is None:
            self.joint_state_inds = [js.name.index(joint_name) for 
                                     joint_name in self.joint_names]
        return js

    def get_q(self, timeout=1.1):
        js = self.get_joint_state(timeout)
        if js is None:
            return None
        q = np.array(js.position)
        return q[self.joint_state_inds]

    def get_qd(self, timeout=1.1):
        js = self.get_joint_state(timeout)
        if js is None:
            return None
        qd = np.array(js.velocity)
        return qd[self.joint_state_inds]

    def get_effort(self, timeout=1.1):
        js = self.get_joint_state(timeout)
        if js is None:
            return None
        effort = np.array(js.effort)
        return effort[self.joint_state_inds]

    def shutdown(self):
        self.arm.shutdown()

def main():
    rospy.init_node("excel_interface")
    excel = ExcelInterface()
    print 'Joint positions:', excel.get_q()
    print 'Joint velocities:', excel.get_qd()
    print 'Joint efforts:', excel.get_effort()

if __name__ == "__main__":
    main()

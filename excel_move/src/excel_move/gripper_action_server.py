#! /usr/bin/env python

import roslib; roslib.load_manifest('actionlib_tutorials')
import rospy
import numpy as np

import actionlib
import robotiq_c_model_control
from robotiq_c_model_control.msg import _CModel_robot_output as outputMsg
from robotiq_c_model_control.robotiq_c_ctrl import RobotiqCGripper

import actionlib_tutorials.msg
import control_msgs.msg


class GripperAction(object):
    # create messages that are used to publish feedback/result
    _feedback = control_msgs.msg.GripperCommandFeedback()
    _result   = control_msgs.msg.GripperCommandResult()

    def __init__(self, name):
        self._action_name = 'gripper_controller/gripper_action'
        self.rob_c_grip = RobotiqCGripper()
        self.rob_c_grip.wait_for_connection()
        if self.rob_c_grip.is_reset():
            self.rob_c_grip.reset()
            self.rob_c_grip.activate()
        self.rob_c_grip.close(block=False)
        rospy.loginfo("Gripper initialized, starting action server: %s" % self._action_name)
        self._as = actionlib.SimpleActionServer(self._action_name, 
                                                control_msgs.msg.GripperCommandAction, 
                                                execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, gripper_command_msg):
        print "Sending command to gripper:"
        print gripper_command_msg

        if np.allclose(self.rob_c_grip.get_pos(), gripper_command_msg.command.position, atol=0.005):
            # already at that position
            self._result.position = self.rob_c_grip.get_pos()
            self._result.effort = gripper_command_msg.command.max_effort
            self._result.stalled = False
            self._result.reached_goal = True
            self._as.set_succeeded(self._result)
            return

        num_retries = 3
        while not rospy.is_shutdown():
            self.rob_c_grip.goto(gripper_command_msg.command.position, 1.0, gripper_command_msg.command.max_effort)
            if self.rob_c_grip.wait_until_moving(1.):
                break
            num_retries -= 1
            if num_retries == 0:
                # timeout
                self._as.set_aborted(text="Timed out waiting for gripper to start moving")
                return

        # gripper should be moving

        timeout = 8.
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                rospy.loginfo("%s: Preempted" % self._action_name)
                self._as.set_preempted()
            if (timeout >= 0. and rospy.get_time() - start_time > timeout) or self.rob_c_grip.is_reset():
                self._as.set_aborted(text="Timed out waiting for gripper to stop moving")
                return
            if self.rob_c_grip.is_stopped():
                break

            self._feedback.position = self.rob_c_grip.get_pos()
            self._feedback.effort = gripper_command_msg.command.max_effort
            self._feedback.stalled = False
            self._feedback.reached_goal = False
            self._as.publish_feedback(self._feedback)
            r.sleep()

        # gripper moved, then stopped
        self._result.position = self.rob_c_grip.get_pos()
        self._result.effort = gripper_command_msg.command.max_effort
        self._result.stalled = False
        self._result.reached_goal = True
        self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('gripper_action_server')
    GripperAction(rospy.get_name())
    rospy.spin()

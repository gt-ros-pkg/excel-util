#! /usr/bin/env python

import roslib; roslib.load_manifest('actionlib_tutorials')
import rospy

import actionlib
import robotiq_c_model_control
from robotiq_c_model_control.msg import _CModel_robot_output as outputMsg

import actionlib_tutorials.msg
import control_msgs.msg


class GripperAction(object):
  # create messages that are used to publish feedback/result
  _feedback = control_msgs.msg.GripperCommandFeedback()
  _result   = control_msgs.msg.GripperCommandResult()

  def __init__(self, name):
    self._action_name = 'gripper_action'
    self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.GripperCommandAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    self.pub = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output)
    
  def execute_cb(self, gripper_command_msg):
    # helper variables
    r = rospy.Rate(1)
    success = True
    
    command = outputMsg.CModel_robot_output()
    command.rACT = 1
    command.rGTO = 1
    command.rSP = 255
    command.rPR = gripper_command_msg.command.position
    command.rFR = gripper_command_msg.command.max_effort
    
    self._feedback.position = gripper_command_msg.command.position
    self._feedback.effort = gripper_command_msg.command.max_effort
    self._feedback.stalled = True
    self._feedback.reached_goal = True
    # publish info to the console for the user
    #rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
    
    # start executing the action
#     for i in xrange(1, goal.order):
#       # check that preempt has not been requested by the client
#       if self._as.is_preempt_requested():
#         rospy.loginfo('%s: Preempted' % self._action_name)
#         self._as.set_preempted()
#         success = False
#         break
#       self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
#       # publish the feedback
#       self._as.publish_feedback(self._feedback)
#       # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
#       r.sleep()
    
    self.pub.publish(command)
    
    if success: 
      self._result.position = self._feedback.position
      self._result.effort = self._feedback.effort
      self._result.stalled = self._feedback.stalled
      self._result.reached_goal = self._feedback.reached_goal
      #rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
  rospy.init_node('gripper_action_server')
  GripperAction(rospy.get_name())
  rospy.spin()
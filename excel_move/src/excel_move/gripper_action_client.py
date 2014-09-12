#! /usr/bin/env python

import roslib; roslib.load_manifest('actionlib_tutorials')
import rospy

# Brings in the SimpleActionClient
import actionlib

import actionlib_tutorials.msg
import control_msgs.msg
import robotiq_c_model_control
from robotiq_c_model_control.msg import _CModel_robot_output as outputMsg

def gripper_client(position, force):
    # Creates the SimpleActionClient, passing the type of the action

    client = actionlib.SimpleActionClient('gripper_controller/gripper_action', control_msgs.msg.GripperCommandAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position = position
    goal.command.max_effort = force

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result() 

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('gripper_action_client')
        position = rospy.get_param('~position')
        force = rospy.get_param('~force')
        result = gripper_client(position, force)
        
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
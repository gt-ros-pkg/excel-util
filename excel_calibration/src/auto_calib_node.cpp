#include <ros/ros.h>
#include <ros/package.h>
#include <moveit/move_group_interface/move_group.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <std_srvs/Empty.h>
#include <control_msgs/GripperCommandAction.h>

using namespace XmlRpc;

double readXmlRpcNumber(XmlRpc::XmlRpcValue& num_val)
{
  ROS_ASSERT(num_val.getType() == XmlRpcValue::TypeDouble ||
             num_val.getType() == XmlRpcValue::TypeInt);
  if(num_val.getType() == XmlRpcValue::TypeInt)
    return (double) static_cast<int>(num_val);

  return static_cast<double>(num_val);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "auto_calib_node");
  ros::NodeHandle nh_param("~");
  bool sim = false, load_board = true;
  nh_param.getParam("sim", sim);
  nh_param.getParam("load_board", load_board);

  ros::NodeHandle nh;
  std_srvs::Empty empty_srv;
  ros::ServiceClient detect_srv = nh.serviceClient<std_srvs::Empty>("/excel_calib_srv/detect_targets");
  ros::ServiceClient calibrate_srv = nh.serviceClient<std_srvs::Empty>("/excel_calib_srv/calibrate");
  ros::ServiceClient save_srv = nh.serviceClient<std_srvs::Empty>("/excel_calib_srv/save");

  std::vector< std::vector<double> > joints_list;
  XmlRpcValue joint_poses;
  nh_param.getParam("joint_poses", joint_poses);
  ROS_ASSERT(joint_poses.getType() == XmlRpcValue::TypeArray);
  joints_list.resize(joint_poses.size());
  for(int i = 0; i < joint_poses.size(); i++) {
    ROS_ASSERT(joint_poses[i].getType() == XmlRpcValue::TypeArray);
    for(int j = 0; j < joint_poses[i].size(); j++)
      joints_list[i].push_back(readXmlRpcNumber(joint_poses[i][j]));
  }

  std::cout <<"list size:"<< joints_list.size()<< std::endl;
  std::cout << "Joint poses:\n";
  for(int i=0; i<joints_list.size(); i++) {
    for(int j=0; j<joints_list[i].size(); j++) {
      std::cout << joints_list[i][j] << ", ";
    }
    std::cout << "\n";
  }

  ros::AsyncSpinner spinner(1);
  spinner.start();
  usleep(1000*1000);

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> excel_ac("vel_pva_trajectory_ctrl/follow_joint_trajectory");
  move_group_interface::MoveGroup group("excel");
  move_group_interface::MoveGroup::Plan my_plan;

  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> 
    gripper_ac("gripper_controller/gripper_action", true);
  gripper_ac.waitForServer();

  for(int i=0; i<joints_list.size(); i++){
    int num_tries = 4;
    group.getCurrentState()->update(true);
    group.setStartStateToCurrentState();
    group.setJointValueTarget(joints_list[i]);
    // try to plan a few times, just to be safe
    while (ros::ok() && num_tries > 0) {
      if (group.plan(my_plan))
        break;
      num_tries--;
    }
    std::cout <<"move to pose "<<i<<std::endl;

    if(!sim){
      ROS_INFO("Running for real");
      excel_ac.waitForServer();

      // Copy trajectory
      control_msgs::FollowJointTrajectoryGoal excel_goal;
      excel_goal.trajectory = my_plan.trajectory_.joint_trajectory;

      // Ask to execute now
      ros::Time time_zero(0.0);
      excel_goal.trajectory.header.stamp = time_zero;

      // Send goal and wait for a result
      excel_ac.sendGoal(excel_goal);
      sleep(0.5);
      if(!excel_ac.waitForResult(ros::Duration(15.))){
        ROS_ERROR("Something wrong");
        break;
      }
    }else
      group.execute(my_plan);

    if (load_board) {
      control_msgs::GripperCommandGoal goal;
      goal.command.position = 0.02;
      goal.command.max_effort = 100;
      gripper_ac.sendGoal(goal);
      gripper_ac.waitForResult(ros::Duration(30.0));

      char blah;
      std::cin >> blah;
      ros::Duration(5.0).sleep();

      goal.command.position = 0.00;
      gripper_ac.sendGoal(goal);
      gripper_ac.waitForResult(ros::Duration(30.0));
      load_board = false;
    }

    //get info from the cameras
    ros::Duration(2.0).sleep();
    detect_srv.call(empty_srv);
    calibrate_srv.call(empty_srv);
    ros::Duration(1.0).sleep();
  }
  calibrate_srv.call(empty_srv);
  save_srv.call(empty_srv);

  spinner.stop();
  return 0;
}

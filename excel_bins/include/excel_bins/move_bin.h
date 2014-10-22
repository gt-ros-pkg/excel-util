#ifndef MOVE_BIN_H
#define MOVE_BIN_H

#include <boost/lexical_cast.hpp>

// ROS/MoveIt includes.
#include "ros/ros.h"
#include "ros/time.h"
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf/transform_broadcaster.h>
#include <moveit/robot_state/attached_body.h>
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <shape_msgs/Mesh.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "tf/transform_datatypes.h"
#include <std_msgs/Bool.h>

# define M_PI 3.14159265358979323846  /* pi */
# define TABLE_HEIGHT 0.88  
# define GRIPPING_OFFSET 0.1  
# define DZ 0.30 

typedef move_group_interface::MoveGroup::Plan MoveGroupPlan;

class MoveBin
{
public:
  // Constructor.
  MoveBin();

  ////////////////////////// Outer actions /////////////////////////
  // The robot tries to plan a trajectory to its home position
  bool moveToHome();

  // From the current joint pose, the robot moves the requested bin from its location
  // to the target location, and backs away
  bool moveBinToTarget(int bin_number, double x_target, double y_target, double angle_target);
  //////////////////////////////////////////////////////////////////

  //////////////// moveBinToTarget composite actions ///////////////
  // From the current joint pose, the robot moves to the grasp location for the given bin
  // The robot is not holding a bin during this method
  bool approachBin(int bin_number, double& bin_height);

  // On the assumption that the robot is currently holding a bin, the robot moves the
  // bin to the target place location, but does not release the bin
  bool deliverBin(double x_target, double y_target, double angle_target, double bin_height);
  //////////////////////////////////////////////////////////////////

  ///////////////////////// Traverse actions ///////////////////////

  // Move arm to pose above bin
  bool moveAboveBin(int bin_number, double& bin_height); 
  void getBinAbovePose(moveit_msgs::CollisionObjectPtr bin_coll_obj, geometry_msgs::Pose& pose,
                       double& bin_height);

  // Move arm to target (X,Y,R), above a place location
  // angle_target is in degrees
  bool carryBinTo(double x_target, double y_target, double angle_target, double bin_height);
  // finds the pose above a target bin
  // angle_target is in degrees
  void getCarryBinPose(double x_target, double y_target, double angle_target, double bin_height, 
                       geometry_msgs::Pose& pose);

  // Move arm across freespace to target pose  
  bool traverseMove(geometry_msgs::Pose& pose);
  //////////////////////////////////////////////////////////////////

  ///////////////////////// Vertical actions ///////////////////////

  // Move arm vertically up above bins
  bool ascent(double bin_height);

  // Move arm vertically down to place height
  bool descent(double bin_height);  

  // From current pose, move arm vertically to target z
  bool verticalMove(double target_z);
  //////////////////////////////////////////////////////////////////

  // grasp bin and attach the collision model to the arm
  bool attachBin(int bin_number);  

  // release bin and detach the collision model from the arm
  bool detachBin();

  // Finds out if the robot needs to rotate clockwise or anti-clockwise
  double optimalGoalAngle(double goal_angle, double current_angle);

  // get the collision object corresponding to the bin_number
  moveit_msgs::CollisionObjectPtr getBinCollisionObject(int bin_number);

  // execute a joint trajectory
  bool executeJointTrajectory(MoveGroupPlan& mg_plan, bool check_safety);
  // stop a joint trajectory
  void stopJointTrajectory();

  bool executeGripperAction(bool is_close, bool wait_for_result);

  void getPlanningScene(moveit_msgs::PlanningScene& planning_scene, 
                        planning_scene::PlanningScenePtr& full_planning_scene);

  void humanUnsafeCallback(const std_msgs::Bool::ConstPtr& msg);

  ros::ServiceClient service_client, fk_client;
  moveit_msgs::GetPositionIK::Request ik_srv_req;
  moveit_msgs::GetPositionIK::Response ik_srv_resp;

  ros::Publisher attached_object_publisher;
  ros::Publisher planning_scene_diff_publisher;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;	
  boost::shared_ptr<tf::TransformListener> tf;
  move_group_interface::MoveGroup group;
  ros::AsyncSpinner spinner;
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_ac;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> excel_ac;
  bool sim;
  moveit_msgs::JointConstraint rail_constraint, shoulder_constraint,elbow_constraint;
  double rail_max, rail_min, rail_tolerance;

  bool vertical_check_safety_, traverse_check_safety_;
  bool human_unsafe_;
  ros::Subscriber hum_unsafe_sub_;

  bool use_gripper;
};

#endif

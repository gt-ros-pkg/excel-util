#ifndef MOVE_BIN_H
#define MOVE_BIN_H

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

# define M_PI 3.14159265358979323846  /* pi */
# define TABLE_HEIGHT 0.88  
# define GRIPPING_OFFSET 0.1  
# define DZ 0.25 

class MoveBin
{
public:
	// Constructor.
	MoveBin();

	/*--------------------------------------------------------------------
	 * move_on_top()
	 * Moves to the specified bin number location
	 *------------------------------------------------------------------*/
	int move_on_top(int bin_number); 

	/*--------------------------------------------------------------------
	 * descent()
	 * Descent to gripping height
	 *------------------------------------------------------------------*/
	int descent();  

	/*--------------------------------------------------------------------
	 * attach_bin()
	 * Attaches the specified bin number to the robot
	 *------------------------------------------------------------------*/
	int attach_bin(int bin_number);  

	/*--------------------------------------------------------------------
	 * ascent()
	 * Ascent to moving height
	 *------------------------------------------------------------------*/
	int ascent();

	/*--------------------------------------------------------------------
	 * carry_bin_to()
	 * Moves to target location keeping the grasping orientation
	 *------------------------------------------------------------------*/
	int carry_bin_to(double x_target, double y_target, double angle_target);

	/*--------------------------------------------------------------------
	 * detach_bin()
	 * Detaches the bin from the robot
	 *------------------------------------------------------------------*/
	int detach_bin();

	/*--------------------------------------------------------------------
	 * optimal_goal_angle()
	 * Finds out if the robot needs to rotate clockwise or anti-clockwise
	 *------------------------------------------------------------------*/
	double optimal_goal_angle(double goal_angle, double current_angle);

	ros::ServiceClient service_client, fk_client;
	moveit_msgs::GetPositionIK::Request service_request;
	moveit_msgs::GetPositionIK::Response service_response;
	moveit_msgs::GetPositionFK::Request fk_request;
	moveit_msgs::GetPositionFK::Response fk_response;

	ros::Publisher attached_object_publisher;
	ros::Publisher planning_scene_diff_publisher;
	planning_scene::PlanningScenePtr full_planning_scene;
	moveit_msgs::PlanningScene planning_scene;
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;	
	boost::shared_ptr<tf::TransformListener> tf;
	move_group_interface::MoveGroup group;
	double bin_height;
	ros::AsyncSpinner spinner;
	actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_ac;
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> excel_ac;
	bool sim;
	moveit_msgs::JointConstraint rail_constraint, shoulder_constraint,elbow_constraint;
	moveit::planning_interface::MoveGroup::Plan my_plan;
	double rail_max, rail_min, rail_tolerance;
};

#endif
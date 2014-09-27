#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/JointConstraint.h>
#include <tf/transform_broadcaster.h>

# define M_PI 3.14159265358979323846  /* pi */

int main(int argc, char **argv)
{
	ros::init(argc, argv, "scan_table");
	ros::NodeHandle nh_param_("~"), nh_;
	usleep(1000*1000);
	// start a ROS spinning thread
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroup::Plan my_plan;
	double x_goal = 0.8, y_goal = 1.8, z_goal = 0.88;

	// this connects to a running instance of the move_group node
	move_group_interface::MoveGroup group("excel");
	group.setEndEffectorLink("gripper_camera_focal_link");

	group.setPlanningTime(10.0);

	// IK Service //
	ros::ServiceClient service_client = nh_.serviceClient<moveit_msgs::GetPositionIK> ("compute_ik");
	while(!service_client.exists())
	{
		ROS_INFO("Waiting for service");
		sleep(1.0);
	}
	moveit_msgs::GetPositionIK::Request service_request;
	moveit_msgs::GetPositionIK::Response service_response;
	service_request.ik_request.group_name = "scanning";
	service_request.ik_request.pose_stamped.header.frame_id = "table_link";
	service_request.ik_request.avoid_collisions = true;
	service_request.ik_request.attempts = 10;
	//service_request.ik_request.ik_link_name = "gripper_camera_focal_link";

	moveit_msgs::JointConstraint shoulder_constraint;
	shoulder_constraint.joint_name = "table_rail_joint";
	shoulder_constraint.position = 3*M_PI/4;
	shoulder_constraint.tolerance_above = M_PI/4;
	shoulder_constraint.tolerance_below = M_PI/4;
	shoulder_constraint.weight = 1;
	service_request.ik_request.constraints.joint_constraints.push_back(shoulder_constraint);
/*
	ROS_INFO("Planning and moving...");
	group.setJointValueTarget("shoulder_pan_joint", 2.33);
	group.setJointValueTarget("shoulder_lift_joint", -1.95);
	group.setJointValueTarget("elbow_joint", 2.21);
	group.setJointValueTarget("wrist_1_joint", -1.26);
	group.setJointValueTarget("wrist_2_joint", -1.41);
	group.setJointValueTarget("wrist_3_joint", 2.22);
	group.setJointValueTarget("table_rail_joint", 2.50);
	//group.setStartState(full_planning_scene->getCurrentState());
	if(group.plan(my_plan)){
		group.execute(my_plan);
		sleep(1.0);
	}else ROS_ERROR("Step 1 failed");
*/
	geometry_msgs::Pose current_pose = group.getCurrentPose().pose;
	service_request.ik_request.pose_stamped.pose = current_pose;
	service_request.ik_request.pose_stamped.pose.position.z = 0.88;
	//service_request.ik_request.pose_stamped.pose.position.x = 0.3;
	service_client.call(service_request, service_response);
	group.setJointValueTarget(service_response.solution.joint_state);
	//group.setStartState(full_planning_scene->getCurrentState());
	if(group.plan(my_plan)){
		group.execute(my_plan);
		sleep(1.0);
	}else ROS_ERROR("Step 2 failed");

	service_request.ik_request.pose_stamped.pose.position.y += 0.1;
	service_client.call(service_request, service_response);
	group.setJointValueTarget(service_response.solution.joint_state);
	//group.setStartState(full_planning_scene->getCurrentState());
	if(group.plan(my_plan)){
		group.execute(my_plan);
		sleep(1.0);
	}else ROS_ERROR("Step 3 failed");

	service_request.ik_request.pose_stamped.pose.position.x += 0.3;
	service_client.call(service_request, service_response);
	group.setJointValueTarget(service_response.solution.joint_state);
	//group.setStartState(full_planning_scene->getCurrentState());
	if(group.plan(my_plan)){
		group.execute(my_plan);
		sleep(1.0);
	}else ROS_ERROR("Step 4 failed");


	service_request.ik_request.pose_stamped.pose.position.y -= 0.1;
	service_client.call(service_request, service_response);
	group.setJointValueTarget(service_response.solution.joint_state);
	//group.setStartState(full_planning_scene->getCurrentState());
	if(group.plan(my_plan)){
		group.execute(my_plan);
		sleep(1.0);
	}else ROS_ERROR("Step 5 failed");


	group.setEndEffectorLink("ee_link");

	moveit_msgs::GetPositionIK::Request service_request_bis;
	moveit_msgs::GetPositionIK::Response service_response_bis;
	service_request_bis.ik_request.group_name = "excel";
	service_request_bis.ik_request.pose_stamped.header.frame_id = "table_link";
	service_request_bis.ik_request.avoid_collisions = true;
	service_request_bis.ik_request.attempts = 10;

	//service_request_bis.ik_request.ik_link_name = "ee_link";
	tf::Quaternion quat = tf::createQuaternionFromRPY(0,M_PI/2,M_PI);
	service_request_bis.ik_request.pose_stamped.pose.orientation.x = quat.x();
	service_request_bis.ik_request.pose_stamped.pose.orientation.y = quat.y();
	service_request_bis.ik_request.pose_stamped.pose.orientation.z = quat.z();
	service_request_bis.ik_request.pose_stamped.pose.orientation.w = quat.w();

	service_request_bis.ik_request.pose_stamped.pose.position.x = 0.5;
	service_request_bis.ik_request.pose_stamped.pose.position.y = 1.3;
	service_request_bis.ik_request.pose_stamped.pose.position.z = 1.4;

	service_client.call(service_request_bis, service_response_bis);

	ROS_INFO("Planning and moving...");
	group.setStartStateToCurrentState();

	group.setJointValueTarget(service_response_bis.solution.joint_state);
	//group.setStartState(full_planning_scene->getCurrentState());
	if(group.plan(my_plan)){
		group.execute(my_plan);
		sleep(1.0);
	}else ROS_ERROR("Step 6 failed");


	ros::shutdown();
}

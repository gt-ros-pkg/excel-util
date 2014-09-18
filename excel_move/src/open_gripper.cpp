#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "open_gripper");
	// start a ROS spinning thread
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	move_group_interface::MoveGroup gripper_group("gripper");
	
	robot_state::RobotStatePtr start = gripper_group.getCurrentState();
	double end = 0.78;
	start->setJointPositions("robotiq_85_left_knuckle_joint", &end);
	gripper_group.setStartState(*start);
	gripper_group.setNamedTarget("opened");
	
	moveit::planning_interface::MoveGroup::Plan my_plan;
	gripper_group.plan(my_plan);
	
	gripper_group.execute(my_plan);
	
	ros::shutdown();
}

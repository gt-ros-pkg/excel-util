#include "back_n_forth.h"

/*--------------------------------------------------------------------
 * MoveBin()
 * Constructor.
 *------------------------------------------------------------------*/
MoveBin::MoveBin() : group("excel"), excel_ac("vel_pva_trajectory_ctrl/follow_joint_trajectory"), gripper_ac("gripper_controller/gripper_action", true) ,spinner(1)
{
	spinner.start();
	boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(ros::Duration(2.0)));
	planning_scene_monitor::PlanningSceneMonitorPtr plg_scn_mon(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf));
	planning_scene_monitor = plg_scn_mon;

	ros::NodeHandle nh_param_("~");
	nh_param_.getParam("sim",sim);

	ros::WallDuration sleep_t(0.5);
	group.setPlanningTime(8.0);
	group.allowReplanning(false);

	service_client = nh_.serviceClient<moveit_msgs::GetPositionIK> ("compute_ik");
	while(!service_client.exists())
	{
		ROS_INFO("Waiting for service");
		sleep(1.0);
	}
	fk_client = nh_.serviceClient<moveit_msgs::GetPositionFK> ("compute_fk");
	while(!fk_client.exists())
	{
		ROS_INFO("Waiting for service");
		sleep(1.0);
	}

	service_request.ik_request.group_name = "excel";
	service_request.ik_request.pose_stamped.header.frame_id = "table_link";
	service_request.ik_request.avoid_collisions = true;
	service_request.ik_request.attempts = 30;

	// Loading planning_scene_monitor //
	planning_scene_monitor->startSceneMonitor();
	planning_scene_monitor->startStateMonitor();
	planning_scene_monitor->startWorldGeometryMonitor();

	// Making sure we can publish attached/unattached objects //
	attached_object_publisher = nh_.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 1);
	while(attached_object_publisher.getNumSubscribers() < 1)
	{
		sleep_t.sleep();
	}
	planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	while(planning_scene_diff_publisher.getNumSubscribers() < 1)
	{
		sleep_t.sleep();
	}

	// Define joint_constraints for the IK service
	rail_constraint.joint_name = "table_rail_joint";
	rail_constraint.position = 2.00;
	rail_constraint.tolerance_above = 1.3;
	rail_constraint.tolerance_below = 1.6;
	rail_constraint.weight = 1;
	shoulder_constraint.joint_name = "shoulder_lift_joint";
	shoulder_constraint.position = -M_PI/4;
	shoulder_constraint.tolerance_above = M_PI/4;
	shoulder_constraint.tolerance_below = M_PI/4;
	shoulder_constraint.weight = 1;
	elbow_constraint.joint_name = "elbow_joint";
	elbow_constraint.position = M_PI/2;
	elbow_constraint.tolerance_above = M_PI/3;
	elbow_constraint.tolerance_below = M_PI/3;
	elbow_constraint.weight = 1;

	rail_max = 3.29;
	rail_min = 0.41;
	rail_tolerance = 0.3;

	if(!sim){
		ROS_INFO("Waiting for action server to be available...");
		excel_ac.waitForServer();
		ROS_INFO("Action server found.");
	}

	robot_stopped = false;
	ros::Subscriber sub = nh_.subscribe("human/safety/stop", 1, &MoveBin::stop_callback,this);
	ros::spinOnce();
}

/*--------------------------------------------------------------------
 * carry_bin_to()
 * Moves to target location keeping the grasping orientation
 *------------------------------------------------------------------*/
int MoveBin::carry_bin_to(double x_target, double y_target, double angle_target)
{
	ROS_INFO("Carrying bin to target");

	planning_scene_monitor->requestPlanningSceneState();
	full_planning_scene = planning_scene_monitor->getPlanningScene();
	full_planning_scene->getPlanningSceneMsg(planning_scene);
	collision_objects = planning_scene.world.collision_objects;

	tf::Quaternion quat_goal = tf::createQuaternionFromRPY(M_PI/2-angle_target*M_PI/180.0,M_PI/2,M_PI);
	service_request.ik_request.pose_stamped.pose.position.x = x_target;
	service_request.ik_request.pose_stamped.pose.position.y = y_target;
	service_request.ik_request.pose_stamped.pose.position.z = TABLE_HEIGHT+GRIPPING_OFFSET+bin_height+DZ;
	service_request.ik_request.pose_stamped.pose.orientation.x = quat_goal.x();
	service_request.ik_request.pose_stamped.pose.orientation.y = quat_goal.y();
	service_request.ik_request.pose_stamped.pose.orientation.z = quat_goal.z();
	service_request.ik_request.pose_stamped.pose.orientation.w = quat_goal.w();

	moveit_msgs::JointConstraint special_rail_constraint;
	special_rail_constraint.joint_name = "table_rail_joint";
	special_rail_constraint.position = rail_max - x_target;
	special_rail_constraint.tolerance_above = std::max(std::min(rail_max - x_target + rail_tolerance, rail_max) - (rail_max - x_target),0.0);
	special_rail_constraint.tolerance_below = std::max((rail_max - x_target) - std::max(rail_max - x_target - rail_tolerance, rail_min),0.0);
	special_rail_constraint.weight = 1;

	service_request.ik_request.constraints.joint_constraints.clear();
	service_request.ik_request.constraints.joint_constraints.push_back(special_rail_constraint);
	service_request.ik_request.constraints.joint_constraints.push_back(shoulder_constraint);
	//service_request.ik_request.constraints.joint_constraints.push_back(elbow_constraint);
	service_client.call(service_request, service_response);
	if(service_response.error_code.val !=1){
		ROS_ERROR("IK couldn't find a solution");
		return 0;
	}

	// Fixing shoulder_pan and wrist_3 given by the IK
	service_response.solution.joint_state.position[1] = this->optimal_goal_angle(service_response.solution.joint_state.position[1],planning_scene.robot_state.joint_state.position[1]);
	service_response.solution.joint_state.position[6] = this->optimal_goal_angle(service_response.solution.joint_state.position[6],planning_scene.robot_state.joint_state.position[6]);

	group.setJointValueTarget(service_response.solution.joint_state);
	group.setStartState(full_planning_scene->getCurrentState());
	if(group.plan(my_plan)){
		if(!sim)
			return executeTrajectory(my_plan.trajectory_.joint_trajectory);
		else 
			group.execute(my_plan);
		return true;
	}
	else{
		ROS_ERROR("Motion planning failed");
		//group.clearPathConstraints();
		return 0;
	}
}

/*--------------------------------------------------------------------
 * optimal_goal_angle()
 * Finds out if the robot needs to rotate clockwise or anti-clockwise
 *------------------------------------------------------------------*/
double MoveBin::optimal_goal_angle(double goal_angle, double current_angle)
{
	while( std::abs(std::max(current_angle,goal_angle) - std::min(current_angle,goal_angle))>M_PI){
		if (goal_angle>current_angle){
			goal_angle -= 2*M_PI;
		}
		else{
			goal_angle += 2*M_PI;
		}
	}
	if(goal_angle>2*M_PI){
		goal_angle -= 2*M_PI;
	}
	if(goal_angle<-2*M_PI){
		goal_angle += 2*M_PI;
	}
	return goal_angle;
}

bool MoveBin::executeTrajectory(trajectory_msgs::JointTrajectory& joint_traj)
{
	// Copy trajectory
	control_msgs::FollowJointTrajectoryGoal excel_goal;
	excel_goal.trajectory = joint_traj;

	// Ask to execute now
	ros::Time time_zero(0.0);
	excel_goal.trajectory.header.stamp = time_zero; 

	// Send goal and wait for a result
	excel_ac.sendGoal(excel_goal);
	
	ros::Rate r(10);
	int count = 0;
	int total_count = 30*10;
	bool is_successful = false;
	
	while ( is_successful && (count<total_count)){
		excel_ac.waitForResult(ros::Duration(0.1));
		ros::spinOnce();
		count++;
		is_successful = excel_ac.getResult()->error_code!=excel_ac.getResult()->SUCCESSFUL;
		r.sleep();
	}
	
	return is_successful;
}

void MoveBin::stop(){
	if(!sim){
		// Empty trajectory
		control_msgs::FollowJointTrajectoryGoal excel_goal;

		// Send goal and wait for a result
		excel_ac.sendGoal(excel_goal);
	}
	else 
		group.stop();
}

void MoveBin::stop_callback(const std_msgs::Bool::ConstPtr& stop)
{
	ROS_INFO("Callback called");
	if (stop->data) {
		this->stop();
		robot_stopped = true;
		sleep(5.0);
		ROS_INFO("STOPPING THE ROBOT");
		robot_stopped = false;
		carry_bin_to(last_x,last_y,last_o);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_bin");
	usleep(1000*1000);

	MoveBin movebin;

	double x1 ,y1, o1, x2 ,y2, o2;
	/*
	std::cout<< "Start location ?" << std::endl;
	std::cout<< "x  :" << std::endl;
	std::cin >> x1;
	std::cout<< "y :" << std::endl;
	std::cin >> y1;
	std::cout<< "End location ?" << std::endl;
	std::cout<< "x  :" << std::endl;
	std::cin >> x2;
	std::cout<< "y :" << std::endl;
	std::cin >> y2;
	*/
	x1 = 2;
	y1 = 0.3;
	
	x2 = 0.5;
	y2 = 1.5;
	
	o1 = 0;
	o2 = 0;

	while(ros::ok()){
		ros::spinOnce();
		movebin.last_x = x1;
		movebin.last_y = y1;
		movebin.last_o = o1;
		if (!movebin.carry_bin_to(x1,y1,o1)){
			ROS_ERROR("Aborting !");
			continue;
		}
		ros::spinOnce();
		movebin.last_x = x2;
		movebin.last_y = y2;
		movebin.last_o = o2;
		if (!movebin.carry_bin_to(x2,y2,o2)){
			ROS_ERROR("Aborting !");
			continue;
		}
	}
	ros::shutdown();
	return 0;
}

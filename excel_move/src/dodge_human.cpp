#include "dodge_human.h"

/*--------------------------------------------------------------------
 * MoveBin()
 * Constructor.
 *------------------------------------------------------------------*/
MoveBin::MoveBin(ros::NodeHandle nh) : group("excel"), excel_ac("vel_pva_trajectory_ctrl/follow_joint_trajectory"), gripper_ac("gripper_controller/gripper_action", true) ,nh_(nh),spinner(1)
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
	sub = nh_.subscribe("human/safety/stop", 1, &MoveBin::stop_callback,this);
	human_pose_sub = nh_.subscribe("human/estimated/pose", 1, &MoveBin::human_pose_callback,this);
	human_unsafe = false;
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
	service_request.ik_request.pose_stamped.pose.position.z = 1.4;
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

	if (human_pose.position.y < 1.4){
		ROS_WARN("Avoiding the human");
		// Correct the rotation to avoid the human
		geometry_msgs::PoseStamped current_pose = group.getCurrentPose();
		service_response.solution.joint_state.position[1] = this->avoid_human(service_response.solution.joint_state.position[1],planning_scene.robot_state.joint_state.position[1], current_pose.pose, service_request.ik_request.pose_stamped.pose);
	}

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
double MoveBin::avoid_human(double goal_angle, double current_angle, geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose)
{
	double cur_x, cur_y, goal_x, goal_y;
	cur_x = current_pose.position.x; cur_y = current_pose.position.y;
	goal_x = goal_pose.position.x; goal_y = goal_pose.position.y;

	// Robot on the human table, goal is not
	if( ((cur_x <1.0)&(cur_y>1.0)) & !((goal_x <1.0)&(goal_y>1.0)) ){
		ROS_WARN("Going from A to B");
		if (goal_angle < current_angle){
			goal_angle += 2*M_PI;
		}	

	}else{
		// Robot on not the human table, but goal is
		if( !((cur_x <1.0)&(cur_y>1.0)) & ((goal_x <1.0)&(goal_y>1.0)) ){
			ROS_WARN("Going from B to A");
			if(goal_angle > current_angle){
				goal_angle -= 2*M_PI;
			}
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

	//while ( !is_successful ){
	while ( excel_ac.getState()== actionlib::SimpleClientGoalState::PENDING||excel_ac.getState()== actionlib::SimpleClientGoalState::ACTIVE||robot_stopped ){
		ros::spinOnce();
		ROS_INFO("In loop");
		if (human_unsafe) {
		  ROS_INFO("human unsafe");
			if (!robot_stopped){
				this->stop();
				robot_stopped = true;

			}
		}
		else if (robot_stopped){
		  ROS_INFO("Robot stopped");
			robot_stopped = false;
			carry_bin_to(last_x,last_y,last_o);
		}
		else{
		  ROS_INFO("Robot not stopped");
		  std::string state = excel_ac.getState().toString();
		  ROS_INFO("%s",state.c_str());
		  //is_successful = excel_ac.getState()== actionlib::SimpleClientGoalState::SUCCEEDED;//excel_ac.getResult()->error_code!=excel_ac.getResult()->SUCCESSFUL;
		  is_successful = excel_ac.getState()== actionlib::SimpleClientGoalState::SUCCEEDED;

			ROS_INFO("Trajectory %s completed", is_successful ? "":"not");
		}
		r.sleep();
	}
	//return is_successful;
	std::string state = excel_ac.getState().toString();
	ROS_INFO("Final state is : %s",state.c_str());
	
	return excel_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

void MoveBin::stop(){
	if(!sim){
		// Empty trajectory
		control_msgs::FollowJointTrajectoryGoal excel_goal;

		// Send goal and wait for a result
		//excel_ac.sendGoal(excel_goal);
		excel_ac.cancelGoal();
	}
	else 
		group.stop();
}

void MoveBin::stop_callback(const std_msgs::Bool::ConstPtr& stop)
{
	human_unsafe = stop->data;
}

void MoveBin::human_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_stamped)
{
  if(isnan(pose_stamped->pose.position.x)){
    geometry_msgs::Pose fake_pose;
    fake_pose.position.x = 1000;
    fake_pose.position.y = 1000;
    human_pose = fake_pose;
  }
  else human_pose = pose_stamped->pose;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_bin");
	ros::NodeHandle nh;
	usleep(1000*1000);

	MoveBin movebin(nh);

	double x1 ,y1, o1, x2 ,y2, o2;
	x1 = 0.5;
	y1 = 1.2;

	x2 = 2.5;
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

#include "scanning.h"

/*--------------------------------------------------------------------
 * Scanning()
 * Constructor.
 *------------------------------------------------------------------*/
Scanning::Scanning() : group("excel"), excel_ac("vel_pva_trajectory_ctrl/follow_joint_trajectory") ,spinner(1), scan_obj(nh_)
{
	spinner.start();
	cout << "Spinner started?" << endl;
	boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(ros::Duration(2.0)));
	planning_scene_monitor::PlanningSceneMonitorPtr plg_scn_mon(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf));
	planning_scene_monitor = plg_scn_mon;

	cout << "Before nh_param?" << endl;
	ros::NodeHandle nh_param_("~");

	cout << "Failed before param?" << endl;
	// nh_param_.getParam("sim",sim);
	sim = false;
	
	cout << "Failed at param?" << endl;

	ros::WallDuration sleep_t(0.5);
	group.setPlanningTime(8.0);
	group.allowReplanning(false);

	service_client = nh_.serviceClient<moveit_msgs::GetPositionIK> ("compute_ik");

	while(!service_client.exists())
	{
		ROS_INFO("Waiting for IK service");
		sleep(1.0);
	}
	fk_client = nh_.serviceClient<moveit_msgs::GetPositionFK> ("compute_fk");
	while(!fk_client.exists())
	{
		ROS_INFO("Waiting for FK service");
		sleep(1.0);
	}

	// Loading planning_scene_monitor //
	planning_scene_monitor->startSceneMonitor();
	planning_scene_monitor->startStateMonitor();
	planning_scene_monitor->startWorldGeometryMonitor();
	planning_scene_monitor->requestPlanningSceneState();
	full_planning_scene = planning_scene_monitor->getPlanningScene();
	full_planning_scene->getPlanningSceneMsg(planning_scene);

	// Define joint_constraints for the IK service //
	rail_constraint.joint_name = "table_rail_joint";
	rail_constraint.position = 2.0;
	rail_constraint.tolerance_above = 1.3;
	rail_constraint.tolerance_below = 1.6;
	rail_constraint.weight = 1;
	shoulder_constraint.joint_name = "shoulder_lift_joint";
	shoulder_constraint.position = -M_PI/2;
	shoulder_constraint.tolerance_above = M_PI/4;
	shoulder_constraint.tolerance_below = 0;
	shoulder_constraint.weight = 1;

	// Config the IK service //
	service_request.ik_request.group_name = "scanning";
	service_request.ik_request.pose_stamped.header.frame_id = "table_link";
	service_request.ik_request.avoid_collisions = true;
	service_request.ik_request.attempts = 30;
	service_request.ik_request.constraints.joint_constraints.push_back(rail_constraint);
	service_request.ik_request.constraints.joint_constraints.push_back(shoulder_constraint);

	// no try yet //
	orientation_try = 0;

	// start with orientation 0 //
	current_orientation = 0;
}

bool Scanning::scan(int pose, int orientation){
	if (pose==0){
		service_request.ik_request.pose_stamped.pose.position.x = 0.60;
		service_request.ik_request.pose_stamped.pose.position.y = 1.94;
		service_request.ik_request.pose_stamped.pose.position.z = 0.95;
	}
	if (pose==1){
		service_request.ik_request.pose_stamped.pose.position.x = 0.52;
		service_request.ik_request.pose_stamped.pose.position.y = 2.11;
		service_request.ik_request.pose_stamped.pose.position.z = 0.95;
	}
	if (pose==2){
		service_request.ik_request.pose_stamped.pose.position.x = 0.48;
		service_request.ik_request.pose_stamped.pose.position.y = 2.00;
		service_request.ik_request.pose_stamped.pose.position.z = 0.95;
	}

	string tag_name;
	switch(pose)
	  {
	  case 0: tag_name = "ASIF";
	    break;
	  case 1: tag_name = "idrive";
	    break;
	  case 2: tag_name = "park";
	    break;
	    
	  }

	tf::Quaternion quat;
	if (orientation==0){
		quat = tf::createQuaternionFromRPY(-M_PI/2,0.9,-M_PI/2);
	}
	if (orientation==1){
		quat = tf::createQuaternionFromRPY(-M_PI/2,0.9,-M_PI/2-M_PI/5);
	}
	if (orientation==2){
		quat = tf::createQuaternionFromRPY(-M_PI/2,0.9,-M_PI/2-M_PI/3);
	}
	service_request.ik_request.pose_stamped.pose.orientation.x = quat.x();
	service_request.ik_request.pose_stamped.pose.orientation.y = quat.y();
	service_request.ik_request.pose_stamped.pose.orientation.z = quat.z();
	service_request.ik_request.pose_stamped.pose.orientation.w = quat.w();

	service_client.call(service_request, service_response);
	if(service_response.error_code.val !=1){
		ROS_ERROR("IK couldn't find a solution for step 1");
	}

	planning_scene_monitor->requestPlanningSceneState();
	full_planning_scene = planning_scene_monitor->getPlanningScene();
	full_planning_scene->getPlanningSceneMsg(planning_scene);
	
	// Fixing shoulder_pan & wrist_3 given by the IK
	service_response.solution.joint_state.position[1] = this->optimal_goal_angle(service_response.solution.joint_state.position[1],planning_scene.robot_state.joint_state.position[1]);
	service_response.solution.joint_state.position[6] = this->optimal_goal_angle(service_response.solution.joint_state.position[6],planning_scene.robot_state.joint_state.position[6]);
	group.setStartState(full_planning_scene->getCurrentState());
	group.setJointValueTarget(service_response.solution.joint_state);

	if(group.plan(my_plan)){
		if(!sim){
			excel_ac.waitForServer();

			// Copy trajectory
			control_msgs::FollowJointTrajectoryGoal excel_goal;
			excel_goal.trajectory = my_plan.trajectory_.joint_trajectory;

			// Ask to execute now
			ros::Time time_zero(0.0);
			excel_goal.trajectory.header.stamp = time_zero; 

			// Specify path and goal tolerance
			//excel_goal.path_tolerance

			// Send goal and wait for a result
			excel_ac.sendGoal(excel_goal);				
		}
		else group.execute(my_plan);
	}
	else{
		ROS_ERROR("Motion planning failed");
	}
	
	sleep(1.0);
	bool ok;

	ok = scan_obj.find_tag(tag_name, 1.);

	// //TODO: Pass in the tag string and remove the following code
	// std::cout << "Do you see pose " << pose<< " at orientation "<< orientation<< " ?"<<std::endl;
	// std::cin >> ok;
	
	// if (ok==2) ros::shutdown();
	
	return ok;
}

/*--------------------------------------------------------------------
 * optimal_goal_angle()
 * Finds out if the robot needs to rotate clockwise or anti-clockwise
 *------------------------------------------------------------------*/
double Scanning::optimal_goal_angle(double goal_angle, double current_angle)
{
	while( std::abs(std::max(current_angle,goal_angle) - std::min(current_angle,goal_angle))>M_PI){
		if (goal_angle>current_angle) goal_angle -= 2*M_PI;
		else goal_angle += 2*M_PI;
	}
	if(goal_angle>2*M_PI) goal_angle -= 2*M_PI;
	if(goal_angle<-2*M_PI) goal_angle += 2*M_PI;
	return goal_angle;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "scanning");
	usleep(1000*1000);
	
	Scanning scanning;
	std::cout << "-----------------------------" << std::endl;
	std::cout << "USE THE NUMBER 2 TO SHUTDOWN" << std::endl;
	std::cout << "-----------------------------" << std::endl;
	while(ros::ok()){
		while(scanning.orientation_try<4 && ros::ok()){
			if(scanning.scan(0, scanning.current_orientation)) break;
			scanning.current_orientation = (scanning.current_orientation +1) % 3;
			scanning.orientation_try += 1;
		}
		scanning.orientation_try = 0;

		while(scanning.orientation_try<4 && ros::ok()){
			if(scanning.scan(1, scanning.current_orientation)) break;
			scanning.current_orientation = (scanning.current_orientation +1) % 3;
			scanning.orientation_try += 1;
		}
		scanning.orientation_try = 0;

		while(scanning.orientation_try<4 && ros::ok()){
			if(scanning.scan(2, scanning.current_orientation)) break;
			scanning.current_orientation = (scanning.current_orientation +1) % 3;
			scanning.orientation_try += 1;
		}
		scanning.orientation_try = 0;
	}

	ros::shutdown();
	return 0;
}

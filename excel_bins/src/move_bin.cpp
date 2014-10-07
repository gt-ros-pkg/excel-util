#include <excel_bins/move_bin.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

/*--------------------------------------------------------------------
 * MoveBin()
 * Constructor.
 *------------------------------------------------------------------*/
MoveBin::MoveBin() : 
  group("excel"), excel_ac("vel_pva_trajectory_ctrl/follow_joint_trajectory"), 
  gripper_ac("gripper_controller/gripper_action", true) ,spinner(1)
{
  spinner.start();
  boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(ros::Duration(2.0)));
  planning_scene_monitor::PlanningSceneMonitorPtr plg_scn_mon(
      new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf));
  planning_scene_monitor = plg_scn_mon;

  ros::NodeHandle nh_, nh_param_("~");
  sim = false;
  use_gripper = true;
  vertical_check_safety_ = false;
  traverse_check_safety_ = true;
  nh_param_.getParam("sim", sim);
  nh_param_.getParam("use_gripper", use_gripper);
  nh_param_.getParam("vertical_check_safety", vertical_check_safety_);
  nh_param_.getParam("traverse_check_safety", traverse_check_safety_);

  human_unsafe_ = false;
	hum_unsafe_sub_ = nh_.subscribe("human/safety/stop", 1, &MoveBin::humanUnsafeCallback,this);

  ros::WallDuration sleep_t(0.5);
  group.setPlanningTime(8.0);
  group.allowReplanning(false);
  group.startStateMonitor(1.0);

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

  rail_max = 3.2;
  rail_min = 0.41;
  rail_tolerance = 0.3;

  if(!sim) {
    ROS_INFO("Waiting for action server to be available...");
    excel_ac.waitForServer();
    ROS_INFO("Action server found.");
  }
}

void MoveBin::humanUnsafeCallback(const std_msgs::Bool::ConstPtr& msg)
{
  human_unsafe_ = msg->data;
}

bool MoveBin::moveToHome()
{
  while (ros::ok()) {
    // Plan trajectory
    group.setStartStateToCurrentState();
    static const double arr[] = {2.267, 2.477, -1.186, 1.134, -1.062, -1.059, -3.927};
    std::vector<double> joint_vals(arr, arr + sizeof(arr) / sizeof(arr[0]));
    group.setJointValueTarget(joint_vals);
    int num_tries = 4;
    MoveGroupPlan my_plan;
    // try to plan a few times, just to be safe
    while (ros::ok() && num_tries > 0) {
      if (group.plan(my_plan))
        break;
      num_tries--;
    }

    if (num_tries > 0) {
      // found plan, let's try and execute
      if (executeJointTrajectory(my_plan, true)) {
        ROS_INFO("Home position joint trajectory execution successful");
        return true;
      }
      else {
        ROS_WARN("Home position joint trajectory execution failed");
        ros::Duration(0.5).sleep();
        continue;
      }
    }
    else {
      ROS_ERROR("Home position Motion planning failed");
      continue;
    }
  }
  return true;
}

bool MoveBin::moveBinToTarget(int bin_number, double x_target, double y_target, double angle_target)
{
  ROS_INFO("Moving bin %d to target (%.3f, %.3f, %f)", bin_number, x_target, y_target, angle_target);
  if(bin_number == -5) {
    return moveToHome();
  }
  double bin_height;
  executeGripperAction(false, false); // open gripper, but don't wait
  if(!approachBin(bin_number, bin_height)) {
    ROS_ERROR("Failed to approach bin #%d.", bin_number);
    return false;
  }
  if(!attachBin(bin_number)) {
    ROS_ERROR("Failed to attach bin #%d.", bin_number);
    return false;
  }
  ///////////////////////////// HOLDING BIN ///////////////////////////////////
  if(!deliverBin(x_target, y_target, angle_target, bin_height)) {
    ROS_ERROR("Failed to deliver bin to target (%.3f, %.3f, %f)", 
                                        x_target, y_target, angle_target);
    return false;
  }
  if(!detachBin()) {
    ROS_ERROR("Failed to detach bin.");
    return false;
  }
  /////////////////////////////////////////////////////////////////////////////
  if(!ascent(bin_height)) {
    ROS_ERROR("Failed to ascend after releasing bin.");
    return false;
  }
  return true;
}

bool MoveBin::approachBin(int bin_number, double& bin_height)
{
  ROS_INFO("Approaching bin %d", bin_number);
  if(!moveAboveBin(bin_number, bin_height)) {
    ROS_ERROR("Failed to move above bin #%d.", bin_number);
    return false;
  }
  if(!descent(bin_height)) {
    ROS_ERROR("Failed to descend after moving above bin.");
    return false;
  }
  return true;
}

bool MoveBin::deliverBin(double x_target, double y_target, double angle_target, double bin_height)
{
  ROS_INFO("Delivering to target (%.3f, %.3f, %f)", x_target, y_target, angle_target);
  if(!ascent(bin_height)) {
    ROS_ERROR("Failed to ascend while grasping bin.");
    return false;
  }
  if(!carryBinTo(x_target, y_target, angle_target, bin_height)) {
    ROS_ERROR("Failed to carry bin to target (%.3f, %.3f, %.3f)", 
                                        x_target, y_target, angle_target);
    return false;
  }
  if(!descent(bin_height)) {
    ROS_ERROR("Failed to descend after moving bin above target place.");
    return false;
  }
  return true;
}

bool MoveBin::moveAboveBin(int bin_number, double& bin_height)
{
  ROS_INFO("Moving above bin %d", bin_number);
  moveit_msgs::CollisionObjectPtr bin_coll_obj = getBinCollisionObject(bin_number);
  if (!bin_coll_obj) {
    // bin not found
    ROS_ERROR("BIN NOT FOUND");
    return false;
  }
  
  geometry_msgs::Pose target_pose;
  getBinAbovePose(bin_coll_obj, target_pose, bin_height);

  return traverseMove(target_pose);
}

bool MoveBin::traverseMove(geometry_msgs::Pose& pose)
{
  ROS_INFO("Traverse move to position (%.2f, %.2f, %.2f)", 
                        pose.position.x, pose.position.y, pose.position.z);
  while (ros::ok()) {
    // update planning scene
    moveit_msgs::PlanningScene planning_scene;
    planning_scene::PlanningScenePtr full_planning_scene;
    getPlanningScene(planning_scene, full_planning_scene);

    ////////////// Perform IK to find joint goal //////////////
    moveit_msgs::GetPositionIK::Request ik_srv_req;

    // setup IK request
    ik_srv_req.ik_request.group_name = "excel";
    ik_srv_req.ik_request.pose_stamped.header.frame_id = "table_link";
    ik_srv_req.ik_request.pose_stamped.header.stamp = ros::Time::now();
    ik_srv_req.ik_request.avoid_collisions = true;
    ik_srv_req.ik_request.attempts = 30;

    // set pose
    ik_srv_req.ik_request.pose_stamped.pose = pose;

    // set joint constraints
    double rail_center = pose.position.x;
    moveit_msgs::JointConstraint special_rail_constraint;
    special_rail_constraint.joint_name = "table_rail_joint";
    special_rail_constraint.position = rail_max - rail_center;
    special_rail_constraint.tolerance_above = std::max(
        std::min(rail_max - rail_center + rail_tolerance, rail_max) - 
                 (rail_max - rail_center), 0.0);
    special_rail_constraint.tolerance_below = 
      std::max((rail_max - rail_center) - 
               std::max(rail_max - rail_center - rail_tolerance, rail_min), 0.0);
    special_rail_constraint.weight = 1;
    ROS_INFO("Special rail constraint: %.3f (+%.3f, -%.3f)", special_rail_constraint.position, 
             special_rail_constraint.tolerance_above, special_rail_constraint.tolerance_below);
    ik_srv_req.ik_request.constraints.joint_constraints.push_back(special_rail_constraint);
    ik_srv_req.ik_request.constraints.joint_constraints.push_back(shoulder_constraint);
    //ik_srv_req.ik_request.constraints.joint_constraints.push_back(elbow_constraint);

    // call IK server
    ROS_INFO("Calling IK for pose pos = (%.2f, %.2f, %.2f), quat = (%.2f, %.2f, %.2f, w %.2f)",
        ik_srv_req.ik_request.pose_stamped.pose.position.x,
        ik_srv_req.ik_request.pose_stamped.pose.position.y,
        ik_srv_req.ik_request.pose_stamped.pose.position.z,
        ik_srv_req.ik_request.pose_stamped.pose.orientation.x,
        ik_srv_req.ik_request.pose_stamped.pose.orientation.y,
        ik_srv_req.ik_request.pose_stamped.pose.orientation.z,
        ik_srv_req.ik_request.pose_stamped.pose.orientation.w);
    moveit_msgs::GetPositionIK::Response ik_srv_resp;
    service_client.call(ik_srv_req, ik_srv_resp);
    if(ik_srv_resp.error_code.val !=1){
      ROS_ERROR("IK couldn't find a solution (error code %d)", ik_srv_resp.error_code.val);
      return false;
    }
    ROS_INFO("IK returned succesfully");
    ///////////////////////////////////////////////////////////

    // Fixing shoulder_pan and wrist_3 given by the IK
    ik_srv_resp.solution.joint_state.position[1] = 
      this->optimalGoalAngle(ik_srv_resp.solution.joint_state.position[1], 
                               planning_scene.robot_state.joint_state.position[1]);
    ik_srv_resp.solution.joint_state.position[6] = 
      this->optimalGoalAngle(ik_srv_resp.solution.joint_state.position[6],
                               planning_scene.robot_state.joint_state.position[6]);

    // Plan trajectory
    group.setStartStateToCurrentState();
    group.setJointValueTarget(ik_srv_resp.solution.joint_state);
    int num_tries = 4;
    MoveGroupPlan my_plan;
    // try to plan a few times, just to be safe
    while (ros::ok() && num_tries > 0) {
      if (group.plan(my_plan))
        break;
      num_tries--;
    }

    if (num_tries > 0) {
      // found plan, let's try and execute
      if (executeJointTrajectory(my_plan, traverse_check_safety_)) {
        ROS_INFO("Traverse joint trajectory execution successful");
        return true;
      }
      else {
        ROS_WARN("Traverse joint trajectory execution failed, going to restart");
        ros::Duration(0.5).sleep();
        continue;
      }
    }
    else {
      ROS_ERROR("Motion planning failed");
      return false;
    }
  }
}

bool MoveBin::ascent(double bin_height)
{
  ROS_INFO("Ascending");
  return verticalMove(TABLE_HEIGHT + GRIPPING_OFFSET + bin_height + DZ);
}

bool MoveBin::descent(double bin_height)
{
  ROS_INFO("Descending");
  return verticalMove(TABLE_HEIGHT + GRIPPING_OFFSET + bin_height);
}

bool MoveBin::verticalMove(double target_z)
{
  ROS_INFO("Vertical move to target z: %f", target_z);

  while (ros::ok()) {
    // update the planning scene to get the robot's state
    moveit_msgs::PlanningScene planning_scene;
    planning_scene::PlanningScenePtr full_planning_scene;
    getPlanningScene(planning_scene, full_planning_scene);

    ////////////// Perform FK to find end effector pose ////////////
    /*
    moveit_msgs::GetPositionFK::Request fk_request;
    moveit_msgs::GetPositionFK::Response fk_response;
    fk_request.header.frame_id = "table_link";
    fk_request.fk_link_names.push_back("ee_link");
    fk_request.robot_state = planning_scene.robot_state;
    fk_client.call(fk_request, fk_response);
    */
    ////////////////////////////////////////////////////////////////

#if 0
    ////////////// Perform IK to find joint goal //////////////
    moveit_msgs::GetPositionIK::Request ik_srv_req;

    // setup IK request
    ik_srv_req.ik_request.group_name = "excel";
    ik_srv_req.ik_request.pose_stamped.header.frame_id = "table_link";
    ik_srv_req.ik_request.avoid_collisions = false;
    ik_srv_req.ik_request.attempts = 100;

    // the target pose is the current location with a different z position
    ik_srv_req.ik_request.pose_stamped = group.getCurrentPose();
    // ik_srv_req.ik_request.pose_stamped = fk_response.pose_stamped[0];
    ik_srv_req.ik_request.pose_stamped.pose.position.z = target_z;

    ik_srv_req.ik_request.constraints.joint_constraints.clear();
    // ik_srv_req.ik_request.constraints.joint_constraints.push_back(shoulder_constraint);
    //ik_srv_req.ik_request.constraints.joint_constraints.push_back(elbow_constraint);
    moveit_msgs::JointConstraint rail_fixed_constraint, shoulder_pan_fixed_constraint,
                                 wrist_3_fixed_constraint;
    rail_fixed_constraint.joint_name = "table_rail_joint";
    shoulder_pan_fixed_constraint.joint_name = "shoulder_pan_joint";
    wrist_3_fixed_constraint.joint_name = "wrist_3_joint";
    const double *rail_current_pose = 
      full_planning_scene->getCurrentState().getJointPositions("table_rail_joint");
    const double *shoulder_pan_current_pose = 
      full_planning_scene->getCurrentState().getJointPositions("shoulder_pan_joint");
    const double *wrist_3_current_pose = 
      full_planning_scene->getCurrentState().getJointPositions("wrist_3_joint");
    rail_fixed_constraint.position = *rail_current_pose;
    shoulder_pan_fixed_constraint.position = *shoulder_pan_current_pose;
    wrist_3_fixed_constraint.position = *wrist_3_current_pose;

    rail_fixed_constraint.tolerance_above = 0.2;
    rail_fixed_constraint.tolerance_below = 0.2;
    rail_fixed_constraint.weight = 1;
    shoulder_pan_fixed_constraint.tolerance_above = 0.2;
    shoulder_pan_fixed_constraint.tolerance_below = 0.2;
    shoulder_pan_fixed_constraint.weight = 1;
    wrist_3_fixed_constraint.tolerance_above = 0.2;
    wrist_3_fixed_constraint.tolerance_below = 0.2;
    wrist_3_fixed_constraint.weight = 1;
    ik_srv_req.ik_request.constraints.joint_constraints.push_back(rail_fixed_constraint);
    ik_srv_req.ik_request.constraints.joint_constraints.push_back(shoulder_pan_fixed_constraint);
    ik_srv_req.ik_request.constraints.joint_constraints.push_back(wrist_3_fixed_constraint);

    ROS_INFO("Calling IK for pose pos = (%.2f, %.2f, %.2f), quat = (%.2f, %.2f, %.2f, w %.2f)",
        ik_srv_req.ik_request.pose_stamped.pose.position.x,
        ik_srv_req.ik_request.pose_stamped.pose.position.y,
        ik_srv_req.ik_request.pose_stamped.pose.position.z,
        ik_srv_req.ik_request.pose_stamped.pose.orientation.x,
        ik_srv_req.ik_request.pose_stamped.pose.orientation.y,
        ik_srv_req.ik_request.pose_stamped.pose.orientation.z,
        ik_srv_req.ik_request.pose_stamped.pose.orientation.w);
    moveit_msgs::GetPositionIK::Response ik_srv_resp;
    service_client.call(ik_srv_req, ik_srv_resp);
    if(ik_srv_resp.error_code.val !=1){
      ROS_ERROR("IK couldn't find a solution (error code %d)", ik_srv_resp.error_code.val);
      return 0;
    }
    ROS_INFO("IK returned succesfully");

    // Fixing wrist_3 given by the IK
    ik_srv_resp.solution.joint_state.position[1] = 
      this->optimalGoalAngle(ik_srv_resp.solution.joint_state.position[1], 
                               planning_scene.robot_state.joint_state.position[1]);
    ik_srv_resp.solution.joint_state.position[6] = this->optimalGoalAngle(ik_srv_resp.solution.joint_state.position[6],planning_scene.robot_state.joint_state.position[6]);

    group.setJointValueTarget(ik_srv_resp.solution.joint_state);
#endif
    group.setStartStateToCurrentState();
    /*
    int num_tries = 4;
    MoveGroupPlan my_plan;
    while(ros::ok() && num_tries > 0) {
      if(group.plan(my_plan))
        return executeJointTrajectory(my_plan);
      num_tries--;
    }
    */
    // ROS_WARN("Motion planning failed");

    geometry_msgs::Pose pose = group.getCurrentPose().pose;
    pose.position.z = target_z;

    ROS_INFO("Calling cart path for pose pos = (%.2f, %.2f, %.2f), quat = (%.2f, %.2f, %.2f, w %.2f)",
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
    // find linear trajectory
    moveit_msgs::RobotTrajectory lin_traj_msg;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(pose);
    double fraction = group.computeCartesianPath(waypoints, 0.07, 0.0, lin_traj_msg, false);

    // create new robot trajectory object
    robot_trajectory::RobotTrajectory lin_rob_traj(group.getCurrentState()->getRobotModel(), "excel");
    // copy the trajectory message into the robot trajectory object
    lin_rob_traj.setRobotTrajectoryMsg(*group.getCurrentState(), lin_traj_msg);

    trajectory_processing::IterativeParabolicTimeParameterization iter_parab_traj_proc;
    if(!iter_parab_traj_proc.computeTimeStamps(lin_rob_traj)) {
      ROS_ERROR("Failed smoothing trajectory");
      return false;
    }
    // put the smoothed trajectory back into the message....
    lin_rob_traj.getRobotTrajectoryMsg(lin_traj_msg);

    MoveGroupPlan lin_traj_plan;
    lin_traj_plan.trajectory_ = lin_traj_msg;
    ROS_INFO("computeCartesianPath fraction = %f", fraction);
    if(fraction < 0.0) {
      ROS_ERROR("Failed computeCartesianPath");
      return false;
    }

    if (executeJointTrajectory(lin_traj_plan, vertical_check_safety_)) {
      ROS_INFO("Vertical joint trajectory execution successful");
      return true;
    }
    else {
      ROS_WARN("Vertical joint trajectory execution failed, going to restart");
      continue;
    }
  }
}

bool MoveBin::attachBin(int bin_number)
{	
  ROS_INFO("Attaching bin %d", bin_number);
  // close gripper
  executeGripperAction(true, true); 

  moveit_msgs::CollisionObjectPtr bin_coll_obj = getBinCollisionObject(bin_number);

  if (bin_coll_obj) {
    ROS_INFO("Attaching the bin");
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "wrist_3_link";
    attached_object.object = *bin_coll_obj;
    attached_object.object.operation = attached_object.object.ADD;
    attached_object_publisher.publish(attached_object);
    return true;
  } else {
    // std::string error_msg = ""+bin_name + " is not in the scene. Aborting !";
    ROS_ERROR("This bin is not in the scene.");
    return false;
  }
}

bool MoveBin::detachBin()
{
  ROS_INFO("Detaching bin");
  // open gripper
  executeGripperAction(false, true);

  // update the planning scene to get the robot's state
  moveit_msgs::PlanningScene planning_scene;
  planning_scene::PlanningScenePtr full_planning_scene;
  getPlanningScene(planning_scene, full_planning_scene);

  if (planning_scene.robot_state.attached_collision_objects.size()>0){
    moveit_msgs::AttachedCollisionObject attached_object = planning_scene.robot_state.attached_collision_objects[0];
    moveit_msgs::GetPositionFK::Request fk_request;
    moveit_msgs::GetPositionFK::Response fk_response;
    fk_request.header.frame_id = "table_link";
    fk_request.fk_link_names.clear();
    fk_request.fk_link_names.push_back("wrist_3_link");
    fk_request.robot_state = planning_scene.robot_state;
    fk_client.call(fk_request, fk_response);

    tf::Quaternion co_quat;
    quaternionMsgToTF(fk_response.pose_stamped[0].pose.orientation, co_quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(co_quat).getRPY(roll, pitch, yaw);
    ROS_INFO_STREAM(roll);
    ROS_INFO_STREAM(pitch);
    ROS_INFO_STREAM(yaw);
    tf::Quaternion quat = tf::createQuaternionFromRPY(0,0,yaw);

    attached_object.object.header.frame_id = "table_link";
    attached_object.object.mesh_poses[0].position = fk_response.pose_stamped[0].pose.position;
    attached_object.object.mesh_poses[0].position.z = TABLE_HEIGHT;
    attached_object.object.mesh_poses[0].orientation.x = quat.x();
    attached_object.object.mesh_poses[0].orientation.y = quat.y();
    attached_object.object.mesh_poses[0].orientation.z = quat.z();
    attached_object.object.mesh_poses[0].orientation.w = quat.w();

    planning_scene.robot_state.attached_collision_objects.clear();
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene_diff_publisher.publish(planning_scene);
    return 1;
  }else{
    ROS_ERROR("There was no bin attached to the robot");
    return 0;
  }
}

/*--------------------------------------------------------------------
 * optimalGoalAngle()
 * Finds out if the robot needs to rotate clockwise or anti-clockwise
 *------------------------------------------------------------------*/
double MoveBin::optimalGoalAngle(double goal_angle, double current_angle)
{
  //std::cout<< "Current angle is : "<<current_angle<<std::endl;
  //std::cout<< "Goal angle is : "<<goal_angle<<std::endl;


  while( std::abs(std::max(current_angle,goal_angle) - std::min(current_angle,goal_angle))>M_PI){
    //std::cout<<"This is not the shortest path"<<std::endl;
    if (goal_angle>current_angle){
      goal_angle -= 2*M_PI;
    }
    else{
      goal_angle += 2*M_PI;
    }

  }

  if(goal_angle>2*M_PI){
    //std::cout<<"Your goal_angle would be too high"<<std::endl<<"Sorry, going the other way"<<std::endl;
    goal_angle -= 2*M_PI;
  }
  if(goal_angle<-2*M_PI){
    //std::cout<<"Your goal_angle would be too small"<<std::endl<<"Sorry, going the other way"<<std::endl;
    goal_angle += 2*M_PI;
  }
  //std::cout<<"Final angle is : "<< goal_angle<< std::endl;
  return goal_angle;
}

bool MoveBin::executeJointTrajectory(MoveGroupPlan& mg_plan, bool check_safety)
{
  int num_pts = mg_plan.trajectory_.joint_trajectory.points.size();
  ROS_INFO("Executing joint trajectory with %d knots and duration %f", num_pts, 
           mg_plan.trajectory_.joint_trajectory.points[num_pts-1].time_from_start.toSec());
  if(sim)
    return group.execute(mg_plan);

  // Copy trajectory
  control_msgs::FollowJointTrajectoryGoal excel_goal;
  excel_goal.trajectory = mg_plan.trajectory_.joint_trajectory;

  // Ask to execute now
  excel_goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(0.15); 

  // Specify path and goal tolerance
  //excel_goal.path_tolerance

  while (check_safety && human_unsafe_ && ros::ok()) {
    ROS_WARN_THROTTLE(1.0, "Human unsafe condition detected. Waiting for this to clear.");
    ros::Duration(1.0/30.0).sleep();
  }
  // Send goal and wait for a result
  excel_ac.sendGoal(excel_goal);
  while (ros::ok()) {
    if (excel_ac.waitForResult(ros::Duration(1.0/30.0))) 
      break;
    if (check_safety && human_unsafe_) {
      ROS_WARN("Human unsafe condition detected. Stopping trajectory");
      stopJointTrajectory();
      ros::Duration(0.5).sleep();
      return false;
    }
  }
  actionlib::SimpleClientGoalState end_state = excel_ac.getState();
  return end_state == actionlib::SimpleClientGoalState::SUCCEEDED;
}

void MoveBin::stopJointTrajectory()
{
  if(sim)
		group.stop();

  ROS_INFO("Stopping joint trajectory");
  excel_ac.cancelGoal();
}

bool MoveBin::executeGripperAction(bool is_close, bool wait_for_result)
{
  if(is_close)
    ROS_INFO("Closing gripper");
  else
    ROS_INFO("Opening gripper");
  if(use_gripper) {
    // send a goal to the action
    control_msgs::GripperCommandGoal goal;
    goal.command.position = (is_close) ? 0.0 : 0.08;
    goal.command.max_effort = 100;
    gripper_ac.sendGoal(goal);
    if(wait_for_result)
      return gripper_ac.waitForResult(ros::Duration(30.0));
    else
      return true;
  }
  else {
    ros::Duration(2.0).sleep();
    return true;
  }
}

void MoveBin::getPlanningScene(moveit_msgs::PlanningScene& planning_scene, 
                               planning_scene::PlanningScenePtr& full_planning_scene)
{
  planning_scene_monitor->requestPlanningSceneState();
  full_planning_scene = planning_scene_monitor->getPlanningScene();
  full_planning_scene->getPlanningSceneMsg(planning_scene);
}

moveit_msgs::CollisionObjectPtr MoveBin::getBinCollisionObject(int bin_number)
{
  std::string bin_name = "bin#" + boost::lexical_cast<std::string>(bin_number); 

  // update the planning scene to get the robot's state
  moveit_msgs::PlanningScene planning_scene;
  planning_scene::PlanningScenePtr full_planning_scene;
  getPlanningScene(planning_scene, full_planning_scene);

  for(int i=0;i<planning_scene.world.collision_objects.size();i++) 
    if(planning_scene.world.collision_objects[i].id == bin_name) 
      return moveit_msgs::CollisionObjectPtr(
          new moveit_msgs::CollisionObject(planning_scene.world.collision_objects[i]));
  return moveit_msgs::CollisionObjectPtr();
}

void MoveBin::getBinAbovePose(moveit_msgs::CollisionObjectPtr bin_coll_obj, geometry_msgs::Pose& pose, 
                              double& bin_height)
{
  pose = bin_coll_obj->mesh_poses[0];

  // fix height
  bin_height = bin_coll_obj->meshes[0].vertices[0].z;
  pose.position.z = TABLE_HEIGHT+GRIPPING_OFFSET+bin_height+DZ;

  // fix orientation
  tf::Quaternion co_quat(pose.orientation.x, pose.orientation.y, 
                         pose.orientation.z, pose.orientation.w);
  tf::Matrix3x3 m(co_quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  tf::Quaternion quat = tf::createQuaternionFromRPY(M_PI/2-yaw,M_PI/2,M_PI);
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
}

/*--------------------------------------------------------------------
 * Moves to target location keeping the grasping orientation
 *------------------------------------------------------------------*/
bool MoveBin::carryBinTo(double x_target, double y_target, double angle_target, double bin_height)
{
  ROS_INFO("Carrying bin to target (%.3f, %.3f, %f)", x_target, y_target, angle_target);
  geometry_msgs::Pose target_pose;
  getCarryBinPose(x_target, y_target, angle_target, bin_height, target_pose);
  return traverseMove(target_pose);
}

void MoveBin::getCarryBinPose(double x_target, double y_target, double angle_target, double bin_height,
                              geometry_msgs::Pose& pose)
{
  tf::Quaternion quat_goal = tf::createQuaternionFromRPY(M_PI/2-angle_target*M_PI/180.0, M_PI/2, M_PI);
  pose.position.x = x_target;
  pose.position.y = y_target;
  pose.position.z = TABLE_HEIGHT + GRIPPING_OFFSET + bin_height + DZ;
  pose.orientation.x = quat_goal.x();
  pose.orientation.y = quat_goal.y();
  pose.orientation.z = quat_goal.z();
  pose.orientation.w = quat_goal.w();
}

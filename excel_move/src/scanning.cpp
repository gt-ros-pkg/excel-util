#include <excel_move/scanning.h>
/*--------------------------------------------------------------------
* Scanning()
* Constructor.
*------------------------------------------------------------------*/
using namespace std;
#define BAD_TAG -2
#define MISS_TAG -1
#define ALL_GOOD 0
Scanning::Scanning(ros::NodeHandle nh_) : group("excel"), excel_ac("vel_pva_trajectory_ctrl/follow_joint_trajectory") ,spinner(1), scan_obj(nh_)
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

    scan_parts_pub = nh_.advertise<std_msgs::Int8>("display/scanned_parts",1);

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
    moveit_msgs::JointConstraint rail_constraint, shoulder_constraint,elbow_constraint, shoulder_pan_constraint;
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
    shoulder_pan_constraint.joint_name = "shoulder_pan_joint";
    shoulder_pan_constraint.position = M_PI/2;
    shoulder_pan_constraint.tolerance_above = M_PI/2;
    shoulder_pan_constraint.tolerance_below = M_PI/2;
    shoulder_pan_constraint.weight = 1;


    // Config the IK service //
    service_request.ik_request.group_name = "scanning";
    service_request.ik_request.pose_stamped.header.frame_id = "table_link";
    service_request.ik_request.avoid_collisions = true;
    service_request.ik_request.attempts = 30;
    service_request.ik_request.constraints.joint_constraints.push_back(rail_constraint);
    service_request.ik_request.constraints.joint_constraints.push_back(shoulder_constraint);
    service_request.ik_request.constraints.joint_constraints.push_back(shoulder_pan_constraint);

    // no try yet //
    orientation_try = 0;

    // start with orientation 0 //
    current_orientation = 0;

}
bool Scanning::move_robot(int pose, int orientation)
{
//*
if (pose==0){
/*
service_request.ik_request.pose_stamped.pose.position.x = 0.60;
service_request.ik_request.pose_stamped.pose.position.y = 1.94;
service_request.ik_request.pose_stamped.pose.position.z = 0.95;
*/
service_request.ik_request.pose_stamped.pose.position.x = 0.56;
service_request.ik_request.pose_stamped.pose.position.y = 2.08;
service_request.ik_request.pose_stamped.pose.position.z = 0.95;
}
if (pose==1){
/*
service_request.ik_request.pose_stamped.pose.position.x = 0.52;
service_request.ik_request.pose_stamped.pose.position.y = 2.11;
service_request.ik_request.pose_stamped.pose.position.z = 0.95;
*/
service_request.ik_request.pose_stamped.pose.position.x = 0.56;
service_request.ik_request.pose_stamped.pose.position.y = 1.90;
service_request.ik_request.pose_stamped.pose.position.z = 0.95;
}
if (pose==2){
/*
service_request.ik_request.pose_stamped.pose.position.x = 0.48;
service_request.ik_request.pose_stamped.pose.position.y = 2.00;
service_request.ik_request.pose_stamped.pose.position.z = 0.95;
*/
service_request.ik_request.pose_stamped.pose.position.x = 0.62;
service_request.ik_request.pose_stamped.pose.position.y = 2.00;
service_request.ik_request.pose_stamped.pose.position.z = 0.95;
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
sleep(2);
return excel_ac.waitForResult(ros::Duration(15.));
}
else group.execute(my_plan);
return true;
}
else{
ROS_ERROR("Motion planning failed");
return false;
}
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

// in case no bad part, good tags returns the list of parts with 0s for the 
// ones not found
int Scanning::scan_it(vector<string> &good_tags, const vector<string> &bad_tags)
{
    vector<string> all_tags;
    all_tags.reserve(good_tags.size()+ bad_tags.size());
    all_tags.insert(all_tags.end(), good_tags.begin(), good_tags.end());
    all_tags.insert(all_tags.end(), bad_tags.begin(), bad_tags.end());

    string cur_str;
    int cur_pose;
    int cur_orientation = 0;
    int orientation_tries = 3;
    vector<bool> glob_oks, oks;
    vector<string> bad_tags_found;

    vector<string> scanned_numbers;

    glob_oks.resize(all_tags.size());
    fill(glob_oks.begin(), glob_oks.end(), false);

    for(int o=0;o<orientation_tries;o++){
        cur_orientation = o;
        for(int i=0; i<good_tags.size(); ++i){
            //we might have already found this tag while looking for another
            if (!glob_oks[i]){
                //debug

                cur_str = good_tags[i];

                cout << "Current Tag = " << cur_str << endl;

                if (cur_str == "111" || cur_str == "333" )
                    cur_pose = 0;
                else if(cur_str == "555" || cur_str == "777" )
                    cur_pose = 1;
                else if(cur_str == "999")
                    cur_pose = 2;


                //move the robot to try seeing the tag at cur_pose
                bool ok = move_robot(cur_pose, cur_orientation);

                cout << "Did you move the robt? " << ok << endl;

                //look for all the tags and return if they were found
                vector<string> not_found;
                for (int t=0; t<all_tags.size(); t++){
                    if(!glob_oks[t])
                        not_found.push_back(all_tags[t]);
                }

                oks = scan_obj.find_tag(not_found);
		
		for(int gi=0; gi<glob_oks.size(); gi++){
		  if(oks[gi]){
		    std_msgs::Int8 msg;
		    msg.data = (boost::lexical_cast<int>(good_tags[gi])/100);
		    scan_parts_pub.publish(msg);
		  }
		}

                cout << "Find returns?" << endl;

                //debug
                //not found
                //oks
                cout << "***************NOT FOUNDS***************" << endl;
                for(int j=0; j<not_found.size(); j++){
                    cout << not_found[j] << "-" << oks[j] << endl;
                }

                //update the global status for looking the tags
                for(int j=0, t=0;j<all_tags.size() && t<not_found.size();j++){
                    if(!glob_oks[j]){
                        glob_oks[j] = glob_oks[j] || oks[t];
                        if (oks[t])
                            cout << "Saw Tag = " << not_found[t] << endl;
                        t++;
                    }
                }

                //globals
                cout << "***************GLOBALS***************" << endl;
                for(int j=0; j<all_tags.size(); j++){
                    cout << all_tags[j] << "-" << glob_oks[j] << endl;
                }

                cout <<"was pose "<< cur_pose<< " and orientation "<<cur_orientation <<endl;

                //check for bad tags found
                for(int j=good_tags.size(); j<all_tags.size(); ++j){
                    if (oks[j]){ //returns the bad tag found
                        bad_tags_found.push_back(all_tags[j]);
                    }
                }
                if(bad_tags_found.size()>0){
                    ROS_ERROR("Found bad parts");
                    return BAD_TAG;
                }

                //if we find the tag we wanted in first place we go seek another
                //if(glob_oks[i]) break;
                //else we change the orientation to keep looking for this one
                //else cur_orientation = (cur_orientation + 1) % 3;
            }
        }
    }


    //Add the tags found to the list of good tags...
    scanned_numbers.resize(good_tags.size());
    for(int p = 0; p<scanned_numbers.size(); p++){
      scanned_numbers[p] = boost::lexical_cast<string>(glob_oks[p]*
				 (boost::lexical_cast<int>(good_tags[p])/100));
    }
    good_tags = scanned_numbers; //return the tags found and those missed are zeros
    //publish scanned parts
    //scanned_parts_pub.publish(scanned_numbers);

    for(int i=0; i<good_tags.size(); ++i){
        //if any of the good tags not found
        if (!glob_oks[i])
            return MISS_TAG;
    }
    return ALL_GOOD;
}

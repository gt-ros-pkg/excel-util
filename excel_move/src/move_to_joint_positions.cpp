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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_to_joint_positions_node");
    ros::NodeHandle nh_param_("~");
    bool sim;
    nh_param_.getParam("sim",sim);

	std::string pkg_path = ros::package::getPath("excel_move");
	std::string file_path = pkg_path + "/src/joint_values.yaml";
	std::ifstream fin(file_path.c_str());
	YAML::Parser parser(fin);
	YAML::Node doc;

	ros::AsyncSpinner spinner(1);
	spinner.start();
	usleep(1000*1000);

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> excel_ac("vel_pva_trajectory_ctrl/follow_joint_trajectory");
	move_group_interface::MoveGroup group("excel");
	move_group_interface::MoveGroup::Plan my_plan;

	parser.GetNextDocument(doc);
	std::vector< std::vector<double> > joints_list;
	for(unsigned i=0;i<doc.size();i++) {
		std::vector<double> joints;
		doc[i]["values"] >> joints;
		joints_list.push_back(joints);
	}
	std::cout <<"list size:"<< joints_list.size()<< std::endl;

	for(int i=0; i<joints_list.size(); i++){
		int num_tries = 4;
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

        //get info from the cameras
		sleep(3.0);
	}

	ros::shutdown();
	return 0;
}

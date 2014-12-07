#include <ros/ros.h>
#include <excel_bins/move_bin.h>
#include <geometry_msgs/PoseArray.h>

geometry_msgs::Pose human_pose;

void human_pose_callback(const geometry_msgs::PoseArray::ConstPtr& poses){
  if(!(isnan(poses->poses[0].position.x)) && !isnan(poses->poses[0].position.y) ){
    human_pose.position.x = poses->poses[0].position.x;
    human_pose.position.y = poses->poses[0].position.y;
  } 
  ROS_INFO_STREAM("human_pose changed x:"<<human_pose.position.x<<" y:"<<human_pose.position.y);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tool_handling_sim");
  ros::NodeHandle nh_;
  usleep(1000*1000);

  //ros::Subscriber human_pose_sub = nh_.subscribe("human_sim/pose", 1, human_pose_callback);
  ros::Subscriber human_pose_sub = nh_.subscribe("human/estimated/pose", 1, human_pose_callback);

  MoveBin movebin;

  geometry_msgs::Pose current_pose;

  geometry_msgs::Pose goal_pose;
  tf::Quaternion quat = tf::createQuaternionFromRPY(0,M_PI/2,M_PI);
  goal_pose.orientation.x = quat.x();
  goal_pose.orientation.y = quat.y();
  goal_pose.orientation.z = quat.z();
  goal_pose.orientation.w = quat.w();
  goal_pose.position.z = 1.3;

  while(ros::ok()){
    current_pose = movebin.group.getCurrentPose().pose;
    //ROS_INFO_STREAM("current_pose x:"<<current_pose.position.x<<" y:"<< current_pose.position.y);
    goal_pose.position.x = std::max(std::min(human_pose.position.x,2.8), 1.0);
    goal_pose.position.y = std::max(std::min(human_pose.position.y, 1.8),1.2);
    //ROS_INFO_STREAM("goal_pose x:"<<goal_pose.position.x<<" y:"<< goal_pose.position.y);

    if(fabs(goal_pose.position.x-current_pose.position.x)>0.1 || fabs(goal_pose.position.y-current_pose.position.y)>0.1){
	    movebin.traverseMove(goal_pose);
    }
    ros::spinOnce();
  }

  sleep(1.0);
  ros::shutdown();
  return 0;
}

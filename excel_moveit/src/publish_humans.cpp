#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <excel_moveit/Humans.h>
#include <excel_moveit/Human.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "publish_humans_node");
  ros::NodeHandle nh;
  ros::Publisher humans_publisher = nh.advertise<excel_moveit::Humans>("humans", 1);
  ros::Rate loop_rate(10);
  usleep(1000*1000);

  excel_moveit::Humans human_array;
  excel_moveit::Human human;

  /* A default pose */
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = 0.7;
  pose.position.y = 2.5;
  pose.position.z = 0.0;
  human.pose = pose;
  human.uncertainty = 0.25;
  human_array.humans.push_back(human);

  /* Another default pose */
  pose.orientation.w = 1.0;
  pose.position.x = 0.7;
  pose.position.y = 1.5;
  pose.position.z = 0.0;
  human.pose = pose;
  human.uncertainty = 0.55;

  /* Publishing the poses */
  human_array.humans.push_back(human);
  humans_publisher.publish(human_array);
  ROS_INFO("Advertising humans position");
  ros::spinOnce();
  loop_rate.sleep();
  return 0;
}

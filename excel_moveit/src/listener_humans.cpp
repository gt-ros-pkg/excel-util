#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <excel_moveit/Humans.h>
#include <excel_moveit/Human.h>

#define HUMAN_SIZE 1

ros::Publisher humans_publisher;

void printHumans(const excel_moveit::Humans& msg)
{
  
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "rail_link";
  attached_object.object.header.frame_id = "rail_link";
  attached_object.object.id = "human";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = HUMAN_SIZE;  

  for (int i=0; i<msg.humans.size(); ++i)
  {
    const excel_moveit::Human& data = msg.humans[i];
    ROS_INFO_STREAM("Human in : " << data.pose.position.x << " " << data.pose.position.y);
    
    primitive.dimensions[1] = data.uncertainty;
    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(data.pose);
  }
  attached_object.object.operation = attached_object.object.ADD;
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  humans_publisher.publish(planning_scene);
  ROS_INFO("-------");
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "listener_humans_node");
  ros::NodeHandle nh_;
  humans_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::Subscriber humans_listener = nh_.subscribe("humans", 1, printHumans);
  ros::spin();
}




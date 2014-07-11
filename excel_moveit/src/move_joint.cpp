#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_joint");
  ros::NodeHandle nh_;
  usleep(1000*1000);
  std::string joint = "shoulder_pan_joint";
  double value = 1.0;
  nh_.param("target_joint",joint,joint);
  nh_.param("target_value",value,value);
  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  spinner.start();
  usleep(1000*1000);
  // this connecs to a running instance of the move_group node
  move_group_interface::MoveGroup group("excel");
  // specify that we plan to move a joint to a specific position
  group.setJointValueTarget(joint, value);
  // plan the motion and then move the group to the sampled target
  group.move();
  ros::waitForShutdown();
}

#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_to_known_position");
  ros::NodeHandle nh_;
  usleep(1000*1000);
  std::string position = "Initial";
  nh_.param("target_position",position,position);
  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // this connects to a running instance of the move_group node
  move_group_interface::MoveGroup group("excel");
  // specify that our target will be a position defined in the SRDF
  group.setNamedTarget(position);
  // plan the motion and then move the group to the sampled target 
  group.move();
  ros::waitForShutdown();
}

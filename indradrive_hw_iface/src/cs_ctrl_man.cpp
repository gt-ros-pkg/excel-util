
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <indradrive_hw_iface/cs_robot_hw.h>

using namespace indradrive;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cs_controller_man");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  std::string joint_name;
  nh_priv.param<std::string>("joint_name", joint_name, "indradrive_cs_joint");

  IndradriveCSRobotHW cs_hw(nh, joint_name);
  cs_hw.init();

  controller_manager::ControllerManager cm(&cs_hw, nh);

  ros::Duration period(1.0/1000.0);
  while (ros::ok()) {
    cs_hw.read();
    cm.update(ros::Time::now(), period);
    cs_hw.write();
  }
}

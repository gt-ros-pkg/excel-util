
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <indradrive_hw_iface/vel_ec_ctrl.h>

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

  VelocityEthercatController cs_hw(nh, nh_priv, joint_name);
  if(cs_hw.init()) {
    printf("Failed to initialize controller\n");
    return -1;
  }

  controller_manager::ControllerManager cm(&cs_hw, nh);

  ros::Duration period(1.0/1000.0);
  ros::Rate r(1000.0);
  while (ros::ok()) {
    cs_hw.read();
    cm.update(ros::Time::now(), period);
    cs_hw.write();
    r.sleep();
  }
}

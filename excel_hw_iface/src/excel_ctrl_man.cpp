
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <indradrive_hw_iface/cs_robot_hw.h>
#include <ur_ctrl_client/ur_robot_hw.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cs_controller_man");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  std::string robot_ip;
  if(!nh_priv.getParam("robot_ip", robot_ip)) {
    ROS_ERROR("Missing robot IP address (robot_ip)");
    return -1;
  }
  XmlRpc::XmlRpcValue v;
  if(!nh_priv.getParam("joint_names", v) || v.size() != 7) {
    ROS_ERROR("Excel robot requires a list of the 7 joint names");
    return -1;
  }
  std::string indradrive_joint_name = v[0];
  std::vector<std::string> ur_joint_names;
  for(int i=1;i<7;i++)
    ur_joint_names.push_back(v[i]);

  indradrive::IndradriveCSRobotHW cs_hw(nh, indradrive_joint_name);
  cs_hw.init();

  ur::URRobotHW ur_hw(nh, ur_joint_names);
  ur_hw.init(robot_ip);

  hardware_interface::RobotHW excel_hw;
  excel_hw.registerInterfaceManager(&cs_hw);
  excel_hw.registerInterfaceManager(&ur_hw);

  controller_manager::ControllerManager cm(&excel_hw, nh);

  ros::Duration period(1.0/1000.0);
  ros::Rate r(1000.0);
  uint64_t counter = 0;
  while (ros::ok()) {
    cs_hw.read();
    if(counter % 8 == 0) // 125 Hz
      ur_hw.read();
    cm.update(ros::Time::now(), period);
    cs_hw.write();
    if(counter % 8 == 0) // 125 Hz
      ur_hw.write();
    r.sleep();
  }
}

#ifndef CS_ROBOT_HW_H
#define CS_ROBOT_HW_H

#include <ros/ros.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>

using hardware_interface::JointStateInterface;
using hardware_interface::VelocityJointInterface;
using hardware_interface::JointStateHandle;
using hardware_interface::JointHandle;

namespace indradrive
{

class IndradriveCSRobotHW : public hardware_interface::RobotHW
{
public:
  IndradriveCSRobotHW(ros::NodeHandle& nh, ros::NodeHandle& nh_priv, std::string& joint_name)
    : nh_(nh)
  {
    // Register joint interface handle
    jnt_state_iface_.registerHandle(
        JointStateHandle(joint_name, &pos_fb_, &vel_fb_, &eff_fb_));
    jnt_vel_iface_.registerHandle(
        JointHandle(jnt_state_iface_.getHandle(joint_name), &vel_cmd_));

    // // Register interfaces
    registerInterface(&jnt_state_iface_);
    registerInterface(&jnt_vel_iface_);
  } 

  virtual int init();
  virtual void read();
  virtual void write();

  ~IndradriveCSRobotHW();

protected:
  ros::NodeHandle nh_;

  // ros_control interfaces
  JointStateInterface     jnt_state_iface_;
  VelocityJointInterface  jnt_vel_iface_;

  double pos_fb_, vel_fb_, eff_fb_;
  double vel_cmd_;

  uint64_t counter_;
};
}

#endif

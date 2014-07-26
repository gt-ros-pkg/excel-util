#include <indradrive_hw_iface/cs_robot_hw.h>

namespace indradrive
{

IndradriveCSRobotHW::IndradriveCSRobotHW(ros::NodeHandle& nh, std::string& joint_name) :
  nh_(nh)
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

void IndradriveCSRobotHW::init()
{
  pos_fb_ = 0.0; vel_fb_ = 0.0; eff_fb_ = 0.0; vel_cmd_ = 0.0;
}

void IndradriveCSRobotHW::read()
{
}

void IndradriveCSRobotHW::write()
{
  pos_fb_ += vel_cmd_*0.001;
  vel_fb_ = vel_cmd_;
}

IndradriveCSRobotHW::~IndradriveCSRobotHW()
{
}

}

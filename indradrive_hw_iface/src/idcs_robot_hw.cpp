#include <indradrive_hw_iface/idcs_robot_hw.h>

namespace indradrive
{

int IndradriveCSRobotHW::init()
{
  double init_pos = 0.0;
  nh_priv_.getParam("init_pos", init_pos);
  pos_fb_ = init_pos; vel_fb_ = 0.0; eff_fb_ = 0.0; vel_cmd_ = 0.0;
  return 0;
}

void IndradriveCSRobotHW::read(ros::Time time, ros::Duration period)
{
}

void IndradriveCSRobotHW::write(ros::Time time, ros::Duration period)
{
  jnt_limits_iface_.enforceLimits(period);

  static double cur_acc = 0.0;
  static const double dt = 0.001;
  static const double gain = 1000.0;
  cur_acc = gain*(vel_cmd_ - vel_fb_);
  vel_fb_ += cur_acc*dt;
  pos_fb_ += vel_fb_*dt;
}

IndradriveCSRobotHW::~IndradriveCSRobotHW()
{
}

}

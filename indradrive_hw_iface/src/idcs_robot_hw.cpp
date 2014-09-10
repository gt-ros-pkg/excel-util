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

void IndradriveCSRobotHW::read()
{
}

void IndradriveCSRobotHW::write()
{
  static double target_pos = pos_fb_;
  const double dt = 0.001;
  target_pos += vel_cmd_*dt;

  double pos_last = pos_fb_;
  double u = 100.0*(target_pos - pos_fb_);
  pos_fb_ += u*dt;
  vel_fb_ = (pos_last - pos_fb_)/dt;
}

IndradriveCSRobotHW::~IndradriveCSRobotHW()
{
}

}

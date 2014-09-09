#include <indradrive_hw_iface/idcs_robot_hw.h>

namespace indradrive
{

int IndradriveCSRobotHW::init()
{
  pos_fb_ = 0.0; vel_fb_ = 0.0; eff_fb_ = 0.0; vel_cmd_ = 0.0;
  return 0;
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

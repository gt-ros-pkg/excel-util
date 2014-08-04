#include <indradrive_hw_iface/vel_ec_ctrl.h>

namespace indradrive {

VelocityEthercatController::VelocityEthercatController(
    ros::NodeHandle& nh, ros::NodeHandle& nh_priv, std::string& joint_name) :
  EthercatController(nh, joint_name),
  drv_stat_pdo_(new PDOConfiguration("Drive status", "S-0-0135", 0, 16, false)),
  pos_fb_pdo_(new PDOConfiguration("Position feedback", "S-0-0051", 0, 32, true)),
  vel_fb_pdo_(new PDOConfiguration("Velocity feedback", "S-0-0040", 0, 32, true)),
  drv_ctrl_pdo_(new PDOConfiguration("Device control", "S-0-0134", 0, 16, false)),
  pos_cmd_pdo_(new PDOConfiguration("Position command", "S-0-0047", 0, 32, true)),
  vel_cmd_pdo_(new PDOConfiguration("Velocity command", "S-0-0036", 0, 32, true)),
  drv_stat_pub_(nh_priv, "drive_status", 1, true),
  command_op_mode_(0),
  drive_state_cmd_(0)
{
  at_pdo_configs_.push_back(drv_stat_pdo_);
  at_pdo_configs_.push_back(pos_fb_pdo_);
  at_pdo_configs_.push_back(vel_fb_pdo_);
  mdt_pdo_configs_.push_back(drv_ctrl_pdo_);
  mdt_pdo_configs_.push_back(pos_cmd_pdo_);
  mdt_pdo_configs_.push_back(vel_cmd_pdo_);

  nh_priv.param<double>("scale_pos_fb", scale_pos_fb_, 1.0);
  nh_priv.param<double>("scale_vel_fb", scale_vel_fb_, 1.0);
  // nh.param<double>("scale_pos_cmd", scale_pos_cmd_, 1.0);
  nh_priv.param<double>("scale_vel_cmd", scale_vel_cmd_, 1.0);

  drv_ctrl_sub_ = nh_priv.subscribe("drive_control_cmd", 1, 
                               &VelocityEthercatController::driveControlCB, this);
}

void VelocityEthercatController::driveControlCB(std_msgs::UInt8ConstPtr msg)
{
  switch(msg->data) {
    case DevCtrlCmds::DRIVE_ON:
      drive_state_cmd_ |= 0x8000;
      break;
    case DevCtrlCmds::TORQUE_ENABLE:
      drive_state_cmd_ |= 0xE000;
      break;
    case DevCtrlCmds::CTRLED_MAX_DECEL:
      drive_state_cmd_ &= 0xDFFF;
      break;
    case DevCtrlCmds::TORQUE_DISABLE:
      drive_state_cmd_ &= 0xBFFF;
      break;
    case DevCtrlCmds::BEST_POSSIB_DECEL:
      drive_state_cmd_ &= 0x7FFF;
      break;
    case DevCtrlCmds::DRIVE_RESET:
      drive_state_cmd_ = 0x0000;
      break;
    default:
      break;
  }
}

int VelocityEthercatController::configureIDNs()
{

  uint8_t telegram_type_data[2] = LENDIAN(0x0007);
  if(ecrt_master_write_idn(master_, slave_info_.position, 0, 
        idn_to_hex("S-0-0015"), telegram_type_data, 2, NULL)) {
    printf("Failed to write telegram type\n");
    return -1;
  }
  printf("Using configurable telegram.\n");

  // Primary operation mode
  uint8_t vel_mode_data[2] = LENDIAN(0x0002);
  // uint8_t vel_mode_data[2] = LENDIAN(0x000B);
  if(ecrt_slave_config_idn(slv_cfg_, 0, idn_to_hex("S-0-0032"), EC_AL_STATE_SAFEOP, vel_mode_data, 2)) {
    printf("Failed to write mode\n");
    return -1;
  }

  return 0;
}

void VelocityEthercatController::readTelegram()
{
  static int32_t last_pos_fb_int = 0;
  static int32_t last_vel_fb_int = 0;
  static uint16_t last_drv_stat_bin = 0;

  int32_t pos_fb_int = EC_READ_S32(pos_fb_pdo_->data_address);
  int32_t vel_fb_int = EC_READ_S32(vel_fb_pdo_->data_address);
  uint16_t drv_stat_bin = EC_READ_U16(drv_stat_pdo_->data_address);

  if(!pos_fb_int) {
    pos_fb_int = last_pos_fb_int;
    vel_fb_int = last_vel_fb_int;
    drv_stat_bin = last_drv_stat_bin;
  } else {
    last_pos_fb_int = pos_fb_int;
    last_vel_fb_int = vel_fb_int;
    last_drv_stat_bin = drv_stat_bin;
  }

  pos_fb_ = pos_fb_int * scale_pos_fb_;
  vel_fb_ = vel_fb_int * scale_vel_fb_;

  if(counter_ % 10 == 0) {
    if(drv_stat_pub_.trylock()) {
      drv_stat_pub_.msg_.data = drv_stat_bin;
      drv_stat_pub_.unlockAndPublish();
    }
  }
}

void VelocityEthercatController::writeTelegram()
{
  // int32_t pos_cmd_int = pos_cmd_ * scale_pos_cmd_;
  // EC_WRITE_S32(pos_cmd_pdo_->data_address, pos_cmd_int);
  EC_WRITE_S32(pos_cmd_pdo_->data_address, 0);
  int32_t vel_cmd_int = vel_cmd_ * scale_vel_cmd_;
  EC_WRITE_S32(vel_cmd_pdo_->data_address, vel_cmd_int);

  uint16_t dev_ctrl_cmd = 0x0000;

  if(counter_ % 2 == 0) 
    dev_ctrl_cmd |= 0x0400;
  dev_ctrl_cmd |= drive_state_cmd_;
  dev_ctrl_cmd |= command_op_mode_;
#if 0
  if(counter_ % 1000 == 0) {
    printf("dev_ctrl_cmd %x\n", dev_ctrl_cmd);
  }
#endif

  EC_WRITE_U16(drv_ctrl_pdo_->data_address, dev_ctrl_cmd);
}

}

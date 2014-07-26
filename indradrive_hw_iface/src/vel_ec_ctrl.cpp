#include <indradrive_hw_iface/vel_ec_ctrl.h>

VelocityEthercatController::VelocityEthercatController(ros::NodeHandle& nh, std::string& joint_name) :
  EthercatController(nh, joint_name),
  drv_stat_pdo_(new PDOConfiguration("Drive status", "S-0-0135", 0, 16, false)),
  pos_fb_pdo_(new PDOConfiguration("Position feedback", "S-0-0051", 0, 32, true)),
  vel_fb_pdo_(new PDOConfiguration("Velocity feedback", "S-0-0040", 0, 32, true)),
  dev_ctrl_pdo_(new PDOConfiguration("Device control", "S-0-0134", 0, 16, false)),
  pos_cmd_pdo_(new PDOConfiguration("Position command", "S-0-0047", 0, 32, true)),
  vel_cmd_pdo_(new PDOConfiguration("Velocity command", "S-0-0036", 0, 32, true)),
  command_op_mode_(0),
  counter_(0)
{
  at_pdo_configs_.push_back(drv_stat_pdo_);
  at_pdo_configs_.push_back(pos_fb_pdo_);
  at_pdo_configs_.push_back(vel_fb_pdo_);
  mdt_pdo_configs_.push_back(dev_ctrl_pdo_);
  mdt_pdo_configs_.push_back(pos_cmd_pdo_);
  mdt_pdo_configs_.push_back(vel_cmd_pdo_);

  nh.param<double>("scale_pos_fb", scale_pos_fb_, 1.0);
  nh.param<double>("scale_vel_fb", scale_vel_fb_, 1.0);
  // nh.param<double>("scale_pos_cmd", scale_pos_cmd_, 1.0);
  nh.param<double>("scale_vel_cmd", scale_vel_cmd_, 1.0);
}

void VelocityEthercatController::configureIDNs()
{

  uint8_t telegram_type_data[2] = LENDIAN(0x0007);
  if(ecrt_master_write_idn(master_, slave_info.position, 0, 
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
}

void VelocityEthercatController::readTelegram()
{
  int32_t pos_fb_int = EC_READ_S32(pos_fb_pdo_->data_address);
  int32_t vel_fb_int = EC_READ_S32(vel_fb_pdo_->data_address);
  uint16_t drv_stat_bin = EC_READ_U16(drv_stat_pdo_->data_address);

  pos_fb_ = pos_fb_int * scale_pos_fb_;
  vel_fb_ = vel_fb_int * scale_vel_fb_;
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
  if(drive_on_)
    dev_ctrl_cmd |= 0x8000;
  if(drive_enable_)
    dev_ctrl_cmd |= 0x4000;
  if(drive_halt_)
    dev_ctrl_cmd |= 0x2000;
  dev_ctrl_cmd |= command_op_mode_;

  EC_WRITE_U16(dev_ctrl_pdo_->data_address, dev_ctrl_cmd);
}

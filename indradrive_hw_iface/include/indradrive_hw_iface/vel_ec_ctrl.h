#include <indradrive_hw_iface/ethercat_ctrl.h>

class VelocityEthercatController : public EthercatController
{
protected:
  VelocityEthercatController();
  virtual void configureIDNs();
  virtual void readTelegram();
  virtual void writeTelegram();

  PDOConfiguration* drv_stat_pdo_;
  PDOConfiguration* pos_fb_pdo_;
  PDOConfiguration* vel_fb_pdo_;

  PDOConfiguration* dev_ctrl_pdo_;
  PDOConfiguration* pos_cmd_pdo_;
  PDOConfiguration* vel_cmd_pdo_;

  double scale_pos_fb_;
  double scale_vel_fb_;
  // double scale_pos_cmd_; // Deactivated for vel ctrl
  double scale_vel_cmd_;

  // drive control commands
  uint16_t command_op_mode_;
  bool drive_on_;
  bool drive_enable_;
  bool drive_halt_;

  uint64_t counter_;
};


#include <boost/shared_ptr.hpp>

#include <indradrive_hw_iface/ethercat_ctrl.h>
#include <realtime_tools/realtime_publisher.h>

#include <std_msgs/UInt16.h>

using realtime_tools::RealtimePublisher;

namespace indradrive {

class VelocityEthercatController : public EthercatController
{
public:
  VelocityEthercatController(
      ros::NodeHandle& nh, ros::NodeHandle& nh_priv, std::string& joint_name);

protected:
  virtual int configureIDNs();
  virtual void readTelegram();
  virtual void writeTelegram();

  void driveControlCB(std_msgs::UInt16ConstPtr msg);

  PDOConfiguration* drv_stat_pdo_;
  PDOConfiguration* pos_fb_pdo_;
  PDOConfiguration* vel_fb_pdo_;

  PDOConfiguration* drv_ctrl_pdo_;
  PDOConfiguration* pos_cmd_pdo_;
  PDOConfiguration* vel_cmd_pdo_;

  RealtimePublisher<std_msgs::UInt16> drv_stat_pub_;
  ros::Subscriber drv_ctrl_sub_;

  double scale_pos_fb_;
  double scale_vel_fb_;
  // double scale_pos_cmd_; // Deactivated for vel ctrl
  double scale_vel_cmd_;

  // drive control commands
  uint16_t drive_state_cmd_;
  uint16_t command_op_mode_;
};

}


#include <std_msgs/Int8MultiArray.h>
#include <excel_bins/Bins.h>
#include <excel_bins/Bin.h>
#include <excel_bins/move_bin.h>

bool slotToTargetPose(const excel_bins::Bin& slot, 
                      double& x_tgt, double& y_tgt, double& o_tgt, bool& is_large)
{
  std::string size = slot.size;
  is_large = size == "large";
  x_tgt = slot.pose.position.x;
  y_tgt = slot.pose.position.y;
  tf::Quaternion co_quat(slot.pose.orientation.x, slot.pose.orientation.y,
                         slot.pose.orientation.z, slot.pose.orientation.w);
  tf::Matrix3x3 m(co_quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  o_tgt = 180*yaw/M_PI;
}

class BinManager
{
public:
  BinManager();

  int find

protected:
  excel_bins::Bins empty_slots;
  std_msgs::Int8MultiArray missing_bins;

  void missingCallback(const std_msgs::Int8MultiArray::ConstPtr& msg)
  {
    missing_bins = *msg;
  }

  void emptyCallback(const excel_bins::Bins::ConstPtr& msg)
  {
    empty_slots = *msg;
  }
};

#ifndef EXTRINSIC_CALIBRATION_CALIBRATION_BROADCASTER_H
#define EXTRINSIC_CALIBRATION_CALIBRATION_BROADCASTER_H

#include <ros/ros.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/SimpleCamera.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <extrinsic_calibration/calibration_job.h>

namespace extrinsic_calibration 
{

typedef pcl::PointCloud<pcl::PointXYZRGB> PCRGB;

tf::Transform gtsamPoseToTF(gtsam::Pose3& gtsam_pose);
gtsam::Pose3 tfPoseToGtsam(tf::Transform& tf_pose);

class CalibrationBroadcaster
{
public:
  CalibrationBroadcaster(ros::NodeHandle nh, CalibrationSetupPtr& setup, bool use_calib=true);
  void setUseCalib(bool use_calib) { use_calib_ = use_calib; }

protected:
  void timerCallback(const ros::TimerEvent & timer_event);

  CalibrationSetupPtr setup_;
  bool use_calib_;

  ros::NodeHandle nh_;
  ros::Timer timer_;
  tf::TransformBroadcaster tf_broadcaster_;
  std::vector<ros::Publisher> target_pc_pubs_;
};
}

#endif

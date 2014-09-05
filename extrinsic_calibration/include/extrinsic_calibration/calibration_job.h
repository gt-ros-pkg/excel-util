#ifndef EXTRINSIC_CALIBRATION_CALIBRATION_JOB_H
#define EXTRINSIC_CALIBRATION_CALIBRATION_JOB_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/SimpleCamera.h>

#include <boost/foreach.hpp>

#include <ros/ros.h>

#include <vector>
#include <math.h>

using boost::shared_ptr;

namespace extrinsic_calibration 
{

#define TARGET_TYPE_CHESSBOARD 0
#define TARGET_TYPE_CIRCLES 1

struct Camera
{
  std::string frame;
  std::string image_topic;
  shared_ptr<gtsam::SimpleCamera> guess_cam;
  shared_ptr<gtsam::SimpleCamera> calibrated_cam;
};

struct Target
{
  std::string frame;
  int type;
  double pt_spacing;
  int rows;
  int cols;
  shared_ptr<gtsam::Pose3> guess_pose;
  shared_ptr<gtsam::Pose3> calibrated_pose;
  std::vector<gtsam::Point3> target_pts;
};

struct Detection
{
  int camera_ind;
  int target_ind;
  std::vector<gtsam::Point2> image_pts;
};

struct Scene
{
  gtsam::Pose3 arm_pose;
  std::vector<shared_ptr<Detection> > detections;
};

struct CalibrationSetup 
{
  std::string world_frame;
  std::string ee_frame;
  std::vector<shared_ptr<Camera> > cameras;
  std::vector<shared_ptr<Target> > targets;

  void diffCalib(shared_ptr<CalibrationSetup>& that, 
                 bool this_use_calib, bool that_use_calib);
  static shared_ptr<CalibrationSetup> readFromParam(ros::NodeHandle& nh, const std::string& param_name);
  void writeToParam(ros::NodeHandle& nh, const std::string& param_name);
};

struct CalibrationJob 
{
  shared_ptr<CalibrationSetup> setup;
  std::vector<shared_ptr<Scene> > scenes;
  static shared_ptr<CalibrationJob> readFromParam(ros::NodeHandle& nh, 
      const std::string& setup_param_name, const std::string& job_param_name);
  void writeToParam(ros::NodeHandle& nh, const std::string& param_name);
};

typedef shared_ptr<CalibrationSetup> CalibrationSetupPtr;
typedef shared_ptr<CalibrationJob> CalibrationJobPtr;
typedef shared_ptr<Camera> CameraPtr;
typedef shared_ptr<Target> TargetPtr;
typedef shared_ptr<Detection> DetectionPtr;
typedef shared_ptr<Scene> ScenePtr;
}

#endif

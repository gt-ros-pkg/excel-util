#ifndef EXTRINSIC_CALIBRATION_CALIBRATION_JOB_H
#define EXTRINSIC_CALIBRATION_CALIBRATION_JOB_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/linear/Sampler.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/range/algorithm/random_shuffle.hpp>

#include <geometry_msgs/Pose.h>

#include <vector>
#include <math.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

using boost::shared_ptr;

namespace extrinsic_calibration 
{

tf::Transform gtsamPoseToTF(gtsam::Pose3& gtsam_pose);

struct Camera
{
  std::string frame;
  shared_ptr<gtsam::SimpleCamera> guess_cam;
  shared_ptr<gtsam::SimpleCamera> calibrated_cam;
};

struct Target
{
  std::string frame;
  std::string type;
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

typedef shared_ptr<CalibrationSetup> CalibrationSetupPtr;

struct CalibrationJob 
{
  CalibrationSetupPtr setup;
  std::vector<shared_ptr<Scene> > scenes;
  static shared_ptr<CalibrationJob> readFromParam(ros::NodeHandle& nh, 
      const std::string& setup_param_name, const std::string& job_param_name);
  void writeToParam(ros::NodeHandle& nh, const std::string& param_name);
};

typedef shared_ptr<CalibrationJob> CalibrationJobPtr;

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

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
  shared_ptr<gtsam::SimpleCamera> calibrated_cam;
  shared_ptr<gtsam::SimpleCamera> guess_cam;
  std::string calibrated_frame;
  std::string guess_frame;

  Camera(const std::string& _calibrated_frame, const std::string& _guess_frame);
};

Camera::Camera(const std::string& _calibrated_frame, const std::string& _guess_frame) :
  calibrated_frame(_calibrated_frame),
  guess_frame(_guess_frame)
{
}


struct Target
{
  shared_ptr<gtsam::Pose3> calibrated_pose;
  shared_ptr<gtsam::Pose3> guess_pose;
  std::string calibrated_frame;
  std::string guess_frame;
  std::vector<gtsam::Point3> target_pts;

  Target(const std::string& _calibrated_frame, const std::string& _guess_frame);
};

Target::Target(const std::string& _calibrated_frame, const std::string& _guess_frame) :
  calibrated_frame(_calibrated_frame),
  guess_frame(_guess_frame)
{
}

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
  std::vector<shared_ptr<Camera> > cameras;
  std::vector<shared_ptr<Target> > targets;

  CalibrationSetup(ros::NodeHandle _nh, 
                   const std::string& _world_frame, const std::string& _ee_frame);

  ros::NodeHandle nh;
  std::string world_frame;
  std::string ee_frame;
  ros::Timer timer;
  tf::TransformBroadcaster tf_broadcaster;
  std::vector<ros::Publisher> target_pc_pubs;
  std::vector<ros::Publisher> target_pc_guess_pubs;
  void timerCallback(const ros::TimerEvent & timer_event);
};

struct CalibrationJob 
{
  shared_ptr<CalibrationSetup> setup;
  std::vector<shared_ptr<Scene> > scenes;
};
}

#endif

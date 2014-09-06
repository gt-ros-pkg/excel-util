
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <extrinsic_calibration/calibration_job.h>
#include <extrinsic_calibration/extrinsic_calibration.h>
#include <extrinsic_calibration/calibration_broadcaster.h>

using namespace extrinsic_calibration;

class CalibrationServer
{
public:
  CalibrationServer(ros::NodeHandle& nh, CalibrationJobPtr& cal_job);

protected:
  CalibrationJobPtr cal_job_;
  CalibrationBroadcaster calib_bc_;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  tf::TransformListener tf_list_;

  std::vector<image_transport::Subscriber> img_subs_;
  std::vector<image_transport::Publisher> img_detect_pubs_;
  ros::ServiceServer detect_srv_;
  ros::ServiceServer calib_srv_;
  ros::ServiceServer save_srv_;
  ros::Subscriber use_calib_sub_;

  std::vector<std::vector<bool> > saving_detections_;
  ScenePtr cur_scene_;

  void imageCallback(int camera_ind, const sensor_msgs::ImageConstPtr& msg);
  bool detectCallback(std_srvs::Empty::Request& request, 
                      std_srvs::Empty::Response& response);
  bool calibrateCallback(std_srvs::Empty::Request& request, 
                         std_srvs::Empty::Response& response);
  bool saveCallback(std_srvs::Empty::Request& request, 
                    std_srvs::Empty::Response& response);
  void useCalibCallback(std_msgs::BoolPtr msg);
};

CalibrationServer::CalibrationServer(ros::NodeHandle& nh, CalibrationJobPtr& cal_job)
  : nh_(nh), it_(nh), cal_job_(cal_job), calib_bc_(nh, cal_job->setup, false)
{
  saving_detections_.resize(cal_job_->setup->cameras.size());
  for(int i = 0; i < cal_job_->setup->cameras.size(); ++i) {
    saving_detections_[i].resize(cal_job_->setup->targets.size(), false);
    CameraPtr cam = cal_job_->setup->cameras[i];
    img_subs_.push_back(it_.subscribe(cam->image_topic, 1, 
                        boost::bind(&CalibrationServer::imageCallback, this, i, _1)));
    img_detect_pubs_.push_back(it_.advertise(cam->image_topic + "_detections", 1));
  }

  detect_srv_ = nh_.advertiseService("detect_targets", &CalibrationServer::detectCallback, this);
  calib_srv_ = nh_.advertiseService("calibrate", &CalibrationServer::calibrateCallback, this);
  save_srv_ = nh_.advertiseService("save", &CalibrationServer::saveCallback, this);
  use_calib_sub_ = nh_.subscribe("use_calib", 1, &CalibrationServer::useCalibCallback, this);
}

void CalibrationServer::imageCallback(int camera_ind, const sensor_msgs::ImageConstPtr& msg)
{
  CameraPtr cam = cal_job_->setup->cameras[camera_ind];
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  for(int target_ind = 0; target_ind < cal_job_->setup->targets.size(); ++target_ind) {
    TargetPtr tgt = cal_job_->setup->targets[target_ind];
    cv::Size pattern_size(tgt->cols, tgt->rows);
    std::vector<cv::Point2f> target_img_pts;
    bool pattern_found;
    cv::Mat grey_img;
    // cv::cvtColor(tgt->type, grey_img, cv::CV_8U
    if(tgt->type == TARGET_TYPE_CHESSBOARD) {
      pattern_found = cv::findChessboardCorners(cv_ptr->image, pattern_size, target_img_pts,
                cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
      if(pattern_found) {
        cv::TermCriteria term_criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1);
        cv::cornerSubPix(cv_ptr->image, target_img_pts, cv::Size(11, 11), cv::Size(-1, -1), 
                         term_criteria);
      }
    }
    else if(tgt->type == TARGET_TYPE_CIRCLES) {
      pattern_found = cv::findCirclesGrid(cv_ptr->image, pattern_size, target_img_pts,
                                          cv::CALIB_CB_SYMMETRIC_GRID);
    }
    if(pattern_found) {
      if(saving_detections_[camera_ind][target_ind]) {
        DetectionPtr detect = DetectionPtr(new Detection);
        detect->camera_ind = camera_ind;
        detect->target_ind = target_ind;
        for(int i = 0; i < target_img_pts.size(); ++i) 
          detect->image_pts.push_back(gtsam::Point2(target_img_pts[i].x, target_img_pts[i].y));
        cur_scene_->detections.push_back(detect);
        saving_detections_[camera_ind][target_ind] = false;
      }
      drawChessboardCorners(cv_ptr->image, pattern_size, cv::Mat(target_img_pts), pattern_found);
    }
    img_detect_pubs_[camera_ind].publish(cv_ptr->toImageMsg());
  }
}

bool CalibrationServer::detectCallback(std_srvs::Empty::Request& request, 
                                       std_srvs::Empty::Response& response)
{
  cur_scene_.reset(new Scene);
  std::cout << "detectCallback in\n";
  for(int i = 0; i < saving_detections_.size(); ++i)
    for(int j = 0; j < saving_detections_[i].size(); ++j)
      saving_detections_[i][j] = true;
  
  tf::StampedTransform arm_pose_tf;
  try {
    tf_list_.waitForTransform(cal_job_->setup->world_frame, cal_job_->setup->ee_frame, 
                              ros::Time(0), ros::Duration(1.0) );
    tf_list_.lookupTransform(cal_job_->setup->world_frame, cal_job_->setup->ee_frame, 
                             ros::Time(0), arm_pose_tf);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    return false;
  }
  cur_scene_->arm_pose = tfPoseToGtsam(arm_pose_tf);
  std::cout << cur_scene_->arm_pose << "\n";
  ros::Duration(1.0).sleep();
  std::cout << "detections.size" << cur_scene_->detections.size() << "\n";
  cal_job_->scenes.push_back(cur_scene_);
  for(int i = 0; i < saving_detections_.size(); ++i)
    for(int j = 0; j < saving_detections_[i].size(); ++j)
      saving_detections_[i][j] = false;
  return true;
}

bool CalibrationServer::calibrateCallback(std_srvs::Empty::Request& request, 
                                          std_srvs::Empty::Response& response)
{
  // BOOST_FOREACH(shared_ptr<Scene> scene, cal_job_->scenes) {
  //   BOOST_FOREACH(shared_ptr<Detection> detect, scene->detections) {
  //     std::cout << "DETECTION\n";
  //     for(int i = 0; i < detect->image_pts.size(); ++i) {
  //       shared_ptr<Camera> cam = cal_job_->setup->cameras[detect->camera_ind];
  //       shared_ptr<Target> tgt = cal_job_->setup->targets[detect->target_ind];
  //       gtsam::Pose3 pose_ee = scene->arm_pose;
  //       gtsam::Point3 pt_tgt = tgt->target_pts[i];
  //       gtsam::Point2 pt_img = detect->image_pts[i];
  //       gtsam::Pose3 pose_tgt = *(tgt->guess_pose);
  //       gtsam::Point3 pt_ee = pose_tgt.transform_from(pt_tgt);
  //       gtsam::Point3 pt_wl = pose_ee.transform_from(pt_ee);
  //       gtsam::Point2 pt_img_est = cam->guess_cam->project(pt_wl);
  //       std::cout << pt_img_est << ",";
  //     }
  //     std::cout << "\n\n\n";
  //   }
  // }
  cameraArmTargetDetectPointCalibrate(cal_job_, 2.0, 0.001, 0.05);
  calib_bc_.setUseCalib(true);
  return true;
}

bool CalibrationServer::saveCallback(std_srvs::Empty::Request& request, 
                                     std_srvs::Empty::Response& response)
{
  cal_job_->writeToParam(nh_, "calibration_job");
  cal_job_->setup->writeToParam(nh_, "calibration_setup");
  cal_job_->writeToParam(nh_, "calibration_job_copy");
  cal_job_->setup->writeToParam(nh_, "calibration_setup_copy");
  return true;
}

void CalibrationServer::useCalibCallback(std_msgs::BoolPtr msg)
{
  calib_bc_.setUseCalib(msg->data);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ros_calibration_server");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner async_spin(4);
  CalibrationJobPtr cal_job = 
    CalibrationJob::readFromParam(nh, "calibration_setup", "calibration_job");
  CalibrationServer calib_srv(nh, cal_job);
  async_spin.start();
  ros::spin();
}

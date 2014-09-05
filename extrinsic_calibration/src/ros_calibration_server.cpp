
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

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  std::vector<image_transport::Subscriber> img_subs_;
  std::vector<image_transport::Publisher> img_detect_pubs_;

  std::vector<std::vector<bool> > saving_detections_;
  ScenePtr cur_scene_;

  void imageCallback(int camera_ind, const sensor_msgs::ImageConstPtr& msg);
};

CalibrationServer::CalibrationServer(ros::NodeHandle& nh, CalibrationJobPtr& cal_job)
  : nh_(nh), it_(nh), cal_job_(cal_job)
{
  saving_detections_.resize(cal_job_->setup->cameras.size());
  for(int i = 0; i < cal_job_->setup->cameras.size(); ++i) {
    saving_detections_[i].resize(cal_job_->setup->targets.size(), false);
    CameraPtr cam = cal_job_->setup->cameras[i];
    img_subs_.push_back(it_.subscribe(cam->image_topic, 1, 
                        boost::bind(&CalibrationServer::imageCallback, this, i, _1)));
    img_detect_pubs_.push_back(it_.advertise(cam->image_topic + "_detections", 1));
  }
}

void CalibrationServer::imageCallback(int camera_ind, const sensor_msgs::ImageConstPtr& msg)
{
  CameraPtr cam = cal_job_->setup->cameras[camera_ind];
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  for(int target_ind = 0; target_ind < cal_job_->setup->targets.size(); ++target_ind) {
    TargetPtr tgt = cal_job_->setup->targets[target_ind];
    cv::Size pattern_size(tgt->rows, tgt->cols);
    std::vector<cv::Point2f> target_img_pts;
    bool pattern_found;
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

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ros_calibration_server");
  ros::NodeHandle nh;
  CalibrationJobPtr cal_job = 
    CalibrationJob::readFromParam(nh, "calibration_setup", "calibration_job");
  CalibrationServer calib_srv(nh, cal_job);
  ros::spin();
}


#include <extrinsic_calibration/calibration_broadcaster.h>

namespace extrinsic_calibration 
{

tf::Transform gtsamPoseToTF(gtsam::Pose3& gtsam_pose)
{
  gtsam::Point3 pos = gtsam_pose.translation();
  gtsam::Quaternion quat = gtsam_pose.rotation().toQuaternion();
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pos.x(), pos.y(), pos.z()));
  transform.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
  return transform;
}

CalibrationBroadcaster::
CalibrationBroadcaster(ros::NodeHandle nh, CalibrationSetupPtr& setup, bool use_calib) :
  nh_(nh), setup_(setup), use_calib_(use_calib)
{
  timer_ = nh_.createTimer(ros::Rate(1.0), &CalibrationBroadcaster::timerCallback, this);
}

void CalibrationBroadcaster::timerCallback(const ros::TimerEvent & timer_event)
{
  BOOST_FOREACH( shared_ptr<Camera> camera, setup_->cameras) {
    if(use_calib_ && camera->calibrated_cam)
      tf_broadcaster_.sendTransform(
          tf::StampedTransform(gtsamPoseToTF(camera->calibrated_cam->pose()), ros::Time::now(), 
                               setup_->world_frame, camera->frame));
    if(!use_calib_ && camera->guess_cam)
      tf_broadcaster_.sendTransform(
          tf::StampedTransform(gtsamPoseToTF(camera->guess_cam->pose()), ros::Time::now(), 
                               setup_->world_frame, camera->frame));
  }
  for(int i = 0; i < setup_->targets.size(); ++i) {
    shared_ptr<Target> target = setup_->targets[i];
    if(use_calib_ && target->calibrated_pose)
      tf_broadcaster_.sendTransform(
          tf::StampedTransform(gtsamPoseToTF(*(target->calibrated_pose)), ros::Time::now(), 
                               setup_->ee_frame, target->frame));
    if(!use_calib_ && target->guess_pose)
      tf_broadcaster_.sendTransform(
          tf::StampedTransform(gtsamPoseToTF(*(target->guess_pose)), ros::Time::now(), 
                               setup_->ee_frame, target->frame));

    if(i == target_pc_pubs_.size()) {
      target_pc_pubs_.push_back(
          nh_.advertise<PCRGB>("target_points_" + boost::lexical_cast<std::string>(i+1), 1));
    }
    PCRGB::Ptr pc_msg (new PCRGB);
    pc_msg->height = pc_msg->width = 1;
    BOOST_FOREACH(gtsam::Point3 gt_pt, target->target_pts) {
      pcl::PointXYZRGB pc_pt(155+std::min((int)(gt_pt.x()*100),130), 155+std::min((int)(gt_pt.y()*130),100), 55); // r g b uint8
      pc_pt.x = gt_pt.x(); pc_pt.y = gt_pt.y(); pc_pt.z = gt_pt.z();
      pc_msg->points.push_back(pc_pt);
    }
    pc_msg->width = pc_msg->points.size();

    pc_msg->header.frame_id = target->frame;
    target_pc_pubs_[i].publish(pc_msg);
  }
}
}

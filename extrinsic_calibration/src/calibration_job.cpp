
#include <extrinsic_calibration/calibration_job.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PCRGB;

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

CalibrationSetup::CalibrationSetup(ros::NodeHandle _nh, 
                                   const std::string& _world_frame, const std::string& _ee_frame) :
  nh(_nh), world_frame(_world_frame), ee_frame(_ee_frame)
{
  timer = nh.createTimer(ros::Rate(1.0), &CalibrationSetup::timerCallback, this);
}

void CalibrationSetup::timerCallback(const ros::TimerEvent & timer_event)
{
  BOOST_FOREACH( shared_ptr<Camera> camera, cameras) {
    if(camera->calibrated_cam)
      tf_broadcaster.sendTransform(
          tf::StampedTransform(gtsamPoseToTF(camera->calibrated_cam->pose()), ros::Time::now(), 
                               world_frame, camera->calibrated_frame));
    if(camera->guess_cam)
      tf_broadcaster.sendTransform(
          tf::StampedTransform(gtsamPoseToTF(camera->guess_cam->pose()), ros::Time::now(), 
                               world_frame, camera->guess_frame));
  }
  for(int i = 0; i < targets.size(); ++i) {
    shared_ptr<Target> target = targets[i];
    if(target->calibrated_pose)
      tf_broadcaster.sendTransform(
          tf::StampedTransform(gtsamPoseToTF(*(target->calibrated_pose)), ros::Time::now(), 
                               ee_frame, target->calibrated_frame));
    if(target->guess_pose)
      tf_broadcaster.sendTransform(
          tf::StampedTransform(gtsamPoseToTF(*(target->guess_pose)), ros::Time::now(), 
                               ee_frame, target->guess_frame));

    if(i == target_pc_pubs.size()) {
      target_pc_pubs.push_back(
          nh.advertise<PCRGB>("target_points_" + boost::lexical_cast<std::string>(i+1), 1));
      target_pc_guess_pubs.push_back(
          nh.advertise<PCRGB>("target_points_" + boost::lexical_cast<std::string>(i+1) + "_guess", 1));
    }
    PCRGB::Ptr pc_msg (new PCRGB);
    pc_msg->height = pc_msg->width = 1;
    BOOST_FOREACH(gtsam::Point3 gt_pt, target->target_pts) {
      pcl::PointXYZRGB pc_pt(155+std::min((int)(gt_pt.x()*100),130), 155+std::min((int)(gt_pt.y()*130),100), 55); // r g b uint8
      pc_pt.x = gt_pt.x(); pc_pt.y = gt_pt.y(); pc_pt.z = gt_pt.z();
      pc_msg->points.push_back(pc_pt);
    }
    pc_msg->width = pc_msg->points.size();

    pc_msg->header.frame_id = target->calibrated_frame;
    target_pc_pubs[i].publish(pc_msg);

    pc_msg->header.frame_id = target->guess_frame;
    target_pc_guess_pubs[i].publish(pc_msg);
  }
}
}

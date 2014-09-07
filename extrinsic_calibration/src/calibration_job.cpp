
#include <extrinsic_calibration/calibration_job.h>

using namespace XmlRpc;

namespace extrinsic_calibration 
{

double readXmlRpcNumber(XmlRpc::XmlRpcValue& num_val)
{
  ROS_ASSERT(num_val.getType() == XmlRpcValue::TypeDouble ||
             num_val.getType() == XmlRpcValue::TypeInt);
  if(num_val.getType() == XmlRpcValue::TypeInt)
    return (double) static_cast<int>(num_val);

  return static_cast<double>(num_val);
}

int readXmlRpcInt(XmlRpc::XmlRpcValue& num_val)
{
  ROS_ASSERT(num_val.getType() == XmlRpcValue::TypeDouble ||
             num_val.getType() == XmlRpcValue::TypeInt);
  if(num_val.getType() == XmlRpcValue::TypeDouble)
    return (int) static_cast<double>(num_val);

  return static_cast<int>(num_val);
}

std::string readXmlRpcString(XmlRpc::XmlRpcValue& str_val)
{
  ROS_ASSERT(str_val.getType() == XmlRpcValue::TypeString);
  return static_cast<std::string>(str_val);
}

gtsam::Point3 readXmlRpcPoint3(XmlRpc::XmlRpcValue& pos_val)
{
  ROS_ASSERT(pos_val.getType() == XmlRpcValue::TypeStruct);
  return gtsam::Point3(readXmlRpcNumber(pos_val["x"]),
                       readXmlRpcNumber(pos_val["y"]),
                       readXmlRpcNumber(pos_val["z"]));
}

gtsam::Rot3 readXmlRpcRot3(XmlRpc::XmlRpcValue& rot_val)
{
  ROS_ASSERT(rot_val.getType() == XmlRpcValue::TypeStruct);
  return gtsam::Rot3::quaternion(readXmlRpcNumber(rot_val["w"]),
                                 readXmlRpcNumber(rot_val["x"]),
                                 readXmlRpcNumber(rot_val["y"]),
                                 readXmlRpcNumber(rot_val["z"]));
}

gtsam::Pose3 readXmlRpcPose3(XmlRpc::XmlRpcValue& pose_val)
{
  XmlRpcValue pos_val = pose_val["translation"];
  XmlRpcValue rot_val = pose_val["rotation"];
  return gtsam::Pose3(readXmlRpcRot3(rot_val), readXmlRpcPoint3(pos_val));
}

gtsam::SimpleCamera readXmlRpcSimpleCamera(XmlRpc::XmlRpcValue& cam_val)
{
  ROS_ASSERT(cam_val.getType() == XmlRpcValue::TypeStruct);
  XmlRpcValue pose_val = cam_val["pose"];
  XmlRpcValue calib_val = cam_val["calibration"];
  ROS_ASSERT(calib_val.getType() == XmlRpcValue::TypeStruct);
  gtsam::Cal3_S2 calib(readXmlRpcNumber(calib_val["fx"]), readXmlRpcNumber(calib_val["fy"]), 
                       readXmlRpcNumber(calib_val["skew"]), 
                       readXmlRpcNumber(calib_val["u0"]), readXmlRpcNumber(calib_val["v0"]));
  return gtsam::SimpleCamera(readXmlRpcPose3(pose_val), calib);
}

CalibrationSetupPtr CalibrationSetup::
    readFromParam(ros::NodeHandle& nh, const std::string& param_name)
{
  CalibrationSetupPtr setup = CalibrationSetupPtr(new CalibrationSetup);
  XmlRpcValue cal_setup_param;
  if(!nh.getParam(param_name, cal_setup_param)) {
    ROS_ERROR_STREAM("Missing Parameter " << param_name.c_str());
    return CalibrationSetupPtr();
  }
  ROS_ASSERT(cal_setup_param.getType() == XmlRpcValue::TypeStruct);
  setup->world_frame = readXmlRpcString(cal_setup_param["world_frame"]);
  setup->ee_frame = readXmlRpcString(cal_setup_param["ee_frame"]);
  XmlRpcValue cameras_val = cal_setup_param["cameras"];
  XmlRpcValue targets_val = cal_setup_param["targets"];

  ROS_ASSERT(cameras_val.getType() == XmlRpcValue::TypeArray);
  ROS_ASSERT(cameras_val.size() > 0);
  ROS_ASSERT(targets_val.getType() == XmlRpcValue::TypeArray);
  ROS_ASSERT(targets_val.size() > 0);

  for(int i = 0; i < cameras_val.size(); i++) {
    XmlRpcValue camera_val = cameras_val[i];
    ROS_ASSERT(camera_val.getType() == XmlRpcValue::TypeStruct);
    shared_ptr<Camera> cam = shared_ptr<Camera>(new Camera);

    cam->frame = readXmlRpcString(camera_val["frame"]);
    cam->image_topic = readXmlRpcString(camera_val["image_topic"]);
    cam->guess_cam.reset(
        new gtsam::SimpleCamera(readXmlRpcSimpleCamera(camera_val["guess_cam"])));
    cam->calibrated_cam.reset(
        new gtsam::SimpleCamera(readXmlRpcSimpleCamera(camera_val["calibrated_cam"])));
    setup->cameras.push_back(cam);
  }

  for(int i = 0; i < targets_val.size(); i++) {
    XmlRpcValue target_val = targets_val[i];
    ROS_ASSERT(target_val.getType() == XmlRpcValue::TypeStruct);
    shared_ptr<Target> tgt = shared_ptr<Target>(new Target);

    tgt->frame = readXmlRpcString(target_val["frame"]);
    tgt->type = readXmlRpcInt(target_val["type"]);
    ROS_ASSERT(tgt->type == TARGET_TYPE_CHESSBOARD || tgt->type == TARGET_TYPE_CIRCLES);
    tgt->pt_spacing = readXmlRpcNumber(target_val["pt_spacing"]);
    tgt->rows = readXmlRpcInt(target_val["rows"]);
    tgt->cols = readXmlRpcInt(target_val["cols"]);
    tgt->guess_pose.reset(new gtsam::Pose3(readXmlRpcPose3(target_val["guess_pose"])));
    tgt->calibrated_pose.reset(new gtsam::Pose3(readXmlRpcPose3(target_val["calibrated_pose"])));

    ROS_ASSERT_MSG(tgt->rows < tgt->cols, 
                   "We must have a pattern with strictly more columns than rows");
    for(int j=0;j<tgt->rows;j++) 
      for(int k=0;k<tgt->cols;k++) 
        tgt->target_pts.push_back(gtsam::Point3(tgt->pt_spacing*k, tgt->pt_spacing*j, 0.0));
    setup->targets.push_back(tgt);
  }
  return setup;
}

void writeXmlRpcPoint3(const gtsam::Point3& pos, XmlRpc::XmlRpcValue& pos_val)
{
  pos_val["x"] = pos.x();
  pos_val["y"] = pos.y();
  pos_val["z"] = pos.z();
}

void writeXmlRpcRot3(const gtsam::Rot3& rot, XmlRpc::XmlRpcValue& rot_val)
{
  gtsam::Quaternion quat = rot.toQuaternion();
  rot_val["w"] = quat.w();
  rot_val["x"] = quat.x();
  rot_val["y"] = quat.y();
  rot_val["z"] = quat.z();
}

void writeXmlRpcPose3(const gtsam::Pose3& pose, XmlRpc::XmlRpcValue& pose_val)
{
  XmlRpcValue pos_val;
  writeXmlRpcPoint3(pose.translation(), pos_val);
  pose_val["translation"] = pos_val;
  XmlRpcValue rot_val;
  writeXmlRpcRot3(pose.rotation(), rot_val);
  pose_val["rotation"] = rot_val;
}

void writeXmlRpcSimpleCamera(const gtsam::SimpleCamera& cam, XmlRpc::XmlRpcValue& cam_val)
{
  XmlRpcValue pose_val;
  writeXmlRpcPose3(cam.pose(), pose_val);
  cam_val["pose"] = pose_val;

  XmlRpcValue calib_val; 
  calib_val["fx"] = cam.calibration().fx();
  calib_val["fy"] = cam.calibration().fy();
  calib_val["skew"] = cam.calibration().skew();
  calib_val["u0"] = cam.calibration().px();
  calib_val["v0"] = cam.calibration().py();
  cam_val["calibration"] = calib_val;
}

void CalibrationSetup::writeToParam(ros::NodeHandle& nh, const std::string& param_name)
{
  XmlRpcValue cal_setup_param;
  cal_setup_param["world_frame"] = world_frame;
  cal_setup_param["ee_frame"] = ee_frame;

  XmlRpcValue cameras_val;
  for(int i = 0; i < cameras.size(); i++) {
    shared_ptr<Camera> cam = cameras[i];
    XmlRpcValue camera_val;
    camera_val["frame"] = cam->frame;
    camera_val["image_topic"] = cam->image_topic;
    XmlRpcValue guess_cam_val;
    writeXmlRpcSimpleCamera(*(cam->guess_cam), guess_cam_val);
    camera_val["guess_cam"] = guess_cam_val;
    XmlRpcValue calibrated_cam_val;
    writeXmlRpcSimpleCamera(*(cam->calibrated_cam), calibrated_cam_val);
    camera_val["calibrated_cam"] = calibrated_cam_val;
    cameras_val[i] = camera_val;
  }
  cal_setup_param["cameras"] = cameras_val;

  XmlRpcValue targets_val;
  for(int i = 0; i < targets.size(); i++) {
    shared_ptr<Target> tgt = targets[i];
    XmlRpcValue target_val;
    target_val["frame"] = tgt->frame;
    target_val["type"] = tgt->type;
    target_val["pt_spacing"] = tgt->pt_spacing;
    target_val["rows"] = tgt->rows;
    target_val["cols"] = tgt->cols;
    XmlRpcValue guess_pose_val;
    writeXmlRpcPose3(*(tgt->guess_pose), guess_pose_val);
    target_val["guess_pose"] = guess_pose_val;
    XmlRpcValue calibrated_pose_val;
    writeXmlRpcPose3(*(tgt->calibrated_pose), calibrated_pose_val);
    target_val["calibrated_pose"] = calibrated_pose_val;
    targets_val[i] = target_val;
  }
  cal_setup_param["targets"] = targets_val;

  nh.setParam(param_name, cal_setup_param);
}

void CalibrationJob::writeToParam(ros::NodeHandle& nh, const std::string& param_name)
{
  XmlRpcValue scenes_val;
  for(int i = 0; i < scenes.size(); ++i) {
    shared_ptr<Scene> scene = scenes[i];
    XmlRpcValue scene_val;

    XmlRpcValue arm_pose_val;
    writeXmlRpcPose3(scene->arm_pose, arm_pose_val);
    scene_val["arm_pose"] = arm_pose_val;

    XmlRpcValue detections_val;
    for(int j = 0; j < scene->detections.size(); ++j) {
      shared_ptr<Detection> detection = scene->detections[j];
      XmlRpcValue detection_val;
      detection_val["camera_ind"] = detection->camera_ind;
      detection_val["target_ind"] = detection->target_ind;

      XmlRpcValue image_pts_val;
      for(int k = 0; k < detection->image_pts.size(); ++k) {
        XmlRpcValue image_pt_val;
        image_pt_val["u"] = detection->image_pts[k].x();
        image_pt_val["v"] = detection->image_pts[k].y();
        image_pts_val[k] = image_pt_val;
      }
      detection_val["image_pts"] = image_pts_val;

      detections_val[j] = detection_val;
    }
    scene_val["detections"] = detections_val;
    scenes_val[i] = scene_val;
  }
  nh.setParam(param_name, scenes_val);
}

CalibrationJobPtr CalibrationJob::
    readFromParam(ros::NodeHandle& nh, const std::string& setup_param_name, 
                                       const std::string& job_param_name)
{
  CalibrationJobPtr job = CalibrationJobPtr(new CalibrationJob);
  job->setup = CalibrationSetup::readFromParam(nh, setup_param_name);

  XmlRpcValue scenes_val;
  if(!nh.getParam(job_param_name, scenes_val)) {
    ROS_INFO_STREAM("Missing Parameter " << job_param_name.c_str());
    return job;
  }
  ROS_ASSERT(scenes_val.getType() == XmlRpcValue::TypeArray);
  ROS_ASSERT(scenes_val.size() > 0);

  for(int i = 0; i < scenes_val.size(); ++i) {
    XmlRpcValue scene_val = scenes_val[i];
    ROS_ASSERT(scene_val.getType() == XmlRpcValue::TypeStruct);

    shared_ptr<Scene> scene = shared_ptr<Scene>(new Scene);
    scene->arm_pose = readXmlRpcPose3(scene_val["arm_pose"]);

    XmlRpcValue detections_val = scene_val["detections"];
    ROS_ASSERT(detections_val.getType() == XmlRpcValue::TypeArray);
    ROS_ASSERT(detections_val.size() > 0);
    for(int j = 0; j < detections_val.size(); ++j) {
      XmlRpcValue detection_val = detections_val[j];
      ROS_ASSERT(detection_val.getType() == XmlRpcValue::TypeStruct);
      shared_ptr<Detection> detection = shared_ptr<Detection>(new Detection);
      detection->camera_ind = detection_val["camera_ind"];
      detection->target_ind = detection_val["target_ind"];

      XmlRpcValue image_pts_val = detection_val["image_pts"];
      ROS_ASSERT(image_pts_val.getType() == XmlRpcValue::TypeArray);
      ROS_ASSERT(image_pts_val.size() > 0);
      for(int k = 0; k < image_pts_val.size(); ++k) {
        XmlRpcValue image_pt_val = image_pts_val[k];
        ROS_ASSERT(image_pt_val.getType() == XmlRpcValue::TypeStruct);
        detection->image_pts.push_back(gtsam::Point2(image_pt_val["u"], image_pt_val["v"]));
      }
      scene->detections.push_back(detection);
    }
    job->scenes.push_back(scene);
  }
  return job;
}

void CalibrationSetup::diffCalib(CalibrationSetupPtr& that, 
                                 bool this_use_calib, bool that_use_calib)
{
  for(int i = 0; i < this->cameras.size(); ++i) {
    gtsam::Pose3 this_cam_pose;
    if(this_use_calib) 
      this_cam_pose = this->cameras[i]->calibrated_cam->pose();
    else 
      this_cam_pose = this->cameras[i]->guess_cam->pose();
    gtsam::Pose3 that_cam_pose;
    if(that_use_calib) 
      that_cam_pose = that->cameras[i]->calibrated_cam->pose();
    else 
      that_cam_pose = that->cameras[i]->guess_cam->pose();
    std::cout << "Camera " << i << ":\n" <<
                 gtsam::Pose3::Logmap(this_cam_pose.between(that_cam_pose)) << "\n";
  }
  for(int i = 0; i < this->targets.size(); ++i) {
    gtsam::Pose3 this_tgt_pose;
    if(this_use_calib) 
      this_tgt_pose = *(this->targets[i]->calibrated_pose);
    else 
      this_tgt_pose = *(this->targets[i]->guess_pose);
    gtsam::Pose3 that_tgt_pose;
    if(that_use_calib) 
      that_tgt_pose = *(that->targets[i]->calibrated_pose);
    else 
      that_tgt_pose = *(that->targets[i]->guess_pose);
    std::cout << "Target " << i << ":\n" <<
                 gtsam::Pose3::Logmap(this_tgt_pose.between(that_tgt_pose)) << "\n";
  }
}
}

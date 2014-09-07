
#include <extrinsic_calibration/extrinsic_calibration.h>
#include <extrinsic_calibration/factors.h>

using namespace gtsam;

namespace extrinsic_calibration 
{
void cameraArmTargetCalibrate(CalibrationJobPtr& cal_job, double pixel_sigma)
{
  noiseModel::Isotropic::shared_ptr pixel_fact_noise = noiseModel::Isotropic::Sigma(2, pixel_sigma);

  gtsam::NonlinearFactorGraph graph;
  Values init_estimate;

  for(int i = 0; i < cal_job->setup->cameras.size(); ++i) {
    gtsam::Pose3 cam_pose = cal_job->setup->cameras[i]->guess_cam->pose();
    init_estimate.insert(Symbol('c', i), cam_pose);
  }

  for(int i = 0; i < cal_job->setup->targets.size(); ++i) {
    gtsam::Pose3 tgt_pose = *(cal_job->setup->targets[i]->guess_pose);
    init_estimate.insert(Symbol('t', i), tgt_pose);
  }

  BOOST_FOREACH(shared_ptr<Scene> scene, cal_job->scenes) {
    BOOST_FOREACH(shared_ptr<Detection> detect, scene->detections) {
      for(int i = 0; i < detect->image_pts.size(); ++i) {
        shared_ptr<Camera> cam = cal_job->setup->cameras[detect->camera_ind];
        shared_ptr<Target> tgt = cal_job->setup->targets[detect->target_ind];
        Cal3_S2 calib_cam = cam->guess_cam->calibration();
        Pose3 pose_ee = scene->arm_pose;
        Point3 pt_tgt = tgt->target_pts[i];
        Point2 pt_img = detect->image_pts[i];
        graph.add(boost::make_shared<CameraArmTargetFactor>(
              pixel_fact_noise, 
              Symbol('c', detect->camera_ind), Symbol('t', detect->target_ind),  
              calib_cam, pose_ee, pt_tgt, pt_img));
      }
    }
  }
  printf("start error: %f\n", graph.error(init_estimate));
  Values result = DoglegOptimizer(graph, init_estimate).optimize();

  for(int i = 0; i < cal_job->setup->cameras.size(); ++i) {
    gtsam::Pose3 cam_pose = result.at<gtsam::Pose3>(Symbol('c', i));
    gtsam::Cal3_S2 cam_calib = cal_job->setup->cameras[i]->guess_cam->calibration();
    cal_job->setup->cameras[i]->calibrated_cam.reset(
        new gtsam::SimpleCamera(cam_pose, cam_calib));
  }

  for(int i = 0; i < cal_job->setup->targets.size(); ++i) {
    gtsam::Pose3 tgt_pose = result.at<gtsam::Pose3>(Symbol('t', i));
    cal_job->setup->targets[i]->calibrated_pose.reset(
        new gtsam::Pose3(tgt_pose));
  }

  result.print();
  printf("end error: %f\n", graph.error(result));
}

void cameraArmTargetPointCalibrate(CalibrationJobPtr& cal_job, double pixel_sigma, 
                                   double pt_xy_sigma, double pt_z_sigma)
{
  noiseModel::Isotropic::shared_ptr pixel_fact_noise = 
                noiseModel::Isotropic::Sigma(2, pixel_sigma);
  noiseModel::Diagonal::shared_ptr point_fact_noise = 
                noiseModel::Diagonal::Sigmas(Vector3(pt_xy_sigma, pt_xy_sigma, pt_z_sigma));

  gtsam::NonlinearFactorGraph graph;
  Values init_estimate;

  for(int i = 0; i < cal_job->setup->cameras.size(); ++i) {
    gtsam::Pose3 cam_pose = cal_job->setup->cameras[i]->guess_cam->pose();
    init_estimate.insert(Symbol('c', i), cam_pose);
  }

  int cur_pt_ind = 0;
  for(int i = 0; i < cal_job->setup->targets.size(); ++i) {
    gtsam::Pose3 tgt_pose = *(cal_job->setup->targets[i]->guess_pose);
    init_estimate.insert(Symbol('t', i), tgt_pose);

    for(int j = 0; j < cal_job->setup->targets[i]->target_pts.size(); ++j) {
      gtsam::Point3 tgt_pt = cal_job->setup->targets[i]->target_pts[j];
      graph.add(boost::make_shared<PriorFactor<Point3> >(
            Symbol('p', cur_pt_ind), tgt_pt, point_fact_noise));
      init_estimate.insert(Symbol('p', cur_pt_ind), tgt_pt);
      cur_pt_ind++;
    }
  }

  BOOST_FOREACH(shared_ptr<Scene> scene, cal_job->scenes) {
    BOOST_FOREACH(shared_ptr<Detection> detect, scene->detections) {
      shared_ptr<Camera> cam = cal_job->setup->cameras[detect->camera_ind];
      shared_ptr<Target> tgt = cal_job->setup->targets[detect->target_ind];
      Cal3_S2 calib_cam = cam->guess_cam->calibration();
      Pose3 pose_ee = scene->arm_pose;

      int pt_ind_offset = 0;
      for(int i = 0; i < detect->target_ind; ++i) 
        pt_ind_offset += cal_job->setup->targets[i]->target_pts.size();

      for(int i = 0; i < detect->image_pts.size(); ++i) {
        Point3 pt_tgt = tgt->target_pts[i];
        Point2 pt_img = detect->image_pts[i];
        graph.add(boost::make_shared<CameraArmTargetPointFactor>(
              pixel_fact_noise, 
              Symbol('c', detect->camera_ind), Symbol('t', detect->target_ind), 
              Symbol('p', pt_ind_offset + i),
              calib_cam, pose_ee, pt_img));
      }
    }
  }
  printf("start error: %f\n", graph.error(init_estimate));
  Values result = DoglegOptimizer(graph, init_estimate).optimize();

  for(int i = 0; i < cal_job->setup->cameras.size(); ++i) {
    gtsam::Pose3 cam_pose = result.at<gtsam::Pose3>(Symbol('c', i));
    gtsam::Cal3_S2 cam_calib = cal_job->setup->cameras[i]->guess_cam->calibration();
    cal_job->setup->cameras[i]->calibrated_cam.reset(
        new gtsam::SimpleCamera(cam_pose, cam_calib));
  }

  for(int i = 0; i < cal_job->setup->targets.size(); ++i) {
    gtsam::Pose3 tgt_pose = result.at<gtsam::Pose3>(Symbol('t', i));
    cal_job->setup->targets[i]->calibrated_pose.reset(
        new gtsam::Pose3(tgt_pose));
  }

  result.print();
  printf("end error: %f\n", graph.error(result));
}

void cameraArmTargetDetectPointCalibrate(CalibrationJobPtr& cal_job, double pixel_sigma, 
                                         double pt_xy_sigma, double pt_z_sigma)
{
  noiseModel::Isotropic::shared_ptr pixel_fact_noise = 
                noiseModel::Isotropic::Sigma(2, pixel_sigma);
  noiseModel::Diagonal::shared_ptr point_fact_noise = 
                noiseModel::Diagonal::Sigmas(Vector3(pt_xy_sigma, pt_xy_sigma, pt_z_sigma));

  gtsam::NonlinearFactorGraph graph;
  Values init_estimate;

  for(int i = 0; i < cal_job->setup->cameras.size(); ++i) {
    gtsam::Pose3 cam_pose = cal_job->setup->cameras[i]->guess_cam->pose();
    init_estimate.insert(Symbol('c', i), cam_pose);
  }

  for(int i = 0; i < cal_job->setup->targets.size(); ++i) {
    gtsam::Pose3 tgt_pose = *(cal_job->setup->targets[i]->guess_pose);
    init_estimate.insert(Symbol('t', i), tgt_pose);
  }

  int cur_pt_ind = 0;
  BOOST_FOREACH(shared_ptr<Scene> scene, cal_job->scenes) {
    BOOST_FOREACH(shared_ptr<Detection> detect, scene->detections) {
      shared_ptr<Camera> cam = cal_job->setup->cameras[detect->camera_ind];
      shared_ptr<Target> tgt = cal_job->setup->targets[detect->target_ind];
      Cal3_S2 calib_cam = cam->guess_cam->calibration();
      Pose3 pose_ee = scene->arm_pose;

      int pt_ind_offset = 0;
      for(int i = 0; i < detect->target_ind; ++i) 
        pt_ind_offset += cal_job->setup->targets[i]->target_pts.size();

      for(int i = 0; i < detect->image_pts.size(); ++i) {
        Point3 pt_tgt = tgt->target_pts[i];
        Point2 pt_img = detect->image_pts[i];
        graph.add(boost::make_shared<CameraArmTargetPointFactor>(
              pixel_fact_noise, 
              Symbol('c', detect->camera_ind), Symbol('t', detect->target_ind), 
              Symbol('p', cur_pt_ind),
              calib_cam, pose_ee, pt_img));

        graph.add(boost::make_shared<PriorFactor<Point3> >(
              Symbol('p', cur_pt_ind), pt_tgt, point_fact_noise));
        init_estimate.insert(Symbol('p', cur_pt_ind), pt_tgt);
        cur_pt_ind++;
      }
    }
  }
  printf("start error: %f\n", graph.error(init_estimate));
  Values result = DoglegOptimizer(graph, init_estimate).optimize();

  for(int i = 0; i < cal_job->setup->cameras.size(); ++i) {
    gtsam::Pose3 cam_pose = result.at<gtsam::Pose3>(Symbol('c', i));
    gtsam::Cal3_S2 cam_calib = cal_job->setup->cameras[i]->guess_cam->calibration();
    cal_job->setup->cameras[i]->calibrated_cam.reset(
        new gtsam::SimpleCamera(cam_pose, cam_calib));
  }

  for(int i = 0; i < cal_job->setup->targets.size(); ++i) {
    gtsam::Pose3 tgt_pose = result.at<gtsam::Pose3>(Symbol('t', i));
    cal_job->setup->targets[i]->calibrated_pose.reset(
        new gtsam::Pose3(tgt_pose));
  }

  result.print();
  printf("end error: %f\n", graph.error(result));
}
}

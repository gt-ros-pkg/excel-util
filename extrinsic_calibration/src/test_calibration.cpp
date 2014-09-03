
#include <extrinsic_calibration/calibration_job.h>
#include <extrinsic_calibration/factors.h>

using namespace extrinsic_calibration;

#define CAMERA_POS_RANGE 3.0
#define CAMERA_FOCAL_LOWER 300.0
#define CAMERA_FOCAL_UPPER 900.0
#define CAMERA_OFFSET_LOWER 300.0
#define CAMERA_OFFSET_UPPER 1000.0
#define TARGET_POS_RANGE 0.2
#define CAL_BOARD_NUM_ROW_LOWER 4
#define CAL_BOARD_NUM_ROW_UPPER 10
#define CAL_BOARD_NUM_COL_ADD 4
#define CAL_BOARD_SPACING_LOWER 0.08
#define CAL_BOARD_SPACING_UPPER 0.24
#define ROBOT_POS_RANGE 1.0

void randomCalibSetup(boost::mt19937& rng, CalibrationSetup& cal_setup, const int num_cams, const int num_targets)
{
  boost::uniform_real<> pos_range(-CAMERA_POS_RANGE, CAMERA_POS_RANGE);
  boost::uniform_real<> focal_range(CAMERA_FOCAL_LOWER, CAMERA_FOCAL_UPPER);
  boost::uniform_real<> offset_range(CAMERA_OFFSET_LOWER, CAMERA_OFFSET_UPPER);
  boost::uniform_real<> target_pos_range(-TARGET_POS_RANGE, TARGET_POS_RANGE);
  boost::uniform_int<> cal_board_row_dist(CAL_BOARD_NUM_ROW_LOWER, CAL_BOARD_NUM_ROW_UPPER);
  boost::uniform_int<> cal_board_col_dist(0, CAL_BOARD_NUM_COL_ADD);
  boost::uniform_real<> cal_board_spacing_range(CAL_BOARD_SPACING_LOWER, CAL_BOARD_SPACING_UPPER);

  cal_setup.cameras.clear();
  cal_setup.targets.clear();
  for(int i=0;i<num_cams;i++) {
    gtsam::Pose3 cam_pose(gtsam::Rot3::Random(rng),
                          gtsam::Point3(pos_range(rng),pos_range(rng),pos_range(rng)));
    std::cout << "cam_pose:\n" << cam_pose << std::endl;
    gtsam::Cal3_S2 cam_calib(focal_range(rng), focal_range(rng), 0.0, 
                             offset_range(rng), offset_range(rng));
    std::string cam_name = "/camera_" + boost::lexical_cast<std::string>(i+1);
    shared_ptr<Camera> cam = shared_ptr<Camera>(new Camera(cam_name, cam_name + "_guess"));
    cam->calibrated_cam.reset(new gtsam::SimpleCamera(cam_pose, cam_calib));
    cal_setup.cameras.push_back(cam);
  }
  for(int i=0;i<num_targets;i++) {
    std::string target_name = "/target_" + boost::lexical_cast<std::string>(i+1);
    shared_ptr<Target> target = shared_ptr<Target>(new Target(target_name, target_name + "_guess"));
    target->calibrated_pose.reset(new gtsam::Pose3(gtsam::Rot3::Random(rng),
                                                   gtsam::Point3(target_pos_range(rng),
                                                                 target_pos_range(rng),
                                                                 target_pos_range(rng))));
    std::cout << "target_pose:\n" << *(target->calibrated_pose) << std::endl;
    int num_rows = cal_board_row_dist(rng);
    int num_cols = num_rows + cal_board_col_dist(rng);
    double cal_board_spacing = cal_board_spacing_range(rng);
    for(int j=0;j<num_rows;j++) 
      for(int k=0;k<num_cols;k++) 
        target->target_pts.push_back(gtsam::Point3(cal_board_spacing*j, cal_board_spacing*k, 0.0));
    cal_setup.targets.push_back(target);
  }
}

void randomCalibJob(boost::mt19937& rng, CalibrationSetup& cal_truth, CalibrationJob& cal_job, int num_scenes)
{
  boost::uniform_real<> pos_range(-ROBOT_POS_RANGE, ROBOT_POS_RANGE);
  boost::uniform_int<> num_detect_dist(1, cal_truth.cameras.size()*cal_truth.targets.size());
  boost::uniform_int<> camera_ind_dist(0, cal_truth.cameras.size()-1);
  boost::uniform_int<> target_ind_dist(0, cal_truth.targets.size()-1);

  cal_job.setup.reset(new CalibrationSetup(cal_truth.nh, cal_truth.world_frame, cal_truth.ee_frame));
  BOOST_FOREACH(shared_ptr<Camera> cam, cal_truth.cameras) {
    shared_ptr<Camera> new_cam = shared_ptr<Camera>(
        new Camera(cam->calibrated_frame, cam->guess_frame));
    gtsam::SimpleCamera calib_cam(*(cam->calibrated_cam));
    new_cam->guess_cam.reset(new gtsam::SimpleCamera(calib_cam));
    cal_job.setup->cameras.push_back(new_cam);
  }
  BOOST_FOREACH(shared_ptr<Target> tgt, cal_truth.targets) {
    shared_ptr<Target> new_tgt = shared_ptr<Target>(
        new Target(tgt->calibrated_frame, tgt->guess_frame));
    gtsam::Pose3 calib_tgt(*(tgt->calibrated_pose));
    new_tgt->guess_pose.reset(new gtsam::Pose3(calib_tgt));
    BOOST_FOREACH(gtsam::Point3 pt, tgt->target_pts) {
      new_tgt->target_pts.push_back(pt);
    }
    cal_job.setup->targets.push_back(new_tgt);
  }
  noiseModel::Isotropic::shared_ptr img_point_noise = noiseModel::Isotropic::Sigma(2, 5);
  gtsam::Sampler img_point_smplr(img_point_noise, 333);

  for(int i = 0; i < num_scenes; ++i) {
    while(ros::ok()) {
      try {
        shared_ptr<Scene> scene(new Scene);
        scene->arm_pose = gtsam::Pose3(gtsam::Rot3::Random(rng),
                                       gtsam::Point3(pos_range(rng),pos_range(rng),pos_range(rng)));
        int num_detects = num_detect_dist(rng);
        for(int j = 0; j < num_detects; ++j) {
          shared_ptr<Detection> detect(new Detection);
          detect->camera_ind = camera_ind_dist(rng);
          detect->target_ind = target_ind_dist(rng);
          shared_ptr<Camera> cam = cal_truth.cameras[detect->camera_ind];
          shared_ptr<Target> target = cal_truth.targets[detect->target_ind];
          BOOST_FOREACH(gtsam::Point3 pt_tar, target->target_pts) {
            gtsam::Point3 pt_ee = target->calibrated_pose->transform_to(pt_tar);
            gtsam::Point3 pt_wl = scene->arm_pose.transform_from(pt_ee);
            gtsam::Point2 pt_img = cam->calibrated_cam->project(pt_wl);
            pt_img = pt_img + img_point_smplr.sample();
            detect->image_pts.push_back(pt_img);
          }
          scene->detections.push_back(detect);
        }
        cal_job.scenes.push_back(scene);
        break;
      }
      catch (gtsam::CheiralityException e) {}
    }
  }
}

void testFactorError(CalibrationJob& cal_job)
{
  noiseModel::Isotropic::shared_ptr mp_fact_noise = noiseModel::Isotropic::Sigma(2, 0.2);

  noiseModel::Isotropic::shared_ptr init_calib_noise = noiseModel::Isotropic::Sigma(6, 0.1);
  gtsam::Sampler init_calib_smplr(init_calib_noise, 333);

  gtsam::NonlinearFactorGraph graph;
  Values init_estimate;

  for(int i = 0; i < cal_job.setup->cameras.size(); ++i) {
    gtsam::Pose3 cam_pose = cal_job.setup->cameras[i]->guess_cam->pose();
    cam_pose = cam_pose * Pose3::Expmap(init_calib_smplr.sample());
    init_estimate.insert(Symbol('c', i), cam_pose);
  }

  for(int i = 0; i < cal_job.setup->targets.size(); ++i) {
    gtsam::Pose3 tgt_pose = *(cal_job.setup->targets[i]->guess_pose);
    tgt_pose = tgt_pose * Pose3::Expmap(init_calib_smplr.sample());
    init_estimate.insert(Symbol('t', i), tgt_pose);
  }

  BOOST_FOREACH(shared_ptr<Scene> scene, cal_job.scenes) {
    BOOST_FOREACH(shared_ptr<Detection> detect, scene->detections) {
      for(int i = 0; i < detect->image_pts.size(); ++i) {
        shared_ptr<Camera> cam = cal_job.setup->cameras[detect->camera_ind];
        shared_ptr<Target> tgt = cal_job.setup->targets[detect->target_ind];
        Cal3_S2 calib_cam = cam->guess_cam->calibration();
        Pose3 pose_ee = scene->arm_pose;
        Point3 pt_tgt = tgt->target_pts[i];
        Point2 pt_img = detect->image_pts[i];
        graph.add(boost::make_shared<CameraArmTargetFactor>(
              mp_fact_noise, 
              Symbol('c',detect->camera_ind), Symbol('t',detect->target_ind),  
              calib_cam, pose_ee, pt_tgt, pt_img));
      }
    }
  }
  Values result = DoglegOptimizer(graph, init_estimate).optimize();
  result.print();
  printf("start error: %f\n", graph.error(init_estimate));
  printf("end error: %f\n", graph.error(result));
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_calibration");
  int seed = boost::lexical_cast<int>(argv[1]);
  int num_cams = 2;
  int num_targets = 1;
  int num_scenes = 4;

  boost::mt19937 rng(seed);

  ros::NodeHandle nh;
  CalibrationSetup cal_setup(nh, "/world_frame", "/ee_link");
  randomCalibSetup(rng, cal_setup, num_cams, num_targets);
  CalibrationJob cal_job;
  randomCalibJob(rng, cal_setup, cal_job, num_scenes);
  testFactorError(cal_job);
  ros::spin();
}

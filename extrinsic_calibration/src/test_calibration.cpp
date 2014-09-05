
#include <extrinsic_calibration/calibration_job.h>
#include <extrinsic_calibration/extrinsic_calibration.h>

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
#define CALIB_OFFSET_NOISE 0.1
#define IMAGE_NOISE 5

int seed;

void randomCalibSetup(boost::mt19937& rng, CalibrationSetup& cal_setup, const int num_cams, const int num_targets)
{
  boost::uniform_real<> pos_range(-CAMERA_POS_RANGE, CAMERA_POS_RANGE);
  boost::uniform_real<> focal_range(CAMERA_FOCAL_LOWER, CAMERA_FOCAL_UPPER);
  boost::uniform_real<> offset_range(CAMERA_OFFSET_LOWER, CAMERA_OFFSET_UPPER);
  boost::uniform_real<> target_pos_range(-TARGET_POS_RANGE, TARGET_POS_RANGE);
  boost::uniform_int<> cal_board_row_dist(CAL_BOARD_NUM_ROW_LOWER, CAL_BOARD_NUM_ROW_UPPER);
  boost::uniform_int<> cal_board_col_dist(1, CAL_BOARD_NUM_COL_ADD);
  boost::uniform_real<> cal_board_spacing_range(CAL_BOARD_SPACING_LOWER, CAL_BOARD_SPACING_UPPER);

  cal_setup.cameras.clear();
  cal_setup.targets.clear();
  for(int i=0;i<num_cams;i++) {
    // gtsam::Pose3 cam_pose(gtsam::Rot3::Random(rng),
    //                       gtsam::Point3(pos_range(rng),pos_range(rng),pos_range(rng)));
    gtsam::Pose3 start_pose(gtsam::Rot3::ypr(0.0, 1.57, 0.0),
                            gtsam::Point3(-2.5, 0.0, 0.0));
    gtsam::Pose3 rot_pose(gtsam::Rot3::ypr(i*3.14*2.0/num_cams, 0.0, 0.0),
                          gtsam::Point3(0.0, 0.0, 0.0));
    gtsam::Pose3 cam_pose = rot_pose*start_pose;
    gtsam::Cal3_S2 cam_calib(focal_range(rng), focal_range(rng), 0.0, 
                             offset_range(rng), offset_range(rng));
    std::string cam_name = "/camera_" + boost::lexical_cast<std::string>(i+1);
    shared_ptr<Camera> cam = shared_ptr<Camera>(new Camera);
    cam->frame = cam_name;
    cam->calibrated_cam.reset(new gtsam::SimpleCamera(cam_pose, cam_calib));
    cal_setup.cameras.push_back(cam);
  }
  for(int i=0;i<num_targets;i++) {
    std::string target_name = "/target_" + boost::lexical_cast<std::string>(i+1);
    shared_ptr<Target> target = shared_ptr<Target>(new Target);
    target->frame = target_name;
    target->calibrated_pose.reset(new gtsam::Pose3(gtsam::Rot3::Random(rng),
                                                   gtsam::Point3(target_pos_range(rng),
                                                                 target_pos_range(rng),
                                                                 target_pos_range(rng))));
    // std::cout << "target_pose:\n" << *(target->calibrated_pose) << std::endl;
    int num_rows = cal_board_row_dist(rng);
    int num_cols = num_rows + cal_board_col_dist(rng);
    double cal_board_spacing = cal_board_spacing_range(rng);
    for(int j=0;j<num_rows;j++) 
      for(int k=0;k<num_cols;k++) 
        target->target_pts.push_back(gtsam::Point3(cal_board_spacing*j, cal_board_spacing*k, 0.0));
    target->type = "chessboard";
    target->pt_spacing = cal_board_spacing;
    target->rows = num_rows;
    target->cols = num_cols;
    cal_setup.targets.push_back(target);
  }
}

void randomCalibJob(boost::mt19937& rng, CalibrationSetup& cal_truth, CalibrationJobPtr& cal_job, int num_scenes)
{
  boost::uniform_real<> pos_range(-ROBOT_POS_RANGE, ROBOT_POS_RANGE);
  boost::uniform_int<> num_detect_dist(1, cal_truth.cameras.size()*cal_truth.targets.size());
  boost::uniform_int<> camera_ind_dist(0, cal_truth.cameras.size()-1);
  boost::uniform_int<> target_ind_dist(0, cal_truth.targets.size()-1);

  gtsam::noiseModel::Isotropic::shared_ptr init_calib_noise = 
                gtsam::noiseModel::Isotropic::Sigma(6, CALIB_OFFSET_NOISE);
  gtsam::Sampler init_calib_smplr(init_calib_noise, seed);
  gtsam::noiseModel::Isotropic::shared_ptr img_point_noise = 
                gtsam::noiseModel::Isotropic::Sigma(2, IMAGE_NOISE);
  gtsam::Sampler img_point_smplr(img_point_noise, seed);

  cal_job->setup.reset(new CalibrationSetup);
  cal_job->setup->world_frame = cal_truth.world_frame;
  cal_job->setup->ee_frame = cal_truth.ee_frame;
  BOOST_FOREACH(shared_ptr<Camera> cam, cal_truth.cameras) {
    shared_ptr<Camera> new_cam = shared_ptr<Camera>(
        new Camera);
    new_cam->frame = cam->frame;
    gtsam::Pose3 cam_pose = cam->calibrated_cam->pose();
    cam_pose = cam_pose * gtsam::Pose3::Expmap(init_calib_smplr.sample());
    new_cam->guess_cam.reset(new gtsam::SimpleCamera(cam_pose, cam->calibrated_cam->calibration()));
    cal_job->setup->cameras.push_back(new_cam);
  }
  BOOST_FOREACH(shared_ptr<Target> tgt, cal_truth.targets) {
    shared_ptr<Target> new_tgt = shared_ptr<Target>(
        new Target);
    new_tgt->frame = tgt->frame;
    gtsam::Pose3 tgt_pose = (*(tgt->calibrated_pose))*gtsam::Pose3::Expmap(init_calib_smplr.sample());
    new_tgt->guess_pose.reset(new gtsam::Pose3(tgt_pose));
    BOOST_FOREACH(gtsam::Point3 pt, tgt->target_pts) {
      new_tgt->target_pts.push_back(pt);
    }
    new_tgt->type = tgt->type;
    new_tgt->pt_spacing = tgt->pt_spacing;
    new_tgt->rows = tgt->rows;
    new_tgt->cols = tgt->cols;
    cal_job->setup->targets.push_back(new_tgt);
  }

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
        cal_job->scenes.push_back(scene);
        break;
      }
      catch (gtsam::CheiralityException e) {}
    }
  }
}

void testReadWriteCalibSetup(ros::NodeHandle& nh, CalibrationSetupPtr& write_cal_setup)
{
  write_cal_setup->writeToParam(nh, "/test_cal_setup");
  CalibrationSetupPtr read_cal_setup = CalibrationSetup::readFromParam(nh, "/test_cal_setup");
  ROS_ASSERT(read_cal_setup->world_frame == write_cal_setup->world_frame);
  ROS_ASSERT(read_cal_setup->ee_frame == write_cal_setup->ee_frame);
  ROS_ASSERT(read_cal_setup->cameras.size() == write_cal_setup->cameras.size());
  ROS_ASSERT(read_cal_setup->targets.size() == write_cal_setup->targets.size());
  for(int i = 0; i < read_cal_setup->cameras.size(); ++i) {
    shared_ptr<Camera> read_cam = read_cal_setup->cameras[i];
    shared_ptr<Camera> write_cam = write_cal_setup->cameras[i];
    ROS_ASSERT(read_cam->frame == write_cam->frame);
    ROS_ASSERT(read_cam->guess_cam->equals(*(write_cam->guess_cam)));
    ROS_ASSERT(read_cam->calibrated_cam->equals(*(write_cam->calibrated_cam)));
  }
  for(int i = 0; i < read_cal_setup->targets.size(); ++i) {
    shared_ptr<Target> read_tgt = read_cal_setup->targets[i];
    shared_ptr<Target> write_tgt = write_cal_setup->targets[i];
    ROS_ASSERT(read_tgt->frame == write_tgt->frame);
    ROS_ASSERT(read_tgt->type == write_tgt->type);
    ROS_ASSERT(read_tgt->pt_spacing == write_tgt->pt_spacing);
    ROS_ASSERT(read_tgt->rows == write_tgt->rows);
    ROS_ASSERT(read_tgt->cols == write_tgt->cols);
    ROS_ASSERT(read_tgt->guess_pose->equals(*(write_tgt->guess_pose)));
    ROS_ASSERT(read_tgt->calibrated_pose->equals(*(write_tgt->calibrated_pose)));
  }
}

void testReadWriteCalibJob(ros::NodeHandle& nh, CalibrationJobPtr& write_cal_job)
{
  write_cal_job->writeToParam(nh, "/test_cal_job");
  write_cal_job->setup->writeToParam(nh, "/test_cal_setup");
  CalibrationJobPtr read_cal_job = CalibrationJob::readFromParam(nh, "/test_cal_setup", "/test_cal_job");
  ROS_ASSERT(read_cal_job->scenes.size() == write_cal_job->scenes.size());
  for(int i = 0; i < read_cal_job->scenes.size(); ++i) {
    shared_ptr<Scene> read_scene = read_cal_job->scenes[i];
    shared_ptr<Scene> write_scene = write_cal_job->scenes[i];
    ROS_ASSERT(read_scene->arm_pose.equals(write_scene->arm_pose));

    ROS_ASSERT(read_scene->detections.size() == read_scene->detections.size());
    for(int j = 0; j < read_scene->detections.size(); ++j) {
      shared_ptr<Detection> read_detect = read_scene->detections[j];
      shared_ptr<Detection> write_detect = write_scene->detections[j];
      ROS_ASSERT(read_detect->camera_ind == write_detect->camera_ind);
      ROS_ASSERT(read_detect->target_ind == write_detect->target_ind);
      for(int k = 0; k < read_detect->image_pts.size(); ++k) 
        ROS_ASSERT(read_detect->image_pts[k].equals(write_detect->image_pts[k]));
    }
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_calibration");
  seed = boost::lexical_cast<int>(argv[1]);
  int num_cams = 5;
  int num_targets = 1;
  int num_scenes = 10;

  boost::mt19937 rng(seed);

  ros::NodeHandle nh;
  CalibrationSetup cal_setup;
  cal_setup.world_frame = "/world_frame";
  cal_setup.ee_frame = "/ee_link";
  randomCalibSetup(rng, cal_setup, num_cams, num_targets);
  CalibrationJobPtr cal_job = CalibrationJobPtr(new CalibrationJob);
  randomCalibJob(rng, cal_setup, cal_job, num_scenes);
  // cameraArmTargetCalibrate(cal_job, 5.0);
  cameraArmTargetPointCalibrate(cal_job);
  std::cout << "Guess calibration difference:\n";
  cal_job->setup->diffCalib(cal_job->setup, false, true);
  std::cout << "After calibration difference:\n";
  cal_setup.diffCalib(cal_job->setup, true, true);
  testReadWriteCalibSetup(nh, cal_job->setup);
  testReadWriteCalibJob(nh, cal_job);

  CalibrationBroadcaster cal_broad(nh, cal_job->setup);
  ros::spin();
}

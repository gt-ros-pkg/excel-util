#ifndef EXTRINSIC_CALIBRATION_FACTORS_H
#define EXTRINSIC_CALIBRATION_FACTORS_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/SimpleCamera.h>

using namespace gtsam;

class CameraArmTargetFactor : public NoiseModelFactor2<Pose3, Pose3>
{
  public:
    Cal3_S2 calib_cam;
    Pose3 pose_ee;
    Point3 pt_tgt;
    Point2 pt_img;
    CameraArmTargetFactor(const SharedNoiseModel& noiseModel, 
                          Key j1, Key j2, Cal3_S2 _calib_cam, Pose3 _pose_ee, 
                                          Point3 _pt_tgt, Point2 _pt_img) :
      NoiseModelFactor2<Pose3, Pose3>(noiseModel, j1, j2), 
        calib_cam(_calib_cam), pose_ee(_pose_ee), pt_tgt(_pt_tgt), pt_img(_pt_img)
      {
        // calib_cam.print();
        // std::cout << "pose_ee:" << std::endl << pose_ee << std::endl;
        // std::cout << "pt_tgt:" << std::endl << pt_tgt << std::endl;
        // std::cout << "pt_img:" << std::endl << pt_img << std::endl;
      }

    Vector evaluateError(const Pose3& pose_cam,
                         const Pose3& pose_tgt,
                         boost::optional<Matrix&> H_1 = boost::none,
                         boost::optional<Matrix&> H_2 = boost::none) const
    {
      Matrix Dtgt_pose;
      Matrix Dtgt_point;
      Point3 pt_ee = pose_tgt.transform_to(pt_tgt, Dtgt_pose, Dtgt_point);
      Matrix Dee_pose;
      Matrix Dee_point;
      Point3 pt_wl = pose_ee.transform_from(pt_ee, Dee_pose, Dee_point);
      SimpleCamera cam(pose_cam, calib_cam);
      Matrix Dcam_pose;
      Matrix Dcam_point;
      Point2 pt_img_est = cam.project(pt_wl, Dcam_pose, Dcam_point);
      // std::cout << "yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy" << std::endl;
      // cam.print();
      // std::cout << Dcam_pose << std::endl;
      // std::cout << Dcam_point << std::endl;
      // std::cout << Dtgt_pose << std::endl;
      // std::cout << "nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn" << std::endl;
      if(H_1) {
        (*H_1) = Dcam_pose;
      }
      if(H_2) {
        (*H_2) = Dcam_point*Dee_point*Dtgt_pose;
      }
      return (pt_img_est - pt_img).vector();
    }
};

// class PointTransFactor : public NoiseModelFactor1<Pose3>
// {
//   public:
//     Point3 cb_p_point, kinect_p_point;
//     PointTransFactor(const gtsam::SharedNoiseModel& noiseModel, 
//                           Key j1, Point3 _cb_p_point, Point3 _kinect_p_point) :
//       NoiseModelFactor1<Pose3>(noiseModel, j1), 
//         cb_p_point(_cb_p_point), kinect_p_point(_kinect_p_point) {}
// 
//     gtsam::Vector evaluateError(const gtsam::Pose3& cb_T_kinect,
//                                 boost::optional<Matrix&> H_1 = boost::none) const
//     {
//       if(H_1) {
//         (*H_1) = gtsam::Matrix_(3, 6);
//         Point3 pt = cb_T_kinect.inverse() * cb_p_point;
//         Matrix cross = Matrix_(3,3,
//                                0.0,-pt.z(),pt.y(),
//                                pt.z(),0.0,-pt.x(),
//                                -pt.y(),pt.x(),0.0);
//         (*H_1).block<3,3>(0,0) = cross;
//         (*H_1).block<3,3>(0,3) = -eye(3);
//       }
//       return (cb_T_kinect.inverse() * cb_p_point - kinect_p_point).vector();
//     }
// };

#endif

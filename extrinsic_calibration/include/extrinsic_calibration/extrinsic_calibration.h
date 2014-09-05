#ifndef EXTRINSIC_CALIBRATION_EXTRINSIC_CALIBRATION_H
#define EXTRINSIC_CALIBRATION_EXTRINSIC_CALIBRATION_H

#include <extrinsic_calibration/calibration_job.h>


namespace extrinsic_calibration 
{

void cameraArmTargetCalibrate(CalibrationJobPtr& cal_job, double pixel_sigma=3.0);

// target points are allowed to move around
void cameraArmTargetPointCalibrate(CalibrationJobPtr& cal_job, double pixel_sigma=3.0, 
                                   double pt_xy_sigma=0.001, double pt_z_sigma=0.015);

// target points are allowed to move around for each detection
void cameraArmTargetDetectPointCalibrate(CalibrationJobPtr& cal_job, double pixel_sigma=3.0, 
                                         double pt_xy_sigma=0.001, double pt_z_sigma=0.015);
}

#endif

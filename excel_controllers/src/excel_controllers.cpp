
#include <pluginlib/class_list_macros.h>

#include <joint_trajectory_controller/hardware_interface_adapter.h>
#include <trajectory_interface/pos_vel_acc_state.h>
#include <hardware_interface/pos_vel_acc_joint_interface.h>

#include <trajectory_interface/quintic_spline_segment.h>
#include <jogging_command_controller/jogging_command_controller.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>

namespace vel_fwd_pva_fwd_controllers
{
  typedef trajectory_interface::PosVelAccState<double> State;
  typedef VelocityForwardHardwareInterfaceAdapter<State> HwIfaceAdapter1;
  typedef PosVelAccForwardHardwareInterfaceAdapter<State> HwIfaceAdapter2;
  typedef NaiveHardwareInterfaceAdapter2<HwIfaceAdapter1, HwIfaceAdapter2, State>
          VelocityForwardPosVelAccForwardHwIfaceAdapter;

  typedef jogging_command_controller::
          JoggingCommandController2<State, VelocityForwardPosVelAccForwardHwIfaceAdapter>
          JoggingCommandController;

  typedef trajectory_interface::QuinticSplineSegment<double> SegmentImpl;
  typedef joint_trajectory_controller::
          JointTrajectoryController2<SegmentImpl, VelocityForwardPosVelAccForwardHwIfaceAdapter>
          JointTrajectoryController;
}

PLUGINLIB_EXPORT_CLASS(vel_fwd_pva_fwd_controllers::JoggingCommandController, 
                       controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(vel_fwd_pva_fwd_controllers::JointTrajectoryController, 
                       controller_interface::ControllerBase)


#include <ros/ros.h>
#include <urdf/model.h>
#include <controller_manager/controller_manager.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#ifdef XENOMAI_REALTIME

#include <sys/mman.h>
#include <rtdm/rtdm.h>
#include <native/task.h>
#include <native/sem.h>
#include <native/mutex.h>
#include <native/timer.h>
#include <rtdk.h>
#include <pthread.h>

#endif

#ifndef TEST_CTRL
#include <indradrive_hw_iface/vel_ec_ctrl.h>
typedef indradrive::VelocityEthercatController IDCSRobotHW;
#else
#include <indradrive_hw_iface/idcs_robot_hw.h>
typedef indradrive::IndradriveCSRobotHW IDCSRobotHW;
#endif

boost::shared_ptr<IDCSRobotHW> idcs_hw_ptr;
boost::shared_ptr<controller_manager::ControllerManager> cm_ptr;
bool stop_requested;

#ifdef XENOMAI_REALTIME

RT_TASK update_loop_task_info;

void signal_handler(int sig)
{
  stop_requested = true;
}

#endif

void update_loop_task(void *arg)
{
  ros::Duration period(1.0/1000.0);
  ros::Rate r(1000.0);
  ros::Time now = ros::Time::now();
#ifdef XENOMAI_REALTIME
	rt_task_set_periodic(NULL, TM_NOW, 1000000); // ns
#else
#endif
  while (ros::ok() && !stop_requested) {
#ifdef XENOMAI_REALTIME
		rt_task_wait_period(NULL);
#else
    r.sleep();
#endif
    idcs_hw_ptr->read(now, period);
    cm_ptr->update(now, period);
    idcs_hw_ptr->write(now, period);
    now = now + period;
  }
}

int main(int argc, char** argv)
{
  stop_requested = false;
  ros::init(argc, argv, "idcs_vel_ctrl_man");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  std::string joint_name;
  nh_priv.param<std::string>("joint_name", joint_name, "indradrive_cs_joint");

  urdf::Model urdf_model;
  if(!urdf_model.initParam("robot_description")) {
    ROS_ERROR("vel_ctrl_man requires a URDF in the robot_description parameter.");
    return -1;
  }

  joint_limits_interface::JointLimits idcs_limits;
  joint_limits_interface::SoftJointLimits idcs_soft_limits;
  boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model.getJoint(joint_name);
  bool urdf_found_limits = getJointLimits(urdf_joint, idcs_limits);
  bool param_srv_found_limits = getJointLimits(joint_name, nh_priv, idcs_limits);
  if(!urdf_found_limits && !param_srv_found_limits) {
    ROS_ERROR("Couldn't find limits for joint %s", joint_name.c_str());
    return -1;
  }
  idcs_soft_limits.min_position = idcs_limits.min_position;
  idcs_soft_limits.max_position = idcs_limits.max_position;
  idcs_soft_limits.k_position = 4.0;
  getSoftJointLimits(joint_name, nh_priv, idcs_soft_limits);

  idcs_hw_ptr.reset(new IDCSRobotHW(nh, nh_priv, joint_name, idcs_limits, idcs_soft_limits));
  if(idcs_hw_ptr->init()) {
    printf("Failed to initialize controller\n");
    return -1;
  }

  cm_ptr.reset(new controller_manager::ControllerManager(idcs_hw_ptr.get(), nh));

#ifdef XENOMAI_REALTIME
  int ret;
  rt_print_auto_init(1);

  signal(SIGTERM, signal_handler);
  signal(SIGINT, signal_handler);

  mlockall(MCL_CURRENT | MCL_FUTURE);

  ret = rt_task_create(&update_loop_task_info, "update_loop", 0, 80, T_FPU);
  if (ret < 0) {
    fprintf(stderr, "Failed to create task: %s\n", strerror(-ret));
    return -1;
  }

  printf("Starting update_loop_task_info...\n");
  ret = rt_task_start(&update_loop_task_info, &update_loop_task, NULL);
  if (ret < 0) {
    fprintf(stderr, "Failed to start task: %s\n", strerror(-ret));
    return -1;
  }

  while (ros::ok() && !stop_requested) {
    sched_yield();
  }

  printf("Deleting realtime task...\n");
  rt_task_delete(&update_loop_task_info);

#else
  update_loop_task(NULL);
#endif

}

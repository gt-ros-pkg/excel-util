
#include <ros/ros.h>
#include <urdf/model.h>
#include <controller_manager/controller_manager.h>
#include <ur_ctrl_client/ur_robot_hw.h>

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
boost::shared_ptr<ur::URRobotHW> ur_hw_ptr;
hardware_interface::RobotHW excel_hw;
bool stop_requested;

#ifdef XENOMAI_REALTIME

RT_TASK update_loop_task_info;

void signal_handler(int sig)
{
  stop_requested = true;
  ros::shutdown();
}

#endif

void update_loop_task(void *arg)
{
  ros::Duration period(1.0/1000.0);
#ifdef XENOMAI_REALTIME
	rt_task_set_periodic(NULL, TM_NOW, 1000000); // ns
#else
  ros::Rate r(1000.0);
#endif
  uint64_t counter = 0;
  while (ros::ok() && !stop_requested) {
#ifdef XENOMAI_REALTIME
		rt_task_wait_period(NULL);
#else
    r.sleep();
#endif
    idcs_hw_ptr->read();
    if(counter % 8 == 0) // 125 Hz
      ur_hw_ptr->read();
    cm_ptr->update(ros::Time::now(), period);
    idcs_hw_ptr->write();
    if(counter % 8 == 0) // 125 Hz
      ur_hw_ptr->write();
  }
}

int main(int argc, char** argv)
{
  stop_requested = false;
  ros::init(argc, argv, "excel_ctrl_man");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  std::string robot_ip;
  if(!nh_priv.getParam("robot_ip", robot_ip)) {
    ROS_ERROR("Missing robot IP address (robot_ip)");
    return -1;
  }
  XmlRpc::XmlRpcValue v;
  if(!nh_priv.getParam("joint_names", v) || v.size() != 7) {
    ROS_ERROR("Excel robot requires a list of the 7 joint names");
    return -1;
  }
  std::string idcs_joint_name = v[0];
  std::vector<std::string> ur_joint_names;
  for(int i=1;i<7;i++)
    ur_joint_names.push_back(v[i]);

  urdf::Model urdf_model;
  if(urdf_model.initParam("robot_description")) {
    ROS_ERROR("ur_ctrl_man requires a URDF in the robot_description parameter.");
    return -1;
  }

  joint_limits_interface::JointLimits ur_limits[6];
  joint_limits_interface::SoftJointLimits ur_soft_limits[6];
  for(int i=0;i<6;i++) {
    boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model.getJoint(ur_joint_names[i]);
    bool urdf_found_limits = getJointLimits(urdf_joint, ur_limits[i]);
    bool param_srv_found_limits = getJointLimits(ur_joint_names[i], nh_priv, ur_limits[i]);
    if(!urdf_found_limits && !param_srv_found_limits) {
      ROS_ERROR("Couldn't find limits for joint %s", ur_joint_names[i].c_str());
      return -1;
    }
    ur_soft_limits[i].min_position = ur_limits[i].min_position;
    ur_soft_limits[i].max_position = ur_limits[i].max_position;
    ur_soft_limits[i].k_position = 10.0;
    getSoftJointLimits(ur_joint_names[i], nh_priv, ur_soft_limits[i]);
  }

  ur_hw_ptr.reset(new ur::URRobotHW(nh, ur_joint_names, ur_limits, ur_soft_limits));

  joint_limits_interface::JointLimits idcs_limits;
  joint_limits_interface::SoftJointLimits idcs_soft_limits;
  boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model.getJoint(idcs_joint_name);
  bool urdf_found_limits = getJointLimits(urdf_joint, idcs_limits);
  bool param_srv_found_limits = getJointLimits(idcs_joint_name, nh_priv, idcs_limits);
  if(!urdf_found_limits && !param_srv_found_limits) {
    ROS_ERROR("Couldn't find limits for joint %s", idcs_joint_name.c_str());
    return -1;
  }
  idcs_soft_limits.min_position = idcs_limits.min_position;
  idcs_soft_limits.max_position = idcs_limits.max_position;
  idcs_soft_limits.k_position = 4.0;
  getSoftJointLimits(idcs_joint_name, nh_priv, idcs_soft_limits);

  idcs_hw_ptr.reset(new IDCSRobotHW(nh, nh_priv, idcs_joint_name, idcs_limits, idcs_soft_limits));

  excel_hw.registerInterfaceManager(idcs_hw_ptr.get());
  excel_hw.registerInterfaceManager(ur_hw_ptr.get());

  cm_ptr.reset(new controller_manager::ControllerManager(&excel_hw, nh));

  ur_hw_ptr->init(robot_ip);
  idcs_hw_ptr->init();

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

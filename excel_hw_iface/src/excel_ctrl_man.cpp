
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <indradrive_hw_iface/vel_ec_ctrl.h>
#include <ur_ctrl_client/ur_robot_hw.h>

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

typedef indradrive::VelocityEthercatController VelEcatCtrl;

boost::shared_ptr<VelEcatCtrl> cs_hw_ptr;
boost::shared_ptr<controller_manager::ControllerManager> cm_ptr;
boost::shared_ptr<ur::URRobotHW> ur_hw_ptr;
hardware_interface::RobotHW excel_hw;
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
    cs_hw_ptr->read();
    if(counter % 8 == 0) // 125 Hz
      ur_hw_ptr->read();
    cm_ptr->update(ros::Time::now(), period);
    cs_hw_ptr->write();
    if(counter % 8 == 0) // 125 Hz
      ur_hw_ptr->write();
  }
}

int main(int argc, char** argv)
{
  stop_requested = false;
  ros::init(argc, argv, "cs_controller_man");

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
  std::string indradrive_joint_name = v[0];
  std::vector<std::string> ur_joint_names;
  for(int i=1;i<7;i++)
    ur_joint_names.push_back(v[i]);

  cs_hw_ptr.reset(new VelEcatCtrl(nh, nh_priv, indradrive_joint_name));
  cs_hw_ptr->init();

  ur_hw_ptr.reset(new ur::URRobotHW(nh, ur_joint_names));
  ur_hw_ptr->init(robot_ip);

  excel_hw.registerInterfaceManager(cs_hw_ptr.get());
  excel_hw.registerInterfaceManager(ur_hw_ptr.get());

  cm_ptr.reset(new controller_manager::ControllerManager(&excel_hw, nh));

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

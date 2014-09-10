
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

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

boost::shared_ptr<IDCSRobotHW> cs_hw_ptr;
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
#ifdef XENOMAI_REALTIME
	rt_task_set_periodic(NULL, TM_NOW, 1000000); // ns
#else
  ros::Rate r(1000.0);
#endif
  while (ros::ok() && !stop_requested) {
#ifdef XENOMAI_REALTIME
		rt_task_wait_period(NULL);
#else
    r.sleep();
#endif
    cs_hw_ptr->read();
    cm_ptr->update(ros::Time::now(), period);
    cs_hw_ptr->write();
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

  cs_hw_ptr.reset(new IDCSRobotHW(nh, nh_priv, joint_name));
  if(cs_hw_ptr->init()) {
    printf("Failed to initialize controller\n");
    return -1;
  }

  cm_ptr.reset(new controller_manager::ControllerManager(cs_hw_ptr.get(), nh));

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

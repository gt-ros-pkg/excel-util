
#include <errno.h>
#include <math.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <map>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{

  pid_t pid = getpid();
  if (setpriority(PRIO_PROCESS, pid, -19))
    fprintf(stderr, "Warning: Failed to set priority: %s\n",
        strerror(errno));

  ros::init(argc, argv, "test_rt_prio");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  ros::spin();
}

#ifndef SCANNING_TEMPLATE_H
#define SCANNING_TEMPLATE_H

#include <ros/ros.h>
#include <vector>
#include <string>

class Scanning
{
public:
  Scanning(ros::NodeHandle nh_) {}
	int scan_it(const std::vector<std::string> &good_bins, const std::vector<std::string> &bad_bins) 
  {
    ROS_INFO("Called scan_it");
    return 0;
  }
};

#endif

#include <actionlib/server/simple_action_server.h>
#include <excel_move/ScanningAction.h>
#include "scanning_template.h"

class ScanningActionServer
{
public:
  ScanningActionServer(ros::NodeHandle& nh, Scanning& scanning) :
    nh_(nh), scanning_(scanning),
    act_srv_(nh, "", boost::bind(&ScanningActionServer::executeCB, this, _1), false)
  {
    act_srv_.start();
  }

protected:
  ros::NodeHandle nh_;
  Scanning& scanning_;

  actionlib::SimpleActionServer<excel_move::ScanningAction> act_srv_;
  excel_move::ScanningFeedback feedback_;
  excel_move::ScanningResult result_;
  bool scan_done_;

  void executeCB(const excel_move::ScanningGoalConstPtr& goal)
  {
    scan_done_ = false;
    ros::Timer scan_thread = nh_.createTimer(ros::Duration(0.001), 
        boost::bind(&ScanningActionServer::scanItCB, this, goal, _1), true);

    ros::Rate r(30);
    while (ros::ok()) {
      if(scan_done_) {
        ROS_INFO("Scanning action succeeded.");
        act_srv_.setSucceeded(result_);
        return;
      }
      if(act_srv_.isPreemptRequested()) {
        ROS_INFO("Preempting bin move");
        scan_thread.stop();
        return;
      }
      ros::spinOnce();
      r.sleep();
    }
  }

  void scanItCB(const excel_move::ScanningGoalConstPtr& goal, const ros::TimerEvent& te)
  {
    result_.result = scanning_.scan_it(goal->good_bins, goal->bad_bins);
    scan_done_ = true;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scanning_action_server");
  usleep(1000*1000);

  ros::NodeHandle nh;

  Scanning scanning(nh);

  ScanningActionServer scanning_as(nh, scanning);
  ros::spin();

  return 0;
}

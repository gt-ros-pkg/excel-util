
#include <actionlib/server/simple_action_server.h>
#include <excel_bins/MoveBinAction.h>
#include <excel_bins/move_bin.h>

class MoveBinActionServer
{
public:
  MoveBinActionServer(ros::NodeHandle& nh, MoveBin& move_bin) :
    nh_(nh), move_bin_(move_bin),
    act_srv_(nh, "move_bin_to_target", boost::bind(&MoveBinActionServer::executeCB, this, _1), false)
  {
    act_srv_.start();
  }

protected:
  ros::NodeHandle nh_;
  MoveBin& move_bin_;

  actionlib::SimpleActionServer<excel_bins::MoveBinAction> act_srv_;
  excel_bins::MoveBinFeedback feedback_;
  excel_bins::MoveBinResult result_;
  bool move_done_;

  void executeCB(const excel_bins::MoveBinGoalConstPtr& goal)
  {
    move_done_ = false;
    ros::Timer move_thread = nh_.createTimer(ros::Duration(0.001), 
        boost::bind(&MoveBinActionServer::moveBinCB, this, goal, _1), true);

    ros::Rate r(30);
    while (ros::ok()) {
      if(move_done_) {
        ROS_INFO("MoveBin action succeeded.");
        act_srv_.setSucceeded(result_);
        return;
      }
      if(act_srv_.isPreemptRequested()) {
        ROS_INFO("Preempting bin move");
        move_thread.stop();
        return;
      }
      ros::spinOnce();
      r.sleep();
    }
  }

  void moveBinCB(const excel_bins::MoveBinGoalConstPtr& goal, const ros::TimerEvent& te)
  {
    move_bin_.moveBinToTarget(goal->bin_id, goal->x_target, goal->y_target, goal->r_target);
    move_done_ = true;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_bin_action_server");
  usleep(1000*1000);

  ros::NodeHandle nh;

  MoveBin move_bin;

  MoveBinActionServer move_bin_as(nh, move_bin);
  ros::spin();

  return 0;
}

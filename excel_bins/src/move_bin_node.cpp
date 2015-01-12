#include <ros/ros.h>
#include <excel_bins/move_bin.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_bin");
  usleep(1000*1000);

  MoveBin movebin;

  int run_prg = 1;

  while(run_prg && ros::ok()){
    int nb;
    double x ,y, o;
    std::cout<< "Which bin number would you like to move ?" << std::endl;
    std::cout<< "bin#" << std::endl;
    std::cin >> nb;
    std::cout<< "Where should I put the bin ?" << std::endl;
    std::cout<< "x :" << std::endl;
    std::cin >> x;
    std::cout<< "y :" << std::endl;
    std::cin >> y;
    std::cout<< "angle :" << std::endl;
    std::cin >> o;

    if(!movebin.moveBinToTarget(nb, x, y, o, false)) {
      ROS_ERROR("Failed to move bin to target.");
    }

    std::cout<< "Keep moving bins ? (0/1)" << std::endl;
    std::cin >> run_prg;
  }
  ros::shutdown();
  return 0;
}

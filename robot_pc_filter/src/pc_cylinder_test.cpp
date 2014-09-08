#include <stdio.h>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <sensor_msgs/PointCloud2.h>

using namespace std;

typedef pcl::PointXYZRGB PRGB;
typedef pcl::PointCloud<PRGB> PCRGB;

void transPoints(const PCRGB &pc_in, const Eigen::Matrix4f &trans, PCRGB &pc_out)
{
    Eigen::MatrixXf m(4,pc_in.size());
    for(size_t i=0;i<pc_in.size();i++) {
        m(0,i) = pc_in[i].x; m(1,i) = pc_in[i].y; m(2,i) = pc_in[i].z; m(3,i) = 1;
    }
    m = trans * m;
    for(size_t i=0;i<pc_in.size();i++) {
        PRGB pt;
        pt.x = m(0,i); pt.y = m(1,i); pt.z = m(2,i); pt.rgb = pc_in[i].rgb;
        pc_out.push_back(pt);
    }
}

class PCCylinderRemoval
{
public:
  PCCylinderRemoval(ros::NodeHandle& nh)
    : nh_(nh)
  {
    pc_sub_ = nh.subscribe<PCRGB>("pc_in", 1, &PCCylinderRemoval::recvPCCallback, this);
    pc_pub_ = nh.advertise<PCRGB>("pc_out", 1);
  }
protected:
  void recvPCCallback(const PCRGB::ConstPtr& msg);

  PCRGB pc_in_;
  ros::NodeHandle nh_;
  ros::Subscriber pc_sub_;
  ros::Publisher pc_pub_;
};

void PCCylinderRemoval::recvPCCallback(const PCRGB::ConstPtr& msg)
{
  pc_pub_.publish(msg);
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "pc_cylinder_test");
    ros::NodeHandle nh_priv("~");
    PCCylinderRemoval pc_cyl_rm(nh_priv);
    ros::spin();
    return 0;
}

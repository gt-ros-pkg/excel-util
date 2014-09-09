#include <stdio.h>
#include <iostream>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include <sensor_msgs/PointCloud2.h>

using namespace std;
using namespace boost::accumulators;
using namespace boost::math;

typedef pcl::PointXYZRGB PRGB;
typedef pcl::PointCloud<PRGB> PCRGB;

typedef accumulator_set<float, stats<tag::mean,tag::variance,tag::count> > mean_var_acc;

#define MAX_Z 6.0
#define MIN_Z 1.3

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
    pc_sub_ = nh.subscribe<PCRGB>("pc_in", 5, &PCCylinderRemoval::recvPCCallback, this);
    pc_pub_ = nh.advertise<PCRGB>("pc_out", 1);
  }
protected:
  void recvPCCallback(const PCRGB::ConstPtr& pc_in);

  PCRGB::Ptr pc_proc_;
  ros::NodeHandle nh_;
  ros::Subscriber pc_sub_;
  ros::Publisher pc_pub_;

  std::vector<mean_var_acc> accs_;
};

void PCCylinderRemoval::recvPCCallback(const PCRGB::ConstPtr& pc_in)
{
  // Eigen::Matrix3f tf;
  // tf << -1.0,  0.0,  0.0,
  //        0.0, -1.0,  0.0,
  //        0.0,  0.0,  1.0;
  Eigen::Matrix4f tf;
  tf << -1.0,  0.0,  0.0,  0.0,
         0.0, -1.0,  0.0,  0.0,
         0.0,  0.0,  1.0,  0.0,
         0.0,  0.0,  0.0,  1.0;
  if(!pc_proc_) {
    pc_proc_.reset(new PCRGB(pc_in->width, pc_in->height));
    // pc_proc_.reset(new PCRGB());
    pc_proc_->header.frame_id = pc_in->header.frame_id;
    accs_.resize(pc_in->size());
  }
  pc_proc_->header.seq = pc_in->header.seq;
  pc_proc_->header.stamp = pc_in->header.stamp;
        
  // pc_proc_->getMatrixXfMap(4, 4, 0) = tf * pc_in->getMatrixXfMap(4, 4, 0);

  for(int i = 0; i < pc_in->size(); ++i) {
    float z = pc_in->at(i).z;
    if(z != z || z < MIN_Z || z > MAX_Z)
      continue;
    accs_[i](z);
  }

  int num_good_pts = 0;
  for(int i = 0; i < pc_in->size(); ++i) 
    if(boost::accumulators::extract::count(accs_[i]) >= 5)
      num_good_pts++;
  if(num_good_pts == 0)
    return;
  Eigen::MatrixXf A(num_good_pts,3);
  //Eigen::MatrixXf A(num_good_pts,2);
  //Eigen::MatrixXf A(num_good_pts,1);
  Eigen::VectorXf b(num_good_pts);
  int cur_ind = 0;
  for(int i = 0; i < pc_in->size(); ++i) {
    if(boost::accumulators::extract::count(accs_[i]) < 5) 
      continue;
    float mean_val = mean(accs_[i]);
    A(cur_ind,0) = mean_val*mean_val;
    A(cur_ind,1) = mean_val;
    A(cur_ind,2) = 1.0f;
    b(cur_ind) = sqrt(variance(accs_[i]));
    cur_ind++;
  }
  Eigen::VectorXf lin_fit = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
  //ROS_INFO("Lin fit: %f", lin_fit[0]);
  // ROS_INFO("Lin fit: %f, %f", lin_fit[0], lin_fit[1]);
  ROS_INFO("Lin fit: %f, %f, %f", lin_fit[0], lin_fit[1], lin_fit[2]);
  // pc_pub_.publish(pc_proc_);
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "pc_cylinder_test");
    ros::NodeHandle nh_priv("~");
    PCCylinderRemoval pc_cyl_rm(nh_priv);
    ros::spin();
    return 0;
}

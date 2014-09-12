#include <stdio.h>
#include <iostream>
#include <algorithm>

#include <urdf/model.h>

#include <tf/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZRGB PRGB;
typedef pcl::PointCloud<PRGB> PCRGB;

struct CylinderPC
{
  Eigen::Vector3f origin_pt;
  Eigen::Vector3f diff_vec; // the vector from the origin point to other axis point
  float length_sq;
  float radius_sq;
  CylinderPC(const tf::Vector3& origin_pt_tf, const tf::Vector3& diff_vec_tf,
             double length, double radius)
    : origin_pt(origin_pt_tf.x(), origin_pt_tf.y(), origin_pt_tf.z()), 
      diff_vec(diff_vec_tf.x(), diff_vec_tf.y(), diff_vec_tf.z()), 
      length_sq((float) (length*length)), radius_sq((float) (radius*radius))
  {}
  CylinderPC() {}
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct CylinderTf
{
  tf::Stamped<tf::Pose> tf_stamped_pose;
  double length;
  double radius;
  CylinderTf(const std::string& frame, tf::Pose& tf_pose, double _length, double _radius)
    : tf_stamped_pose(tf_pose, ros::Time(), frame), length(_length), radius(_radius)
  {}
};

class CylinderTfTransformer
{
public:
  CylinderTfTransformer(ros::NodeHandle& nh, std::vector<boost::shared_ptr<CylinderTf> > cyl_tfs)
    : tf_list_(nh), cyl_tfs_(cyl_tfs)
  {
    add_radius = 0.05;
    add_length = 0.05;
    nh.getParam("add_radius", add_radius);
    nh.getParam("add_length", add_length);
    for(int i = 0; i < cyl_tfs.size(); i++) {
      cyl_tfs[i]->radius += add_radius;
      cyl_tfs[i]->length += add_length;
    }
  }

  boost::shared_ptr<CylinderPC> getCylinder(boost::shared_ptr<CylinderTf> cyl_tf, const ros::Time& target_time, const std::string& target_frame)
  {
    tf::Stamped<tf::Pose> cur_pose;
    tf::Vector3 z_axis, origin, diff_vec, origin_pt;
    cyl_tf->tf_stamped_pose.stamp_ = target_time;
    try {
      tf_list_.waitForTransform(cyl_tf->tf_stamped_pose.frame_id_, target_frame, target_time, ros::Duration(1.0));
      tf_list_.transformPose(target_frame, cyl_tf->tf_stamped_pose, cur_pose);
    } 
    catch(tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
    }
    z_axis = cur_pose.getBasis().getColumn(2);
    origin = cur_pose.getOrigin();
    origin_pt = origin - 0.5*z_axis*cyl_tf->length;
    diff_vec = z_axis*cyl_tf->length;
    
    return boost::shared_ptr<CylinderPC>(new CylinderPC(origin_pt, diff_vec, cyl_tf->length, cyl_tf->radius));
  }

  void getCylinders(const ros::Time& target_time, const std::string& target_frame, 
                    std::vector<boost::shared_ptr<CylinderPC> >& cyls)
  {
    for(int i = 0; i < cyl_tfs_.size(); i++) 
      cyls.push_back(getCylinder(cyl_tfs_[i], target_time, target_frame));
  }

protected:
  tf::TransformListener tf_list_;
  std::vector<boost::shared_ptr<CylinderTf> > cyl_tfs_;

  double add_radius;
  double add_length;
};

class PCCylinderRemoval
{
public:
  PCCylinderRemoval(ros::NodeHandle& nh, boost::shared_ptr<CylinderTfTransformer> cyl_tf_trans)
    : nh_(nh), cyl_tf_trans_(cyl_tf_trans), num_proc(0), pd_mat(3, 640*480), tmp_mat(3, 640*480), dot_arr(640*480)
  {
    pc_sub_ = nh.subscribe<PCRGB>("pc_in", 5, &PCCylinderRemoval::recvPCCallback, this);
    pc_pub_ = nh.advertise<PCRGB>("pc_out", 1);
  }
protected:
  void recvPCCallback(const PCRGB::ConstPtr& pc_in);
  void getCylinderInds(boost::shared_ptr<CylinderPC> cyl, PCRGB::Ptr& pc_in, pcl::PointIndices::Ptr& inds);

  ros::NodeHandle nh_;
  boost::shared_ptr<CylinderTfTransformer> cyl_tf_trans_;
  ros::Subscriber pc_sub_;
  ros::Publisher pc_pub_;

  PCRGB::Ptr pc_proc_;

  Eigen::MatrixXf pd_mat;
  Eigen::MatrixXf tmp_mat;
  Eigen::ArrayXf dist_sq_arr;
  Eigen::ArrayXf dot_arr;

  int num_proc;
  ros::Duration time_passed;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// courtesy Greg James http://www.flipcode.com/archives/Fast_Point-In-Cylinder_Test.shtml
void PCCylinderRemoval::
     getCylinderInds(boost::shared_ptr<CylinderPC> cyl, PCRGB::Ptr& pc_in, pcl::PointIndices::Ptr& inds)
{
  float pdx, pdy, pdz, dot;
  for(int i=0;i<pc_in->size();i++) {
    PRGB pt = pc_in->at(i);
    pdx = pt.x-cyl->origin_pt(0);
    pdy = pt.y-cyl->origin_pt(1);
    pdz = pt.z-cyl->origin_pt(2);
    dot = cyl->diff_vec(0)*pdx + cyl->diff_vec(1)*pdy + cyl->diff_vec(2)*pdz;
    if(dot >= 0.0f && dot <= cyl->length_sq)
      if((pdx*pdx + pdy*pdy + pdz *pdz) - dot*dot/cyl->length_sq <= cyl->radius_sq)
        inds->indices.push_back(i);
  }
}

void PCCylinderRemoval::recvPCCallback(const PCRGB::ConstPtr& pc_in)
{
  PCRGB::Ptr pc_down_smpled(new PCRGB);
  PCRGB::Ptr possib_pc(new PCRGB);
  PCRGB::Ptr robot_pc(new PCRGB);

  pcl::VoxelGrid<PRGB> voxel_grid;
  pcl::ExtractIndices<PRGB> extract;

  voxel_grid.setInputCloud(pc_in);
  voxel_grid.setLeafSize(0.01, 0.01, 0.01);
  voxel_grid.filter(*pc_down_smpled);
        
  ros::Time start_time;
  start_time = ros::Time::now();


  std::vector<boost::shared_ptr<CylinderPC> > cyls;
  ros::Time pc_time;
  pc_time.fromNSec(pc_in->header.stamp*1000);

  tf::Pose elbow_tf_pose(tf::Quaternion(0.70710678, 0, 0, 0.70710678), tf::Vector3(0, 0, 0));
  double elbow_length = 0.55;
  double elbow_radius = 0.8;
  boost::shared_ptr<CylinderTf> elbow_cyl_tf(new CylinderTf("forearm_link", elbow_tf_pose, elbow_length, elbow_radius));

  boost::shared_ptr<CylinderPC> elbow_cyl_pc = cyl_tf_trans_->getCylinder(elbow_cyl_tf, pc_time, pc_in->header.frame_id);

  pcl::PointIndices::Ptr possib_pt_inds(new pcl::PointIndices);
  getCylinderInds(elbow_cyl_pc, pc_down_smpled, possib_pt_inds);

  extract.setInputCloud(pc_down_smpled);
  extract.setIndices(possib_pt_inds);
  extract.filter(*possib_pc);

  pcl::PointIndices::Ptr robot_pt_inds(new pcl::PointIndices);

  cyl_tf_trans_->getCylinders(pc_time, pc_in->header.frame_id, cyls);
  for(int i=0;i<cyls.size();i++) {
  //   ROS_INFO("origin %d: %f %f %f", i, cyls[i]->origin_pt(0), cyls[i]->origin_pt(1), cyls[i]->origin_pt(2));
    getCylinderInds(cyls[i], possib_pc, robot_pt_inds);
  }

  extract.setInputCloud(possib_pc);
  extract.setIndices(robot_pt_inds);
  extract.filter(*robot_pc);

  pc_pub_.publish(robot_pc);
  num_proc++;
  time_passed += ros::Time::now() - start_time;
  ROS_INFO_THROTTLE(1.0, "Avg time %f", time_passed.toSec()/num_proc);

  // CylinderPC cylind;
  // cylind.origin_pt = Eigen::Vector3f(0.21, 0.07, 1.74);
  // cylind.diff_vec = Eigen::Vector3f(0.0, 0.05,  0.0);
  // cylind.length_sq = (cylind.diff_vec).squaredNorm();
  // cylind.radius_sq = 100.3*100.3;

  // pcl::PointIndices::Ptr pt_inds(new pcl::PointIndices);
  // getCylinderInds(cylind, pc_in, pt_inds);
  // pcl::ExtractIndices<PRGB> extract;
  // extract.setInputCloud(pc_in);
  // extract.setIndices(pt_inds);
  // PCRGB new_pc;
  // new_pc.header.frame_id = pc_in->header.frame_id;
  // new_pc.header.stamp = pc_in->header.stamp;
  // extract.filter(new_pc);
  // pc_pub_.publish(new_pc);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "pc_cylinder_test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  std::vector<boost::shared_ptr<CylinderTf> > cyl_tfs;

  urdf::Model urdf_model;
  if(!urdf_model.initParam("/robot_description")) {
    ROS_ERROR("excel_ctrl_man requires a URDF in the robot_description parameter.");
    return -1;
  }
  std::vector<boost::shared_ptr<urdf::Link> > links;
  urdf_model.getLinks(links);
  for(int i = 0; i < links.size(); i++) {
    boost::shared_ptr<urdf::Link> link = links[i];
    for(int j = 0; j < link->collision_array.size(); j++) {
      boost::shared_ptr<urdf::Collision> collision = link->collision_array[j];
      if(collision->geometry && collision->geometry->type == urdf::Geometry::CYLINDER) {
        boost::shared_ptr<urdf::Cylinder> cyl = boost::static_pointer_cast<urdf::Cylinder>(collision->geometry);
        urdf::Pose urdf_pose = collision->origin;
        tf::Pose tf_pose(tf::Quaternion(urdf_pose.rotation.x, urdf_pose.rotation.y, 
                                        urdf_pose.rotation.z, urdf_pose.rotation.w), 
                         tf::Vector3(urdf_pose.position.x, urdf_pose.position.y, urdf_pose.position.z));
        boost::shared_ptr<CylinderTf> cyl_tf(new CylinderTf(link->name, tf_pose, cyl->length, cyl->radius));
        cyl_tfs.push_back(cyl_tf);
      }
    }
  }
  boost::shared_ptr<CylinderTfTransformer> cyl_tf_trans(new CylinderTfTransformer(nh, cyl_tfs));
  ros::Duration(1.0).sleep();

  PCCylinderRemoval pc_cyl_rm(nh_priv, cyl_tf_trans);
  ros::spin();
  return 0;
}

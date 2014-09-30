#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <shape_msgs/Mesh.h>
#include <excel_bins/Bins.h>
#include <excel_bins/Bin.h>
#include <sstream>
#include <iostream>     // std::cout
#include <algorithm>    // std::find
#include <vector>       // std::vector

#define M_PI 3.14159265358979323846  /* pi */
#define TABLE_HEIGHT 0.88

moveit_msgs::CollisionObject collision_object;
moveit_msgs::PlanningScene planning_sc;
shapes::Mesh* m;
ros::Publisher planning_scene_diff_publisher;

void bins_callback(const excel_bins::Bins::ConstPtr& msg)
{
	excel_bins::Bin current_bin;
	std::vector<std::string> bins_updated;
	std::vector<std::string> bins_in_scene;
	for(int i=0;i<planning_sc.world.collision_objects.size();i++){
		bins_in_scene.push_back(planning_sc.world.collision_objects[i].id);
	}				
	
	//ROS_INFO("%d bins to update", msg->bins.size());
	for(int i=0;i<msg->bins.size();i++){
		current_bin = msg->bins[i];
		bins_updated.push_back(current_bin.name);
		collision_object.id = current_bin.name;
		std::string bin_size = current_bin.size;
		if ((bin_size=="small")||(bin_size=="large")){
			std::string dir = "package://excel_bins/meshes/bin_"+bin_size+".stl";
			m = shapes::createMeshFromResource(dir);
		}
		else{
			ROS_ERROR("Wrong size specified");
			break;
		}
		shape_msgs::Mesh co_mesh;
		shapes::ShapeMsg co_mesh_msg;
		shapes::constructMsgFromShape(m,co_mesh_msg);
		co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);

		// Define bin's position //
		geometry_msgs::Pose mesh_pose;
		mesh_pose = current_bin.pose;
		mesh_pose.position.z = TABLE_HEIGHT;

		// By default we plan to ADD a bin
		collision_object.operation = collision_object.ADD;

		for(int i=0;i<planning_sc.world.collision_objects.size();i++){
			if (planning_sc.world.collision_objects[i].id == current_bin.name){
				geometry_msgs::Pose diff_pose = planning_sc.world.collision_objects[i].mesh_poses[0];
				diff_pose.position.x -= current_bin.pose.position.x;
				diff_pose.position.y -= current_bin.pose.position.y;
				diff_pose.position.z -= current_bin.pose.position.z;
				
				tf::Quaternion tf_quat;
				tf::quaternionMsgToTF(current_bin.pose.orientation, tf_quat);
				double roll1, pitch1, yaw1, roll2, pitch2, yaw2 ;
				tf::Matrix3x3(tf_quat).getRPY(roll1, pitch1, yaw1);
				tf::quaternionMsgToTF(planning_sc.world.collision_objects[i].mesh_poses[0].orientation, tf_quat);
				tf::Matrix3x3(tf_quat).getRPY(roll2, pitch2, yaw2);
				double diff_coeff = (diff_pose.position.x)*diff_pose.position.x + (diff_pose.position.y)*diff_pose.position.y;
								
				if ((diff_coeff<0.001) && (abs(yaw2-yaw1)<0.001)){
					collision_object.operation = collision_object.APPEND;
				}else{
					//std::cout << current_bin.name <<" to pose :\n" << "x = "<<mesh_pose.position.x<<" ; y = "<<mesh_pose.position.y<<" ; z = "<<mesh_pose.position.z<< std::endl;
				}
			}
		}
		// APPEND is just used as a boolean to decide if a change has to be done
		if(collision_object.operation != collision_object.APPEND){
			// Attach object operation //
			collision_object.meshes.clear();
			collision_object.mesh_poses.clear();
			collision_object.meshes.push_back(co_mesh);
			collision_object.mesh_poses.push_back(mesh_pose);
			
			// Put the bin in the environment //
			planning_sc.world.collision_objects.clear();
			planning_sc.world.collision_objects.push_back(collision_object);
			planning_sc.is_diff = true;
			planning_scene_diff_publisher.publish(planning_sc);
		}	
	}
	
	// Remove old bins
  /*
	for( std::vector<std::string>::const_iterator i = bins_in_scene.begin(); i != bins_in_scene.end(); ++i){	    
	    std::vector<std::string>::iterator it;
	    it = std::find(bins_updated.begin(), bins_updated.end(), *i);
	    if (it == bins_updated.end()){
	    	collision_object.id = *i;
	    	//std::cout<< "Removing "<<*i <<std::endl;
	    	// detach object operation //
	    	collision_object.operation = collision_object.REMOVE;
			
	    	// Put the bin in the environment //
	    	planning_sc.world.collision_objects.clear();
	    	planning_sc.world.collision_objects.push_back(collision_object);
	    	planning_sc.is_diff = true;
	    	planning_scene_diff_publisher.publish(planning_sc);
	    }
	}
  */
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "update_bins");
	ros::WallDuration sleep_t(1.0);
	usleep(1000*1000);
	ros::NodeHandle nh_;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	collision_object.header.frame_id = "table_link";

	// Load the planning_scene and make sure we can publish the scene //
	planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	while(planning_scene_diff_publisher.getNumSubscribers() < 1)
	{
		sleep_t.sleep();
	}

	// Loading planning_scene_monitor //
	boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(ros::Duration(2.0)));
	planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf));
	planning_scene_monitor->startSceneMonitor();
	planning_scene_monitor->startStateMonitor();
	planning_scene_monitor->startWorldGeometryMonitor();

	
	// Update the planning scene //
	planning_scene_monitor->requestPlanningSceneState();
	planning_scene::PlanningScenePtr full_planning_scene;
	full_planning_scene = planning_scene_monitor->getPlanningScene();
	full_planning_scene->getPlanningSceneMsg(planning_sc);

	ros::Subscriber sub = nh_.subscribe("bins_update", 1, bins_callback);
	
	while(ros::ok()){
		full_planning_scene = planning_scene_monitor->getPlanningScene();
		full_planning_scene->getPlanningSceneMsg(planning_sc);
		sleep_t.sleep();
	}

	ros::shutdown();
}

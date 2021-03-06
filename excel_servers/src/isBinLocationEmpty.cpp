#include "ros/ros.h"
#include "excel_servers/BinLocationEmpty.h"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <shape_msgs/Mesh.h>

class BinCollisionDetection
{
	public:
		// Constructor.
		BinCollisionDetection();

		bool checkForBinCollision(excel_servers::BinLocationEmpty::Request  &req, excel_servers::BinLocationEmpty::Response &res);

		planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
		//boost::shared_ptr<tf::TransformListener> tf;
		planning_scene::PlanningScenePtr full_planning_scene;
		ros::NodeHandle nh_;
		ros::ServiceServer service;
};

BinCollisionDetection::BinCollisionDetection() {
	boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(ros::Duration(2.0)));
	planning_scene_monitor::PlanningSceneMonitorPtr plg_scn_mon(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf));

	planning_scene_monitor = plg_scn_mon;
	planning_scene_monitor->startSceneMonitor();
	planning_scene_monitor->startStateMonitor();
	planning_scene_monitor->startWorldGeometryMonitor();

	service = nh_.advertiseService("is_bin_location_empty",&BinCollisionDetection::checkForBinCollision,this);

	ROS_INFO("ADVERTISING SERVICE is_bin_location_empty");
}

bool BinCollisionDetection::checkForBinCollision(excel_servers::BinLocationEmpty::Request &req, excel_servers::BinLocationEmpty::Response &res)
{
	ROS_INFO("Collision check called");

	moveit_msgs::PlanningScene planning_scene;
	planning_scene.is_diff = false;

	planning_scene_monitor->requestPlanningSceneState();
	full_planning_scene = planning_scene_monitor->getPlanningScene();
	full_planning_scene->getPlanningSceneMsg(planning_scene);

	moveit_msgs::AttachedCollisionObject attached_object;
	attached_object.object.operation = attached_object.object.ADD;
	attached_object.link_name = "table_link";

	shapes::Mesh* m;
	std::string path;

	for(int i = 0;i<req.bin_to_place.size();i++){
		attached_object.object = req.bin_to_place[i];

		if(req.bin_to_place[i].id =="small_bin")
			path = "package://excel_bins/meshes/bin_small.stl"; 
		else path = "package://excel_bins/meshes/bin_large.stl";

		m = shapes::createMeshFromResource(path);
		shape_msgs::Mesh co_mesh;
		shapes::ShapeMsg co_mesh_msg;
		shapes::constructMsgFromShape(m, co_mesh_msg);
		co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);
		attached_object.object.meshes.clear();
		attached_object.object.meshes.push_back(co_mesh);

		planning_scene.robot_state.attached_collision_objects.clear();
		planning_scene.robot_state.attached_collision_objects.push_back(attached_object);

		collision_detection::CollisionResult collision_result;
		collision_detection::CollisionRequest collision_request;
		full_planning_scene->setPlanningSceneMsg(planning_scene);
		full_planning_scene->checkCollision(collision_request, collision_result);

		res.empty.push_back(!collision_result.collision);
	}
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "isBinLocationEmpty_node");
	usleep(1000*1000);

	BinCollisionDetection bin_check;

	ROS_INFO("READY TO CHECK BIN COLLISION");
	ros::spin();

	return 0;
}

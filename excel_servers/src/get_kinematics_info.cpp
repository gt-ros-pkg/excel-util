#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <std_msgs/Float32MultiArray.h>
//#include <sensor_msgs/JointState.h>
#include <excel_servers/KinematicsInfo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

class KinematicsInfoService
{
	public:
		// Constructor.
		KinematicsInfoService();

		// Service callback
		bool get_kinematics_info(excel_servers::KinematicsInfo::Request &req, excel_servers::KinematicsInfo::Response &res);

		void getPlanningScene(moveit_msgs::PlanningScene& planning_scene, planning_scene::PlanningScenePtr& full_planning_scene);

		void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);

		// Global variables
		ros::NodeHandle nh_;
		ros::ServiceServer service;
		ros::Subscriber sub;
		robot_model_loader::RobotModelLoader robot_model_loader;
		robot_model::RobotModelPtr kinematic_model;
		planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
		sensor_msgs::JointState current_joint_state;
};

KinematicsInfoService::KinematicsInfoService() : robot_model_loader("robot_description"){
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(ros::Duration(2.0)));
	planning_scene_monitor::PlanningSceneMonitorPtr plg_scn_mon(
			new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf));
	planning_scene_monitor = plg_scn_mon;
	planning_scene_monitor->startSceneMonitor();
	planning_scene_monitor->startStateMonitor();
	planning_scene_monitor->startWorldGeometryMonitor();

	kinematic_model = robot_model_loader.getModel();

	service = nh_.advertiseService("get_kinematics_info",&KinematicsInfoService::get_kinematics_info,this);
	sub = nh_.subscribe("joint_states", 1, &KinematicsInfoService::joint_states_callback, this);	
}

void KinematicsInfoService::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg){
	this->current_joint_state = *msg;
}

bool KinematicsInfoService::get_kinematics_info(excel_servers::KinematicsInfo::Request &req, excel_servers::KinematicsInfo::Response &res){
	// update planning scene
	moveit_msgs::PlanningScene planning_scene;
	planning_scene::PlanningScenePtr full_planning_scene;
	getPlanningScene(planning_scene, full_planning_scene);

	const moveit::core::RobotState kinematic_state = full_planning_scene->getCurrentState();
	const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("excel");

	// Get jacobian
	Eigen::MatrixXd jacobian_mat = kinematic_state.getJacobian(joint_model_group);
	jacobian_mat.transposeInPlace();
	Eigen::VectorXd B(Eigen::Map<Eigen::VectorXd>(jacobian_mat.data(), jacobian_mat.cols()*jacobian_mat.rows()));
	for (int i=0;i<B.size();i++){
		res.jacobian.data.push_back(B(i));
	}

	// Get positions
	const double * positions = kinematic_state.getVariablePositions();
	for (int i=0;i<7;i++){
		res.positions.data.push_back(positions[i]);
	}
	
	// Get velocities
	for (int i=0;i<7;i++){
		res.velocities.data.push_back(this->current_joint_state.velocity[i]);
	}
	
	return true;
}

void KinematicsInfoService::getPlanningScene(moveit_msgs::PlanningScene& planning_scene, 
                               planning_scene::PlanningScenePtr& full_planning_scene)
{
  planning_scene_monitor->requestPlanningSceneState();
  full_planning_scene = planning_scene_monitor->getPlanningScene();
  full_planning_scene->getPlanningSceneMsg(planning_scene);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kinematics_info_server_node");
	usleep(1000*1000);

	KinematicsInfoService kinematics_info;

	ROS_INFO("SERVICE READY");
	ros::spin();

	return 0;
}

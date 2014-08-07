#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf/transform_broadcaster.h>
#include <moveit/robot_state/attached_body.h>
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <shape_msgs/Mesh.h>

# define M_PI 3.14159265358979323846  /* pi */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_bin");
    ros::NodeHandle nh_, nh_param_("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    double x_target, y_target, angle_target;

    // Load parameters //
    nh_param_.getParam("x_goal",x_target);
    nh_param_.getParam("y_goal",y_target);
    nh_param_.getParam("angle",angle_target);

    // Loading planning_scene_monitor //
    boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(ros::Duration(2.0)));
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf));
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startStateMonitor();
    planning_scene_monitor->startWorldGeometryMonitor();

    // Making sure we can publish attached/unattached objects //
    ros::Publisher attached_object_publisher = nh_.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 1);
    while(attached_object_publisher.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }
    ros::Publisher planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while(planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }

    // Loading the group //
    move_group_interface::MoveGroup group("excel");
    group.setPlanningTime(8.0);

    // Update the planning scene //
    planning_scene_monitor->requestPlanningSceneState();
    planning_scene::PlanningScenePtr full_planning_scene = planning_scene_monitor->getPlanningScene();
    moveit_msgs::PlanningScene planning_scene;
    full_planning_scene->getPlanningSceneMsg(planning_scene);

    // Giving the number of objects in world and attached to the robot //
    std::vector<const moveit::core::AttachedBody*> attached_bodies ;
    full_planning_scene->getCurrentState().getAttachedBodies(attached_bodies);
    std::vector<moveit_msgs::CollisionObject> collision_objects = planning_scene.world.collision_objects;
    ROS_INFO_STREAM("Nb AttachedBodies : " << attached_bodies.size());
    ROS_INFO_STREAM("Nb of collision objects" << collision_objects.size() );


    ROS_INFO("Moving close to the bin");
    geometry_msgs::Pose object_pick_pose = collision_objects[0].mesh_poses[0];
    tf::Quaternion co_quat(object_pick_pose.orientation.x, object_pick_pose.orientation.y, object_pick_pose.orientation.z, object_pick_pose.orientation.w);
    tf::Matrix3x3 m(co_quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    tf::Quaternion quat = tf::createQuaternionFromRPY(M_PI/2-yaw,M_PI/2,M_PI);
    object_pick_pose.position.z += 0.4;
    object_pick_pose.orientation.x = quat.x();
    object_pick_pose.orientation.y = quat.y();
    object_pick_pose.orientation.z = quat.z();
    object_pick_pose.orientation.w = quat.w();
    group.setPoseTarget(object_pick_pose);
    group.move();
    //sleep(1.0);

    ROS_INFO("Descent");
    object_pick_pose.position.z = 1.05;
    group.setPoseTarget(object_pick_pose);
    group.move();

    ROS_INFO("Attaching the bin");
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "wrist_3_link";
    attached_object.object = collision_objects[0];
    attached_object.object.operation = attached_object.object.ADD;
    attached_object_publisher.publish(attached_object);
    //sleep(1.0);

    ROS_INFO("Lift");
    object_pick_pose.position.z += 0.4;
    group.setPoseTarget(object_pick_pose);
    group.move();
    //sleep(1.0);

    ROS_INFO("Moving to target");
    geometry_msgs::Pose target_pose;
    tf::Quaternion quat_goal = tf::createQuaternionFromRPY(M_PI/2-angle_target*M_PI/180.0,M_PI/2,M_PI);
    target_pose.orientation.x = quat_goal.x();
    target_pose.orientation.y = quat_goal.y();
    target_pose.orientation.z = quat_goal.z();
    target_pose.orientation.w = quat_goal.w();
    target_pose.position.x = x_target;
    target_pose.position.y = y_target;
    target_pose.position.z = 1.45;
    group.setPoseTarget(target_pose);
    group.move();
    //sleep(1.0);

    ROS_INFO("Descent");
    target_pose.position.z = 1.05;
    group.setPoseTarget(target_pose);
    group.move();
    //sleep(1.0);

    // Update the scene //
    full_planning_scene = planning_scene_monitor->getPlanningScene();
    full_planning_scene->getPlanningSceneMsg(planning_scene);

    ROS_INFO("Detaching the bin and putting in back to the environment");
    planning_scene.robot_state.attached_collision_objects.clear();
    attached_object.object.mesh_poses.clear();
    geometry_msgs::Pose new_bin_pose;
    tf::Quaternion quat_bin = tf::createQuaternionFromRPY(0,0,angle_target*M_PI/180.0);
    new_bin_pose.orientation.x = quat_bin.x();
    new_bin_pose.orientation.y = quat_bin.y();
    new_bin_pose.orientation.z = quat_bin.z();
    new_bin_pose.orientation.w = quat_bin.w();
    new_bin_pose.position = target_pose.position;
    new_bin_pose.position.z = 0.83;
    attached_object.object.mesh_poses.push_back(new_bin_pose);
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene_diff_publisher.publish(planning_scene);
    //sleep(1.0);

    ROS_INFO("Lift");
    target_pose.position.z += 0.4;
    group.setPoseTarget(target_pose);
    group.move();
    //sleep(1.0);

    ROS_INFO("Go to Initial position");
    group.setNamedTarget("Initial");
    group.move();
    //sleep(1.0);

    ros::shutdown();
}

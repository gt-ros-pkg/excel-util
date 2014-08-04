#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <shape_msgs/Mesh.h>
#include <iostream>
#include <sstream>

# define M_PI 3.14159265358979323846  /* pi */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_bin");
    ros::NodeHandle nh_, nh_param_("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    double x_goal, y_goal, z_goal, angle;
    std::string bin_size;

    // Load the planning_scene and make sure we can publish the scene //
    moveit_msgs::PlanningScene planning_scene;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::WallDuration sleep_t(0.5);
    ros::Publisher planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while(planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }

    std::vector<std::string> objects_names = planning_scene_interface.getKnownObjectNames();
    int nb_bins = objects_names.size();
    std::ostringstream os;
    os << nb_bins;
    std::string object_id = "bin#"+ os.str();

    // Define the attached object message //
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "table_link";
    collision_object.id = object_id;

    // Define the mesh //
    shapes::Mesh* m;
    nh_param_.getParam("bin_size",bin_size);
    if ((bin_size=="small")||(bin_size=="large")){
        std::string dir = "package://excel_bins/meshes/bin_"+bin_size+".stl";
        m = shapes::createMeshFromResource(dir);
    }
    else{
        m = shapes::createMeshFromResource("package://excel_bins/meshes/bin_small.stl");
        bin_size = "small";
    }
    shape_msgs::Mesh co_mesh;
    shapes::ShapeMsg co_mesh_msg;
    shapes::constructMsgFromShape(m,co_mesh_msg);
    co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);

    // Define bin's position //
    geometry_msgs::Pose mesh_pose;
    if(nh_param_.getParam("x_goal",x_goal)){
        mesh_pose.position.x = x_goal;
    }
    if(nh_param_.getParam("y_goal",y_goal)){
        mesh_pose.position.y = y_goal;
    }
    if(nh_param_.getParam("z_goal",z_goal)){
        mesh_pose.position.z = z_goal;
    }
    if(nh_param_.getParam("angle",angle)){
        tf::Quaternion quat = tf::createQuaternionFromRPY(0,0,angle*M_PI/180);
        mesh_pose.orientation.x = quat.x();
        mesh_pose.orientation.y = quat.y();
        mesh_pose.orientation.z = quat.z();
        mesh_pose.orientation.w = quat.w();
    }

    // Give info back
    std::ostringstream info;
    info << "Adding a "<< bin_size <<" bin at pose :\n" << "x = "<<mesh_pose.position.x<<" ; y = "<<mesh_pose.position.y<<" ; z = "<<mesh_pose.position.z;
    ROS_INFO_STREAM(info.str());

    // Attach object operation //
    collision_object.meshes.push_back(co_mesh);
    collision_object.mesh_poses.push_back(mesh_pose);
    collision_object.operation = collision_object.ADD;

    // Put the bin in the environment //
    planning_scene.world.collision_objects.push_back(collision_object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);

    ros::shutdown();
}

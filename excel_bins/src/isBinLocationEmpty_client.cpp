#include "ros/ros.h"
#include "excel_bins/BinLocationEmpty.h"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <shape_msgs/Mesh.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "isBinLocationEmpty_client_node");
    usleep(1000*1000);

    double x_goal = 0.5;
    double y_goal = 0.5;
    double z_goal = 0.88;
    double angle = 0.0;

    ros::NodeHandle nh_;
    ros::ServiceClient client = nh_.serviceClient<excel_bins::BinLocationEmpty>("is_bin_location_empty");
    ros::service::waitForService("is_bin_location_empty");

    excel_bins::BinLocationEmpty srv;

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "table_link";
    collision_object.id = "bin_to_place";

    // Define the mesh //
    shapes::Mesh* m;
    std::string bin_size = "large";
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
    mesh_pose.position.x = x_goal;
    mesh_pose.position.y = y_goal;
    mesh_pose.position.z = z_goal;

    tf::Quaternion quat = tf::createQuaternionFromRPY(0,0,angle*M_PI/180);
    mesh_pose.orientation.x = quat.x();
    mesh_pose.orientation.y = quat.y();
    mesh_pose.orientation.z = quat.z();
    mesh_pose.orientation.w = quat.w();

    collision_object.meshes.push_back(co_mesh);
    collision_object.mesh_poses.push_back(mesh_pose);
    collision_object.operation = collision_object.ADD;

    srv.request.bin_to_place = collision_object;

    if (client.call(srv))
    {
        ROS_INFO("Bin location is%s empty", srv.response.empty ? "":" not");
    }
    else
    {
        ROS_ERROR("Failed to call service is_bin_location_empty");
        return 1;
    }

    ros::shutdown();

    return 0;
}

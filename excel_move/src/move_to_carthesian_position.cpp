#include <moveit/move_group_interface/move_group.h>
#include <tf/transform_broadcaster.h>

# define M_PI 3.14159265358979323846  /* pi */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_to_carthesian_position");
    ros::NodeHandle nh_("~");
    usleep(1000*1000);
    // start a ROS spinning thread
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate rate(10.0);

    std::string position = "Initial";
    double x_goal, y_goal, z_goal;


    // this connects to a running instance of the move_group node
    move_group_interface::MoveGroup group("excel");
    ROS_INFO_STREAM(group.getCurrentPose());
    geometry_msgs::PoseStamped current_pose = group.getCurrentPose();
    geometry_msgs::Pose target_pose;

    std::vector<double> rpy = group.getCurrentRPY();
    ROS_INFO_STREAM(rpy[0]);
    ROS_INFO_STREAM(rpy[1]);
    ROS_INFO_STREAM(rpy[2]);

    group.setPlanningTime(15.0);
//    group.setPlannerId("RTTConnect");

    //moveit_msgs::Constraints constraints2 = group.getPathConstraints();
    //std::vector<std::string> csts = group.getKnownConstraints();
    //ROS_INFO_STREAM("constraint name " << csts.size()<<".");

    tf::Quaternion quat = tf::createQuaternionFromRPY(0,M_PI/2,M_PI);

    target_pose = current_pose.pose;
    if(nh_.getParam("x_goal",x_goal)){
        ROS_INFO("x_goal found");
        target_pose.position.x = x_goal;
    }
    if(nh_.getParam("y_goal",y_goal)){
        ROS_INFO("y_goal found");
        target_pose.position.y = y_goal;
    }
    if(nh_.getParam("z_goal",z_goal)){
        ROS_INFO("z_goal found");
        target_pose.position.z = z_goal;
    }
    target_pose.orientation.x = quat.x();
    target_pose.orientation.y = quat.y();
    target_pose.orientation.z = quat.z();
    target_pose.orientation.w = quat.w();

    tf::TransformBroadcaster br;
    tf::Transform transform;

    moveit_msgs::OrientationConstraint ocm = moveit_msgs::OrientationConstraint();

    ocm.header.frame_id = "table_link";
    ocm.orientation.x = quat.x();
    ocm.orientation.y = quat.y();
    ocm.orientation.z = quat.z();
    ocm.orientation.w = quat.w();
    ocm.link_name = "ee_link";
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = M_PI;
    ocm.weight = 1.0;

    moveit_msgs::Constraints constraints;
    constraints.orientation_constraints.push_back(ocm);
    constraints.name = "roll_pitch_control";

    //group.setPathConstraints(constraints);

    group.setStartStateToCurrentState();
    group.setPoseTarget(target_pose);
    ROS_INFO("Planning and moving...");
    group.move();

    ROS_INFO("Displaying goal frame...");
    while (nh_.ok()){
        transform.setOrigin(tf::Vector3(target_pose.position.x,target_pose.position.y,target_pose.position.z ));
        transform.setRotation(quat);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "table_link","frame_goal"));
        rate.sleep();
    }

    ros::waitForShutdown();
}

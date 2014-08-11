#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/JointConstraint.h>
#include <tf/transform_broadcaster.h>

# define M_PI 3.14159265358979323846  /* pi */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_to_carthesian_position");
    ros::NodeHandle nh_param_("~"), nh_;
    usleep(1000*1000);
    // start a ROS spinning thread
    ros::AsyncSpinner spinner(1);
    spinner.start();

    double x_goal, y_goal, z_goal;
    bool path_constraint;
    nh_param_.getParam("constraint",path_constraint);

    // this connects to a running instance of the move_group node
    move_group_interface::MoveGroup group("excel");
    ROS_INFO_STREAM(group.getCurrentPose());
    geometry_msgs::PoseStamped current_pose = group.getCurrentPose();

    std::vector<double> rpy = group.getCurrentRPY();
    ROS_INFO_STREAM(rpy[0]);
    ROS_INFO_STREAM(rpy[1]);
    ROS_INFO_STREAM(rpy[2]);

    group.setPlanningTime(10.0);


    // IK Service //
    ros::ServiceClient service_client = nh_.serviceClient<moveit_msgs::GetPositionIK> ("compute_ik");
    while(!service_client.exists())
    {
        ROS_INFO("Waiting for service");
        sleep(1.0);
    }
    moveit_msgs::GetPositionIK::Request service_request;
    moveit_msgs::GetPositionIK::Response service_response;
    service_request.ik_request.group_name = "excel";
    service_request.ik_request.pose_stamped.header.frame_id = "table_link";
    service_request.ik_request.avoid_collisions = true;
    service_request.ik_request.attempts = 50;

    // Joint Constraints //
    std::vector<moveit_msgs::JointConstraint> joint_constraints;
    joint_constraints.resize(4);
    joint_constraints[0].joint_name="elbow_joint";
    joint_constraints[0].position=-M_PI/2;
    joint_constraints[0].tolerance_above=M_PI/2;
    joint_constraints[0].tolerance_below=M_PI/2;
    joint_constraints[0].weight=10.0;

    joint_constraints[1].joint_name="shoulder_lift_joint";
    joint_constraints[1].position=-M_PI/2;
    joint_constraints[1].tolerance_above=M_PI/2;
    joint_constraints[1].tolerance_below=M_PI/2;
    joint_constraints[1].weight=10.0;

    joint_constraints[2].joint_name="wrist_1_joint";
    joint_constraints[2].position=-M_PI/2;
    joint_constraints[2].tolerance_above=M_PI/2;
    joint_constraints[2].tolerance_below=M_PI/2;
    joint_constraints[2].weight=10.0;

    joint_constraints[3].joint_name="wrist_2_joint";
    joint_constraints[3].position=M_PI/2;
    joint_constraints[3].tolerance_above=M_PI/2;
    joint_constraints[3].tolerance_below=M_PI/2;
    joint_constraints[3].weight=10.0;

    service_request.ik_request.constraints.joint_constraints = joint_constraints;


    tf::Quaternion quat = tf::createQuaternionFromRPY(0,M_PI/2,M_PI);

    service_request.ik_request.pose_stamped.pose = current_pose.pose;
    if(nh_param_.getParam("x_goal",x_goal)){
        service_request.ik_request.pose_stamped.pose.position.x = x_goal;
    }
    if(nh_param_.getParam("y_goal",y_goal)){
        service_request.ik_request.pose_stamped.pose.position.y = y_goal;
    }
    if(nh_param_.getParam("z_goal",z_goal)){
        service_request.ik_request.pose_stamped.pose.position.z = z_goal;
    }
    service_request.ik_request.pose_stamped.pose.orientation.x = quat.x();
    service_request.ik_request.pose_stamped.pose.orientation.y = quat.y();
    service_request.ik_request.pose_stamped.pose.orientation.z = quat.z();
    service_request.ik_request.pose_stamped.pose.orientation.w = quat.w();

    if(path_constraint){
        ROS_INFO("PATH CONSTRAINT ON");
        moveit_msgs::OrientationConstraint ocm = moveit_msgs::OrientationConstraint();
        ocm.header.frame_id = "table_link";
        ocm.orientation.x = quat.x();
        ocm.orientation.y = quat.y();
        ocm.orientation.z = quat.z();
        ocm.orientation.w = quat.w();
        ocm.link_name = "ee_link";
        ocm.absolute_x_axis_tolerance = 0.5;
        ocm.absolute_y_axis_tolerance = 0.5;
        ocm.absolute_z_axis_tolerance = M_PI;
        ocm.weight = 1.0;

        moveit_msgs::Constraints constraints;
        constraints.orientation_constraints.push_back(ocm);
        constraints.name = "roll_pitch_control";

        group.setPathConstraints(constraints);
    }

    ROS_INFO("Planning and moving...");
    group.setStartStateToCurrentState();
    service_client.call(service_request, service_response);
    group.setJointValueTarget(service_response.solution.joint_state);
    group.move();
    /*
    tf::TransformBroadcaster br;
    tf::Transform transform;

    ROS_INFO("Displaying goal frame...");
    while (nh_param_.ok()){
        transform.setOrigin(tf::Vector3(target_pose.position.x,target_pose.position.y,target_pose.position.z ));
        transform.setRotation(quat);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "table_link","frame_goal"));
        rate.sleep();
    }
*/
    ros::shutdown();
}

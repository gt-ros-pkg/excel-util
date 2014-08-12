#include <tf/transform_broadcaster.h>
#include <ros/ros.h>

# define M_PI 3.14159265358979323846  /* pi */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_transform");
    ros::NodeHandle nh_;
    usleep(1000*1000);
    static tf::TransformBroadcaster br;
    tf::Transform transform, transform2;
    tf::Quaternion q;
    transform.setOrigin( tf::Vector3(0.0, 1.5, 4.5) );
    transform2.setOrigin( tf::Vector3(1.3, 3.5, 4.2) );
    q.setRPY(M_PI, 0, -M_PI/2.0);
    transform.setRotation(q);
    transform2.setRotation(q);
    while(ros::ok()){
        br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "table_link", "4.5_camera") );
        br.sendTransform( tf::StampedTransform(transform2, ros::Time::now(), "table_link", "3.5_camera") );
    }

    ros::shutdown();
}

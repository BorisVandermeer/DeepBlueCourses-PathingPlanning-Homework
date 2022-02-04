#include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/PoseArray.h>

// #include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

ros::Publisher odom_puber;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"random_odom_generator");
    ros::NodeHandle nh("/waypoint_generator");
    odom_puber = nh.advertise<nav_msgs::Odometry>("odom", 10);

    nav_msgs::Odometry Data;




}

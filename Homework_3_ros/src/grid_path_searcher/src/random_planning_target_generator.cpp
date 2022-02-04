#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>

ros::Publisher target_puber;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"random_target_generator");
    ros::NodeHandle nh("/waypoint_generator");
    target_puber = nh.advertise<nav_msgs::Path>("waypoints", 50);

    
    ros::Time current_time,last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    nav_msgs::Path target;

    target.header.stamp = current_time;
    target.header.frame_id = "odom";

    double x,y,z;
    x = 30;y = 30; z = 30;

    geometry_msgs::PoseStamped tmp_posestamped;
    tmp_posestamped.pose.position.x = x;
    tmp_posestamped.pose.position.y = y;
    tmp_posestamped.pose.position.z = z;

    tmp_posestamped.header.stamp = current_time;
    tmp_posestamped.header.frame_id="odom";

    target.poses.push_back(tmp_posestamped);


    while(ros::ok())
    {
        ROS_INFO("Target is published.");
        target_puber.publish(target);
        ros::spin();
    }

    return 0;
}
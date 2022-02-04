#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>

ros::Publisher target_puber;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"random_goal_generator");
    ros::NodeHandle nh;
    target_puber = nh.advertise<geometry_msgs::PoseStamped>("goal", 50);

    ros::Time current_time;
    current_time = ros::Time::now();

    double x,y,z;
    x = 30;y = 30; z = 30;

    geometry_msgs::PoseStamped Goal;
    Goal.pose.position.x = x;
    Goal.pose.position.y = y;
    Goal.pose.position.z = z;
    Goal.header.stamp = current_time;
    Goal.header.frame_id="odom";


    while(ros::ok())
    {
        ROS_INFO("Target is published.");
        target_puber.publish(Goal);
        ros::spin();
    }

    return 0;
}
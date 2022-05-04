#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
using namespace std;

ros::Publisher vel_publisher;
ros::Subscriber pose_subscriber;
nav_msgs::Odometry tb3_odom1;
void tf3_1_odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    ROS_INFO("%s", msg->header.frame_id.c_str());
  // ROS_INFO("%f", msg->twist.twist.linear.x);
    tb3_odom1.pose.pose.position.x=msg->pose.pose.position.x;
    //ROS_INFO("%f",tb3_odom1.pose.pose.position.x);
}

int main(int argc, char **argv) {
  // Create a node to run the code
    ros::init(argc, argv, "multi_mr_fixed_dynamic");
    ros::NodeHandle nh;

    // create a subscriber to the "/odom" topic so we can get the odometry message
    pose_subscriber = nh.subscribe("/tb3_1/odom", 1000, tf3_1_odomCallback);

    // Run program until manually stopped
    ros::spin();

    return 0;
}

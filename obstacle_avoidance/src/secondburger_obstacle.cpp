#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
using namespace std;

ros::Publisher velpub;
nav_msgs::Odometry tb3_odom1;


int main(int argc, char **argv) {
  // Create a node to run the code
    ros::init(argc, argv, "secondburger_obstacle");
    ros::NodeHandle n1;// first obstacle

    // create a subscriber to the "/odom" topic so we can get the odometry message
    ros::Publisher velpub = n1.advertise<geometry_msgs::Twist>("/tb3_2/cmd_vel", 1);
    ros::Duration(2).sleep();
    geometry_msgs::Twist velmsg;
    double vx=0.08;
    velmsg.linear.x=-vx;
    velpub.publish(velmsg);
    int counter=0;
    while (counter<50){
    ROS_INFO("Obstacle velosity is: %f , counter is : %d",vx, counter);
    ros::Duration(1).sleep();

    counter++;
    }
    velmsg.linear.x=0;
    velpub.publish(velmsg);
    return 0;
}

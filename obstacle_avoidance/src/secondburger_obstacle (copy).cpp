#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <math.h>
using namespace std;

ros::Publisher velpub;

ros::Subscriber odomObs1_sub;
nav_msgs::Odometry tb3_odom1;
nav_msgs::Odometry odomObs1_msg;
geometry_msgs::Twist velmsg;
ros::Publisher log_publisher;

geometry_msgs::Pose2D obs1pose2d;
geometry_msgs::Twist logdata;
float obs1v=0.0; // obstacle velocity is 0 meter/sec
double obs1roll, obs1pitch, obs1yaw;
float Ts=1/20.0,Fs=20,kpw=5,kiw=2,kdw=0.1,uw=0;
int counter=0,i=0;
double PIDW(double besttheta,double kpw,double kiw,double kdw);



void odomObs1_callback(const nav_msgs::Odometry::ConstPtr& odomobs1_msg){
    obs1v=odomobs1_msg->twist.twist.linear.x;
    obs1pose2d.x = odomobs1_msg->pose.pose.position.x;
    obs1pose2d.y = odomobs1_msg->pose.pose.position.y;
    tf::Quaternion q(
        odomobs1_msg->pose.pose.orientation.x,
        odomobs1_msg->pose.pose.orientation.y,
        odomobs1_msg->pose.pose.orientation.z,
        odomobs1_msg->pose.pose.orientation.w
                );
    tf::Matrix3x3 m(q);
    m.getRPY(obs1roll, obs1pitch, obs1yaw);

    obs1pose2d.theta = obs1yaw;
    //ROS_INFO("oYaw: %f",obs1yaw*R2D);
//ROS_INFO("Seq: [%d]", odomobs1_msg->header.seq);
//ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", odomobs1_msg->pose.pose.position.x,odomobs1_msg->pose.pose.position.y, odomobs1_msg->pose.pose.position.z);
//ROS_INFO("VelX: [%f], Wz: [%f]", odomobs1_msg->twist.twist.linear.x,odomobs1_msg->twist.twist.angular.z);
}
int main(int argc, char **argv) {
  // Create a node to run the code
    ros::init(argc, argv, "secondburger_obstacle");
    ros::NodeHandle n1;// first obstacle

    // create a subscriber to the "/odom" topic so we can get the odometry message
    ros::Publisher velpub = n1.advertise<geometry_msgs::Twist>("/tb3_2/cmd_vel", 1);
    odomObs1_sub     = n1.subscribe("/tb3_2/odom", 10, odomObs1_callback);
    log_publisher    =n1.advertise<geometry_msgs::Twist>("logger",10);
    ros::Duration(2).sleep();
    double vx=0.0;
    velmsg.linear.x=vx;
    velpub.publish(velmsg);

    vector<float> thetavect={10,30,120,-120};
    ros::Rate loop_rate(20);
    while (ros::ok()){

    uw=PIDW(thetavect[i]*M_PI/180,kpw,kiw,kdw);
    velmsg.angular.z=uw;
    velpub.publish(velmsg);

    counter++;
    if(counter>=100){
        i++;
        counter=0;
    }
    if (i>3)
        i=0;
    ros::spinOnce();
    loop_rate.sleep();
    }

    return 0;
}

double PIDW(double besttheta,double kpw,double kiw,double kdw){
    double e=0,kpError=0,kiError=0,kdError=0,uw=0;
    e=besttheta-double(obs1pose2d.theta);

    kpError=kpw*e;
    kiError=kiw*(kiError+e*Ts);
    kdError=kdw*(e-kdError)/Ts;
    uw=kpError+kiError+kdError;
    if (uw>0.5 || uw<-0.5){
        if(uw>0){
            uw=0.5;
        }
        else{
            uw=-0.5;
        }
    }
//    logdata.linear.x=kpw;
//    logdata.linear.y=kiw;
//    logdata.linear.z=kdw;
//    logdata.angular.x=uw;
//    logdata.angular.y=e;
//    log_publisher.publish(logdata);
    ROS_INFO("counter : %d, theta:%f, Besttheta: %f, error : %f , uw: %f",counter,obs1pose2d.theta,besttheta,e,uw);
    ROS_INFO("kp: %f, ki: %f, kd: %f : ",kpw,kiw,kdw);
    ROS_INFO("Ekp: %f, Eki: %f, Ekd: %f : , Ts: %f",kpError,kiError,kdError,Ts);


    return uw;
}

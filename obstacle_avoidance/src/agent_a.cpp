#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"
#include <map>
#include <string>
#include <iterator>
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <math.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Subscriber laser_subscriber;
ros::Subscriber odomObs1_sub;
ros::Subscriber odomagnt1_sub;

sensor_msgs::LaserScan laser_msg;
nav_msgs::Odometry odomObs1_msg;



bool rasa=true,actioncancel=true;

//general vatiables
double R2D=180/M_PI, D2R=M_PI/180;
int observation=0;
bool cancelgaols=true;
int hz=20;
//end

//lidar
float max_range=3.5;
std::vector<float> laser_ranges;
//end

//agent 1 properties
float agent1v=0.1; // meter/sec
//end

//odom obsacle1 and agent1
geometry_msgs::Pose2D agent1pose2d;
geometry_msgs::Pose2D obs1pose2d;
double obs1roll, obs1pitch, obs1yaw;
double agent1roll, agent1pitch, agent1yaw;
bool obstaclexist=false;
float obs1v=0.1; // meter/sec

//end

//map and locations
int loc1=1;
int loc2=2;
int loc3=3;
int loc4=4;
float x[4][3] = {{4,0,1}, {-11,-9,1}, {-10.5,6,1},{2,7,1}}; // goal point
float rasapos[3]={0,0,0};
// end

//Variables in RVO algorithm
double dis_agent_obs=0;
float agnte1Rad=0.2,obs1Rad=0.2,NR=max_range,agentrelvx=0,agentrelvy=0;
float rel_yaw=0;
float losangle=0;
float r_mink=obs1Rad+agnte1Rad/2; // minkowski circle with more radius than obstacle radius
float collision_time=10000;
bool c1,c2,c3,c4;
//end

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
// Read and process laser scan values
    laser_msg = *scan_msg;

    laser_ranges = laser_msg.ranges;
    size_t range_size = laser_ranges.size();
    float left_side = 0.0, right_side = 0.0;
    float range_min = laser_msg.range_min, range_max = laser_msg.range_max;
    if((*min_element (laser_ranges.begin(), laser_ranges.end()))>0 and (*min_element (laser_ranges.begin(), laser_ranges.end()) <=max_range )){
       obstaclexist=true;
   // ROS_INFO("range_min: %f, range_max: %f, range_size: %f",range_min,range_max,range_size);
//    for(int i=0;i<range_size;i++){
//        ROS_INFO("laser output:  %f",laser_ranges[i]);

//    }
    ROS_INFO(" minimum range: %f ",*min_element (laser_ranges.begin(), laser_ranges.end()));}
    else {
         obstaclexist=false;
    }

}

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

void odomagent1_callback(const nav_msgs::Odometry::ConstPtr& odomoagetn1_msg){
//ROS_INFO("Agetn 1 Seq: [%d]", odomoagetn1_msg->header.seq);
//ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", odomoagetn1_msg->pose.pose.position.x,odomoagetn1_msg->pose.pose.position.y, odomoagetn1_msg->pose.pose.position.z);
//ROS_INFO("VelX: [%f], Wz: [%f]", odomoagetn1_msg->twist.twist.linear.x,odomoagetn1_msg->twist.twist.angular.z);
    agent1pose2d.x = odomoagetn1_msg->pose.pose.position.x;
    agent1pose2d.y = odomoagetn1_msg->pose.pose.position.y;
    tf::Quaternion q(
        odomoagetn1_msg->pose.pose.orientation.x,
        odomoagetn1_msg->pose.pose.orientation.y,
        odomoagetn1_msg->pose.pose.orientation.z,
        odomoagetn1_msg->pose.pose.orientation.w
                );
    tf::Matrix3x3 m(q);
    m.getRPY(agent1roll, agent1pitch, agent1yaw);

    agent1pose2d.theta = agent1yaw;
    //ROS_INFO("agent 1 Yaw: %f",agent1yaw*R2D);
}



int main(int argc, char** argv){

ros::init(argc, argv, "agent_a");
ros::NodeHandle n;
//tell the action client that we want to spin a thread by default
MoveBaseClient ac("/tb3_1/move_base", true);
laser_subscriber = n.subscribe("/tb3_1/scan", 10, laser_callback);
odomObs1_sub     = n.subscribe("/tb3_2/odom", 10, odomObs1_callback);
odomagnt1_sub    = n.subscribe("/tb3_1/odom", 10, odomagent1_callback);
ros::Rate loop_rate(hz);
//wait for the action server to come up
while(!ac.waitForServer(ros::Duration(5.0))){
ROS_INFO("Waiting for the move_base action server to come up");
}

move_base_msgs::MoveBaseGoal goal;

//we'll send a goal to the robot to move 1 meter forward
goal.target_pose.header.frame_id = "map";
goal.target_pose.header.stamp = ros::Time::now();

goal.target_pose.pose.position.x = x[0][0];
goal.target_pose.pose.position.y = x[0][1];
goal.target_pose.pose.orientation.w = x[0][2];
ROS_INFO("The Goal position is : x: %f y: %f theta: %f", x[0][0], x[0][1], x[0][2]);
ac.sendGoal(goal);
ROS_INFO("Sending goal");
ros::Duration(2.0).sleep();
while (ros::ok()){

    if(obstaclexist){
        observation++;
        //ROS_INFO(" minimum range: %f , observation: %d, anget 1 Yaw angle : %f ",*min_element (laser_ranges.begin(), laser_ranges.end()),observation,(agent1yaw*R2D));
        if(cancelgaols){
            ROS_INFO("cancel all goals");
            ac.cancelAllGoals();
            cancelgaols=false;
        }

        dis_agent_obs=ceil(sqrt((agent1pose2d.x-obs1pose2d.x)*(agent1pose2d.x-obs1pose2d.x)+(agent1pose2d.y-obs1pose2d.y)*(agent1pose2d.y-obs1pose2d.y))*100.0)/100.0; // unite is meter and Round to Two Decimal Places
        if(dis_agent_obs<NR){
            agentrelvx=agent1v*cos(agent1yaw)-obs1v*cos(obs1yaw);
            agentrelvy=agent1v*sin(agent1yaw)-obs1v*sin(obs1yaw);
            rel_yaw=ceil((atan2(agentrelvy,agentrelvx))*100.0)/100.0;
            losangle=ceil((atan2((obs1pose2d.x-agent1pose2d.x),(obs1pose2d.y-agent1pose2d.y)))*100.0)/100.0;
        if(dis_agent_obs!=0)
            losangle=ceil((asin(r_mink/dis_agent_obs))*100)/100.0;

        else
            losangle=ceil(M_PI/2*100)/100.0;

        if(agentrelvx-(agent1v*(cos(agent1yaw)))<=0)
            c1=true;
        else if (agentrelvx-(agent1v*(cos(obs1yaw)))<=0)
            c2=true;
        else if (agentrelvy-(agent1v*(sin(agent1yaw)))<=0)
            c3=true;
        else if (agentrelvy-(agent1v*(sin(obs1yaw)))<=0)
            c4=true;
        if(c1 ||c2 || c3 || c4)
           // collision_time=abs(self.all_agents_pose_dict[i][0] - self.pose.x)/abs(v_mag * (cos(self.pose.theta) - cos(self.all_agents_pose_dict[i][2])))
        }








        ROS_INFO("agent 1 Yaw: %f",agent1yaw*R2D);
    }

    else {
        ROS_INFO("No obstackle in 3.5 radius");
        cancelgaols=true;
    }



    //ac.waitForResult();
    actionlib::SimpleClientGoalState state = ac.getState();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Action finished: %s",state.toString().c_str());
    break;
    // ROS_INFO("Hooray, the base moved 1 meter forward");
    }
    else if (ac.getState() == actionlib::SimpleClientGoalState::LOST){
    //ROS_INFO("Action failed: LOST");
    ROS_INFO("no goal recieved");
    }
    ros::spinOnce();
    loop_rate.sleep();
    }
return 0;
}

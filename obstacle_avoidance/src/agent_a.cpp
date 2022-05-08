#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"
#include <map>
#include <string>
#include <iterator>
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/Odometry.h>
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Subscriber laser_subscriber;
ros::Subscriber odomObs1_sub;

sensor_msgs::LaserScan laser_msg;
nav_msgs::Odometry odomObs1_msg;
bool rasa=true,actioncancel=false;
Obs1Rad=0.2
agnte1Rad=0.2
int observation=0;
int loc1=1;
int loc2=2;
int loc3=3;
int loc4=4;
float x[4][3] = {{4,0,1}, {-11,-9,1}, {-10.5,6,1},{2,7,1}}; // goal point
float rasapos[3]={0,0,0};
        // insert elements in random order
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    // Read and process laser scan values
    laser_msg = *scan_msg;
    std::vector<float> laser_ranges;
    laser_ranges = laser_msg.ranges;
    size_t range_size = laser_ranges.size();
    float left_side = 0.0, right_side = 0.0;
    float range_min = laser_msg.range_min, range_max = laser_msg.range_max;
//    ROS_INFO("range_min: %f, range_max: %f, range_size: %f",range_min,range_max,range_size);
//    for(int i=0;i<range_size;i++){
//        ROS_INFO("laser output:  %f",laser_ranges[i]);

//    }
    ROS_INFO(" minimum range: %f ",*min_element (laser_ranges.begin(), laser_ranges.end()));
    if((*min_element (laser_ranges.begin(), laser_ranges.end()))<3 ){
        ROS_INFO(" minimum range: %f ",*min_element (laser_ranges.begin(), laser_ranges.end()));
        //ROS_INFO("Cancel all goals!");
        //ROS_INFO("wait for 5 second!!");
        //ROS_INFO("observation: %d",observation);
        //ros::Duration(5.0).sleep();
        actioncancel=true;
        observation++;

    }
}

void odomObs1_callback(const nav_msgs::Odometry::ConstPtr& odomobs1_msg){
    ROS_INFO("Seq: [%d]", odomobs1_msg->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", odomobs1_msg->pose.pose.position.x,odomobs1_msg->pose.pose.position.y, odomobs1_msg->pose.pose.position.z);
    ROS_INFO("VelX: [%f], Wz: [%f]", odomobs1_msg->twist.twist.linear.x,odomobs1_msg->twist.twist.angular.z);
}


int main(int argc, char** argv){

  ros::init(argc, argv, "agent_a");
  ros::NodeHandle n;
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("/tb3_1/move_base", true);
  laser_subscriber = n.subscribe("/tb3_1/scan", 10, laser_callback);
  odomObs1_sub     = n.subscribe("/tb3_2/odom", 10, odomObs1_callback);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  while (1){
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = x[0][0];
      goal.target_pose.pose.position.y = x[0][1];
      goal.target_pose.pose.orientation.w = x[0][2];

      if (rasa){
        ROS_INFO("The Goal position is : x: %f y: %f theta: %f", x[0][0], x[0][1], x[0][2]);
        ac.sendGoal(goal);
        ROS_INFO("Sending goal");
        rasa=false;
      }

      if(actioncancel && observation<2){
      //ros::Duration(10.0).sleep();
      ROS_INFO("cancel all goals");
      ac.cancelAllGoals();
      ROS_INFO("wait for 5 second");
      ros::Duration(5.0).sleep();
      rasa=true;
      actioncancel=false;
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
  }
  return 0;
}

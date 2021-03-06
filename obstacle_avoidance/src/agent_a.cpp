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
#include "geometry_msgs/Twist.h"
#include <vector>
#include<bits/stdc++.h>
#include <obstacle_avoidance/Points.h>


using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher  velpubagent1;
ros::Publisher  Pointspubagent1;

ros::Subscriber laser_subscriber;
ros::Subscriber odomObs1_sub;
ros::Subscriber odomagnt1_sub;

sensor_msgs::LaserScan laser_msg;
nav_msgs::Odometry odomObs1_msg;
geometry_msgs::Twist velmsgagent1;
// use these point to create the local cost map
obstacle_avoidance::Points pobs1;
float x13,x23,x14,x24,y13,y23,y14,y24,offset1,offset2;

bool rasa=true,actioncancel=true;
//local cost map variables
float resmap=0.01;


//end

//general vatiables
double R2D=180.0/M_PI, D2R= M_PI/180.0;
int observation=0;
bool cancelgaols=true;
int hz=20;
float Ts=1.0/hz,kpw=5,kiw=2,kdw=0.1,uw=0;
bool updatervo=true;
int rvocounter=0,goalcounter=0;
//end

//lidar
float max_range=3.5;
std::vector<float> laser_ranges;
//end

//agent 1 properties
float agent1v=0.1,vx=0; // meter/sec
//end

//odom obsacle1 and agent1
geometry_msgs::Pose2D agent1pose2d;
geometry_msgs::Pose2D obs1pose2d;
double obs1roll, obs1pitch, obs1yaw;
double agent1roll, agent1pitch, agent1yaw;
bool obstaclexist=false;
float obs1v=0.0; // obstacle velocity is 0 meter/sec

//end

//map and locations
int loc1=1;
int loc2=2;
int loc3=3;
int loc4=4;
float x[4][3] = {{6,0,1}, {-11,-9,1}, {-10.5,6,1},{2,7,1}}; // goal point
float rasapos[3]={0,0,0};
// end

//Variables in RVO algorithm
double dis_agent_obs=0;
float agnte1Rad=0.2,obs1Rad=0.2,NR=max_range,agentrelvx=0,agentrelvy=0;
float rel_yaw=0;
float losangle=0,losangle2=0, alpha=0,alpha2=0;
vector<float> alpharange={0.0,0.0},alpharange2={0.0,0.0};// cone angle min and max [min,max]
vector<float> RVO={0.0, 0.0}; // RVO range base on new cone and its apex.
float r_mink=obs1Rad+agnte1Rad/2; // minkowski circle with more radius than obstacle radius
float collision_time=10000;
bool c1,c2,c3,c4;
float num=0,den=0;
float least_distance=10;
float incr=0.01;// RVO resolution
float desire_heading=0,bestheading=0;
int indexbest=0;

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
    //ROS_INFO(" minimum range: %f ",*min_element (laser_ranges.begin(), laser_ranges.end()));
    }
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
    pobs1.x[1]=odomobs1_msg->pose.pose.position.x;
    pobs1.y[1]=odomobs1_msg->pose.pose.position.y;
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
    pobs1.x[0]=odomoagetn1_msg->pose.pose.position.x;
    pobs1.y[0]=odomoagetn1_msg->pose.pose.position.y;
    //ROS_INFO("agent 1 Yaw: %f",agent1yaw*R2D);
}



double PIDW(double besttheta,double kpw,double kiw,double kdw){
    double e=0,kpError=0,kiError=0,kdError=0,uw=0;
    e=besttheta-double(agent1pose2d.theta);

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
//    ROS_INFO("c: %d, rvoc: %d, theta:%f, Besttheta: %f, error : %f , uw: %f",goalcounter,rvocounter,agent1pose2d.theta*R2D,besttheta*R2D,e,uw);
    //ROS_INFO("kp: %f, ki: %f, kd: %f : ",kpw,kiw,kdw);
    //ROS_INFO("Ekp: %f, Eki: %f, Ekd: %f : , Ts: %f",kpError,kiError,kdError,Ts);


    return uw;
}

int main(int argc, char** argv){

ros::init(argc, argv, "agent_a");
ros::NodeHandle n;
//tell the action client that we want to spin a thread by default
MoveBaseClient ac("/tb3_1/move_base", true);
laser_subscriber = n.subscribe("/tb3_1/scan", 10, laser_callback);
odomObs1_sub     = n.subscribe("/tb3_2/odom", 10, odomObs1_callback);
odomagnt1_sub    = n.subscribe("/tb3_1/odom", 10, odomagent1_callback);
velpubagent1     = n.advertise<geometry_msgs::Twist>("/tb3_1/cmd_vel", 1);
Pointspubagent1  = n.advertise<obstacle_avoidance::Points>("/pointscostmap/obstacl1", 1);

vector<float> heading_vect;// -pi to pi
for (float i=-M_PI;i<=M_PI;i=i+incr)
    heading_vect.push_back(ceil(i*100.0)/100.0);

for (auto it = heading_vect.begin(); it != heading_vect.end(); it++)
    ROS_INFO("heading vector : %f",*it);
//float k=0.0;
//for (int j=0;j<pobs1.x.size();j++) {
//    k=k+0.01;
//    pobs1.x[j]=k;
//    pobs1.y[j]=k;

//}
//for (int j=0;j<pobs1.x.size();j++){
//    ROS_INFO("x: %f y: %f",pobs1.x[j],pobs1.y[j]);
//    Pointspubagent1.publish(pobs1);
//    ros::Duration(1.0).sleep();
//}

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
            losangle=ceil((atan2((obs1pose2d.y-agent1pose2d.y),(obs1pose2d.x-agent1pose2d.x)))*100.0)/100.0;

        if(dis_agent_obs>=r_mink)
            alpha=ceil((asin(r_mink/dis_agent_obs))*100)/100.0;
        else
            alpha=ceil(M_PI/2*100)/100.0;
        //ROS_INFO("agentrelvx: %f agentrelvy: %f rel_yaw: %f losangle: %f alpha: %f",agentrelvx,agentrelvy,rel_yaw*R2D,losangle*R2D,alpha*R2D);

        if(agentrelvx-(agent1v*(cos(agent1yaw)))<=0)
            c1=true;
        else if (agentrelvx-(agent1v*(cos(obs1yaw)))<=0)
            c2=true;
        else if (agentrelvy-(agent1v*(sin(agent1yaw)))<=0)
            c3=true;
        else if (agentrelvy-(agent1v*(sin(obs1yaw)))<=0)
            c4=true;
        if(c1 ||c2 || c3 || c4)
           collision_time=abs((obs1pose2d.x-r_mink*cos(losangle))-agent1pose2d.x)/abs(agentrelvx);
        alpharange[0]=losangle-alpha;
        alpharange[1]=losangle+alpha;
        num=agent1v*sin(agent1yaw)+obs1v*sin(obs1yaw);
        den=agent1v*cos(agent1yaw)+obs1v*cos(obs1yaw);
        RVO[0]=ceil(((alpharange[0]+atan2(num,den))/2)*100.0)/100.0;
        RVO[1]=ceil(((alpharange[1]+atan2(num,den))/2)*100.0)/100.0;
        if(dis_agent_obs<least_distance)
            least_distance=dis_agent_obs;
        vx=(least_distance/3)/NR;
        if(losangle<0 && losangle>-3.14){
            offset1=-r_mink;
            offset2=+0.666*dis_agent_obs; // 2/3*dis

        }
        else {
            offset1=r_mink;
            offset2=-0.666*dis_agent_obs;
        }
        losangle2=ceil((atan((obs1pose2d.y-agent1pose2d.y)/(obs1pose2d.x-agent1pose2d.x)))*100.0)/100.0;
        alpharange2[0]=losangle2-alpha;
        alpharange2[1]=losangle2+alpha;
            ROS_INFO("losangle %f, alpha %f ,alpharange[0] %f ,alpharange[1] %f dis_agent_obs %f",losangle2,alpha,alpharange2[0],alpharange2[1],dis_agent_obs);
            float a=(alpharange[0]+alpharange[1])/2;
//            x13=((obs1pose2d.x/losangle)+obs1pose2d.y+ alpharange[0]*agent1pose2d.x-agent1pose2d.y)/(alpharange[0]+(1/losangle));
//            x23=((obs1pose2d.x/losangle)+obs1pose2d.y+ alpharange[1]*agent1pose2d.x-agent1pose2d.y)/(alpharange[1]+(1/losangle));
    //        x14=((obs1pose2d.x/losangle)+obs1pose2d.y+offset2+ alpharange[0]*agent1pose2d.x-agent1pose2d.y)/(alpharange[0]+(1/losangle));
    //        x24=((obs1pose2d.x/losangle)+obs1pose2d.y+offset2+ alpharange[1]*agent1pose2d.x-agent1pose2d.y)/(alpharange[1]+(1/losangle));
//            y13=alpharange[0]*(x13-agent1pose2d.x)+agent1pose2d.y;
//            y23=alpharange[1]*(x23-agent1pose2d.x)+agent1pose2d.y;
    //        y14=alpharange[0]*(x14-agent1pose2d.x)+agent1pose2d.y;
    //        y24=alpharange[1]*(x24-agent1pose2d.x)+agent1pose2d.y;
            pobs1.x[2]= alpharange2[0];
            pobs1.x[3]= alpharange2[1];
            pobs1.x[4]=losangle2;

            pobs1.x[5]=dis_agent_obs;
            pobs1.y[2]=-10;
            pobs1.y[3]=-10;
            pobs1.y[4]=-10;
            pobs1.y[5]=-10;
        //ROS_INFO("(%f,%f),(%f,%f),(%f,%f),(%f,%f)",agent1pose2d.x,agent1pose2d.y,obs1pose2d.x,obs1pose2d.y,x13,y13,x23,y23);

        //ROS_INFO("dis_agent_obs<NR %f, Vx: %f, collision_time: %f",dis_agent_obs,vx,collision_time);
        Pointspubagent1.publish(pobs1);
        }
        //ROS_INFO("alpharange[0]: %f < agent1yaw: %f < alpharange[1]: %f",alpharange[0]*R2D,agent1yaw*R2D,alpharange[1]*R2D);
        //ROS_INFO("num %f ,den %f , atan2: %f",num*R2D,den*R2D,atan2(num,den)*R2D);
        //ROS_INFO("RVO[0]:        %f < agent1yaw: %f < RVO[1]:        %f",RVO[0],agent1yaw,RVO[1]);
        //

////check collision probability
        if(RVO[0]<agent1yaw && agent1yaw<RVO[1]){ // if true, will be collision in moment of time
            desire_heading=ceil((atan2((x[0][1]-agent1pose2d.y),(x[0][0]-agent1pose2d.x)))*100.0)/100.0;
           // ROS_INFO("desire_heading %f",desire_heading*R2D);
            vector<float> absheading_vect;
            vector<float> heading_vect2;
            heading_vect2=heading_vect;
            vector<float>::iterator it=find(heading_vect2.begin(),heading_vect2.end(), RVO[0]); //find index min RVO in Circle -pi,+pi
            vector<float>::iterator it2=find(heading_vect2.begin(),heading_vect2.end(),RVO[1] );//find index max RVO in Circle -pi,+pi
            vector<float>::iterator it3,it4;
           // ROS_INFO("it : %d it2 : %d diff : %d",it,it2,it2-it);
            //RVO region
            for (int k=0;k<(it2-it);k++)//removing RVO[0]<Yaw<RVO[1] from [-pi,+pi]-> new area outside RVO
                heading_vect2.erase(it);

            for ( int k=0;k<heading_vect2.size();k++)//removing desired heading from new area outside RVO
                absheading_vect.push_back(abs(heading_vect2[k]-desire_heading));

            indexbest=min_element (absheading_vect.begin(), absheading_vect.end())-absheading_vect.begin();//find minimum value and its index
            bestheading=heading_vect[indexbest];
            rvocounter++;
            updatervo=true;
        }
        else {
            bestheading=ceil((atan2((x[0][1]-agent1pose2d.y),(x[0][0]-agent1pose2d.x)))*100.0)/100.0;
            updatervo=false;
        }
        if ((ceil(sqrt((agent1pose2d.x-x[0][0])*(agent1pose2d.x-x[0][0])+(agent1pose2d.y-x[0][1])*(agent1pose2d.y-x[0][1]))*100.0)/100.0)>0.01 ){

       // ROS_INFO("Besttheta %f",bestheading*R2D);
        uw=PIDW(bestheading,kpw,kiw,kdw);
        velmsgagent1.angular.z=uw;
        velmsgagent1.linear.x=agent1v;
        velpubagent1.publish(velmsgagent1);
        }
        else {
            uw=PIDW(x[0][3],kpw,kiw,kdw);
            ROS_INFO("Vx is zero wz =x[0][3]");
            velmsgagent1.angular.z=uw;
            velmsgagent1.linear.x=0;
            velpubagent1.publish(velmsgagent1);
        }
    }
    else {
        //ROS_INFO("No obstackle in 3.5 radius");
        cancelgaols=true;
        goalcounter++;
        if(abs(goalcounter-rvocounter)>300 && !updatervo ){
            rvocounter=0;
            goalcounter=0;
            ac.sendGoal(goal);
            ROS_INFO("Sending goal");
        }
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

#include</home/morteza/catkin-ws-test/src/simple_layers/include/simple_layers/simple_layer.h>
#include <pluginlib/class_list_macros.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
/***   Client  ****/
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>
/***   Client  ****/

#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>
#include "simple_layers/PointCallBack.h"

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)  //注册插件

using costmap_2d::LETHAL_OBSTACLE;
namespace simple_layer_namespace
{
SimpleLayer::SimpleLayer() {}
boost::recursive_mutex lock_;

std::vector<float> req_X1;
std::vector<float> req_Y1;
std::vector<float> req_static_obstacle_X1;
std::vector<float> req_static_obstacle_Y1;

float x1 =0,y1=0,x2=0,y2=0,x3=0,y3=0,x4=0,y4=0;
float x11 =0,y11=0,x12=0,y12=0,x13=0,y13=0,x14=0,y14=0;
float x31 =0,y31=0,x32=0,y32=0,x33=0,y33=0,x34=0,y34=0;
std_srvs::Empty srv;

void VelocityObstacleCB(const visualization_msgs::Marker& line_list)
{
  //std::cout <<"##############################################"<<std::endl;
  x1=line_list.points.at(1).x;
  x2=line_list.points.at(2).x;
  x3=line_list.points.at(3).x;
  x4=line_list.points.at(4).x;
  y1=line_list.points.at(1).y;
  y2=line_list.points.at(2).y;
  y3=line_list.points.at(3).y;
  y4=line_list.points.at(4).y;
}


void SimpleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SimpleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void SimpleLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
 // if (!enabled_)
  //  return;
  boost::recursive_mutex::scoped_lock lock(lock_);

  mark_x_ = origin_x ;//+ cos(origin_yaw);
  mark_y_ = origin_y  ;//+ sin(origin_yaw);

  *min_x = std::min(*min_x, mark_x_-5);
  *min_y = std::min(*min_y, mark_y_-5);
  *max_x = std::max(*max_x, mark_x_+5);
  *max_y = std::max(*max_y, mark_y_+5);
}

double a ,b; //計算斜率

void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<simple_layers::PointCallBack>("add_two_ints");
    simple_layers::PointCallBack srv;
    boost::recursive_mutex::scoped_lock lock(lock_);

    srv.request.a = 1;
    if(client.call(srv))
    {
      req_X1=srv.response.array_line_point_X1;
      req_Y1=srv.response.array_line_point_Y1;
      req_static_obstacle_X1=srv.response.static_obstacle_array_X1;
      req_static_obstacle_Y1=srv.response.static_obstacle_array_Y1;

    }
    else
    {
        std::cout<<"wait for service"<<std::endl;
    }

  float x_array[50];
  float y_array[50];

  unsigned int mx1,my1;
  int i =0;
  double segment;
  double sample_x;
  double sample_y;
  double square_range=2.5; //+-2公尺內才會畫
  // std::cout << "req_[DYNAMIC]_obstacle_size =  "<<req_Y1.size() <<std::endl;
  if(req_Y1.size()>4)
  {
    for( i = 0; i< req_Y1.size();i++)
    {
      x_array[i] = req_X1.at(i);
      y_array[i] = req_Y1.at(i);
      //std::cout << x_array[i] <<std::endl;
      //std::cout << y_array[i] <<std::endl;
    }

    for(int j =0;j<(req_Y1.size()/4);j++)
    {
        i =0;
        a=(y_array[j*4+3]-y_array[j*4])/(x_array[j*4+3]-x_array[j*4+0]);
        b=y_array[j*4]-a*x_array[j*4];
        segment = (x_array[j*4+3]-x_array[j*4])/19;
        sample_x = x_array[j*4];
        sample_y = y_array[j*4];

        while(i<20){
          if(mark_x_+square_range>sample_x && mark_x_-3.0<sample_x && mark_y_+square_range>sample_y && mark_y_-square_range<sample_y )
          {
            master_grid.worldToMap(sample_x, sample_y, mx1, my1);
            master_grid.setCost(mx1, my1, LETHAL_OBSTACLE);
          }
          sample_x+=segment;
          sample_y=a*sample_x+b;
          i++;
        }
      /*##################11####33############################################*/
        i =0;
        a=(y_array[j*4+2]-y_array[j*4])/(x_array[j*4+2]-x_array[j*4]);
        b=y_array[j*4]-a*x_array[j*4];
        segment = (x_array[j*4+2]-x_array[j*4])/19;
        sample_x =x_array[j*4];
        sample_y = y_array[j*4];
        while(i<20){
          if(mark_x_+square_range>sample_x && mark_x_-square_range<sample_x && mark_y_+square_range>sample_y && mark_y_-square_range<sample_y )
          {
            master_grid.worldToMap(sample_x, sample_y, mx1, my1);
            master_grid.setCost(mx1, my1, LETHAL_OBSTACLE);
          }
          sample_x+=segment;
          sample_y=a*sample_x+b;
          i++;
        }
      /*##################222####444############################################*/
        i =0;
        a=(y_array[j*4+3]-y_array[j*4+1])/(x_array[j*4+3]-x_array[j*4+1]);
        b=y_array[j*4+1]-a*x_array[j*4+1];
        segment = (x_array[j*4+3]-x_array[j*4+1])/19;
        sample_x =x_array[j*4+1];
        sample_y = y_array[j*4+1];
        while(i<20){
          if(mark_x_+square_range>sample_x && mark_x_-square_range<sample_x && mark_y_+square_range>sample_y && mark_y_-square_range<sample_y )
          {
            master_grid.worldToMap(sample_x, sample_y, mx1, my1);
            master_grid.setCost(mx1, my1, LETHAL_OBSTACLE);
          }
          sample_x+=segment;
          sample_y=a*sample_x+b;
          i++;
        }
        /*######################222#########333###################################*/
        i =0;
        a=(y_array[j*4+2]-y_array[j*4+1])/(x_array[j*4+2]-x_array[j*4+1]);
        b=y_array[j*4+1]-a*x_array[j*4+1];
        segment = (x_array[j*4+2]-x_array[j*4+1])/19;
        sample_x =x_array[j*4+1];
        sample_y = y_array[j*4+1];
        while(i<20){
          if(mark_x_+square_range>sample_x && mark_x_-square_range<sample_x && mark_y_+square_range>sample_y && mark_y_-square_range<sample_y )
          {
            master_grid.worldToMap(sample_x, sample_y, mx1, my1);
            master_grid.setCost(mx1, my1, LETHAL_OBSTACLE);
          }
          sample_x+=segment;
          sample_y=a*sample_x+b;
          i++;
        }
    }
  }


  float static_x_array[100];
  float static_y_array[100];
  float x_segment_distance;
  float y_segment_distance;
  // std::cout << "req_[STATIC]_obstacle_size = "<< req_static_obstacle_X1.size() <<std::endl;
  if(req_static_obstacle_X1.size()>4)
  {

    for( i = 0; i< req_static_obstacle_X1.size();i++)
    {
      static_x_array[i] = req_static_obstacle_X1.at(i);
      static_y_array[i] = req_static_obstacle_Y1.at(i);
      // std::cout << "static_X_array ["<< i <<"] =" <<static_x_array[i] <<std::endl;
      // std::cout << "static_Y_array ["<< i <<"] =" <<static_y_array[i] <<std::endl;

    }
/*
0    1
******          +x
*    *
*    *   +y    (0,0)     -y
******
3    2          -x
*/

    for(int j =0;j<(req_static_obstacle_X1.size()/4);j++)
    {
        x_segment_distance=(static_x_array[j*4]-static_x_array[j*4+3])/19;
        y_segment_distance=(static_y_array[j*4]-static_y_array[j*4+1])/19;
        sample_x = static_x_array[j*4+0];
        sample_y = static_y_array[j*4+0];
        i=0;
        while(i<20){
          master_grid.worldToMap(static_x_array[j*4], (static_y_array[j*4]-i*y_segment_distance), mx1, my1);  //0->1
          master_grid.setCost(mx1, my1, LETHAL_OBSTACLE);
          master_grid.worldToMap(static_x_array[j*4+3], (static_y_array[j*4+3]-i*y_segment_distance), mx1, my1);//3->2
          master_grid.setCost(mx1, my1, LETHAL_OBSTACLE);

          master_grid.worldToMap((static_x_array[j*4+1]-i*y_segment_distance), static_y_array[j*4+1], mx1, my1);//1->2
          master_grid.setCost(mx1, my1, LETHAL_OBSTACLE);
          master_grid.worldToMap((static_x_array[j*4]-i*y_segment_distance), static_y_array[j*4], mx1, my1);//0->3
          master_grid.setCost(mx1, my1, LETHAL_OBSTACLE);
          i++;
        }

    }

  }
}
} // end namespace


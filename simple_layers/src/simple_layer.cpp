#include<simple_layers/simple_layer.h>
#include <pluginlib/class_list_macros.h>
#include <vector>
#include <obstacle_avoidance/Points.h>
#include <math.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using namespace std;
bool flag=false;
namespace simple_layer_namespace
{

SimpleLayer::SimpleLayer() {

    ros::NodeHandle n("~/" + name_);
    pointssubobs1 = n.subscribe("/pointscostmap/obstacl1", 10, &simple_layer_namespace::SimpleLayer::pointsobs1Callback,this);
    for(int i=0;i<pobs1.x.size();i++){
        this->pobs1.x[i]=-10;
        this->pobs1.y[i]=-10;
    }

}
void SimpleLayer::onInitialize()
{

  ros::NodeHandle nh("~/" + name_);

  current_ = true;

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SimpleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  //local cost map

}

void SimpleLayer::pointsobs1Callback(const obstacle_avoidance::Points& points)
{


    xr=         points.x[0];
    yr=         points.y[0];
    xo=         points.x[1];
    yo=         points.y[1];
    alphan=     points.x[2];
    alphap=     points.x[3];
    losangle=   points.x[4];
    dis=        points.x[5];
    flag=true;

}

void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void SimpleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{

  if (!enabled_)
    return;

    mark_x_=robot_x;
    mark_y_=robot_y;
    *min_x = std::min(*min_x, mark_x_-8);
    *min_y = std::min(*min_y, mark_y_-8);
    *max_x = std::max(*max_x, mark_x_+8);
    *max_y = std::max(*max_y, mark_y_+8);

}

void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  unsigned int mx;
  unsigned int my;
float disy,detax_y13,detax_y23,detay_y13,detay_y23,detax_y14,detax_y24,detay_y14,detay_y24;

vector<float> x3={0},x4={0},x13,x23,y13,y23,y3,y4;
if(flag==true){
if (sin(losangle)>=0){

    disy=sqrt((dis*dis)-0.09)+0.3;
    detax_y13=(7.0/5)*disy*cos(alphan);
    detax_y23=(7.0/5)*disy*cos(alphap);
    detay_y13=(7.0/5)*disy*sin(alphan);
    detay_y23=(7.0/5)*disy*sin(alphap);

    detax_y14=(2.0/5.0)*disy*cos(alphan);
    detax_y24=(2.0/5.0)*disy*cos(alphap);
    detay_y14=(2.0/5.0)*disy*sin(alphan);
    detay_y24=(2.0/5.0)*disy*sin(alphap);
   // ROS_INFO("p xr: %f detax_y13 %f detax_y14 %f detax_y24 %f alphan %f disy %f",xr,detax_y13,detax_y14,detax_y24,alphan,disy);

}
else {
    disy=sqrt((dis*dis)-0.09)-0.3;
    detax_y13=-(10.0/5.0)*disy*cos(alphan);//cos(pi+alphan(c))=-cos(alphan(c))
    detax_y23=-(10.0/5)*disy*cos(alphap);
    detay_y13=-(10.0/5)*disy*sin(alphan);//sin(pi+alphan(c))=-sin(alphan(c))
    detay_y23=-(10.0/5)*disy*sin(alphap);

    detax_y14=-(2.0/5)*disy*cos(alphan);
    detax_y24=-(2.0/5)*disy*cos(alphap);
    detay_y14=-(2.0/5)*disy*sin(alphan);
    detay_y24=-(2.0/5)*disy*sin(alphap);
   // ROS_INFO("n xr: %f detax_y13 %f detax_y14 %f detax_y24 %f alphan %f disy %f",xr,detax_y13,detax_y14,detax_y24,alphan,disy);

}
float res=0.001;
for(float k=xr+min(detax_y13, detax_y23);k<=xr+max(detax_y23,detax_y13);k=k+res){
//k is same as x3
    x3.push_back(k);
    y3.push_back((-1/losangle)*(k-(xr+detax_y13))+(yr+detay_y13));
    mark_x_=k;
    mark_y_=(-1/losangle)*(k-(xr+detax_y13))+(yr+detay_y13);
    if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
        master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    }
}
for(float k=xr+min(detax_y14, detax_y24);k<=xr+max(detax_y24,detax_y14);k=k+res){
//k is same as x4
    x4.push_back(k);
    y4.push_back((-1/losangle)*(k-(xr+detax_y14))+(yr+detay_y14));
    mark_x_=k;
    mark_y_=(-1/losangle)*(k-(xr+detax_y14))+(yr+detay_y14);
    if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
        master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    }
}
//ROS_INFO("x3.size()-1 %d x4.size()-1 %d",x3.size()-1,x4.size()-1);
for(float k=min(x3[x3.size()-1], x4[x4.size()-1]);k<=max(x3[x3.size()-1], x4[x4.size()-1]);k=k+res){
//k is same as x13
    x13.push_back(k);
    y13.push_back((alphan)*(k-xr)+yr);
    mark_x_=k;
    mark_y_=(alphan)*(k-xr)+yr;
    if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
        master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    }
}
for(float k=min(x3[1], x4[1]);k<=max(x3[1], x4[1]);k=k+res){//x[0]={0}!
//k is same as x13
    x23.push_back(k);
    y23.push_back((alphap)*(k-xr)+yr);
    mark_x_=k;
    mark_y_=(alphap)*(k-xr)+yr;
    if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
        master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    }
}
flag=false;
}
}

} // end namespace

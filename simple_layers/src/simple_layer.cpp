#include<simple_layers/simple_layer.h>
#include <pluginlib/class_list_macros.h>
#include <vector>
#include <obstacle_avoidance/Points.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using namespace std;
namespace simple_layer_namespace
{

SimpleLayer::SimpleLayer() {

    ros::NodeHandle n("~/" + name_);
    pointssubobs1 = n.subscribe("/pointscostmap/obstacl1", 10, &simple_layer_namespace::SimpleLayer::pointsobs1Callback,this);

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
    pobs1.x[2]=points.x[2];
    pobs1.y[2]=points.y[2];
  ROS_INFO("I heard x: [%f] and y: [%f]", points.x[2],points.y[2]);
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

//  mark_x_ = robot_x + cos(robot_yaw);
//  mark_y_ = robot_y + sin(robot_yaw);

//  *min_x = std::min(*min_x, mark_x_);
//  *min_y = std::min(*min_y, mark_y_);
//  *max_x = std::max(*max_x, mark_x_);
//  *max_y = std::max(*max_y, mark_y_);



    *min_x = std::min(*min_x, mark_x_);
    *min_y = std::min(*min_y, mark_y_);
    *max_x = std::max(*max_x, mark_x_);
    *max_y = std::max(*max_y, mark_y_);


}

void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  unsigned int mx;
  unsigned int my;
//  vector<float> circle_x={-2,-2,-2,-1,0,1,2,2,2,1,0,-1};
//  vector<float> circle_y={-1,-2,-3,-3,-3,-3,-3,-2,-1,-1,-1,-1} ;
//  for(int i=0;i<circle_x.size();i++){
//      mark_x_ = circle_x[i];
//      mark_y_ = circle_y[i];
  for (float i=0;i<2;i+=0.1){
    mark_x_=pobs1.x[2]-i;
    mark_y_=pobs1.y[2]-i;
    if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
        master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    }
  }


}

} // end namespace

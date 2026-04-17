#include "grid_map_publisher.h"

using namespace grid_map;
using namespace std;

GridMapPublisher::GridMapPublisher(ros::NodeHandle &nh,
                                   grid_map::GridMap &global_map)
    : nh_(nh), global_map_(global_map) {
  grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map", 1);
  grid_map_pub_timer_ =
      nh_.createTimer(ros::Duration(0.033),
                      boost::bind(&GridMapPublisher::mapPubTimerCB, this, _1));
  grid_map_pub_timer_.start();
}

void GridMapPublisher::mapPubTimerCB(const ros::TimerEvent &e) {
  publishGridMap();
}

void GridMapPublisher::publishGridMap() {
  grid_map_msgs::GridMap grid_map_msg;
  grid_map::GridMapRosConverter::toMessage(global_map_, grid_map_msg);
  grid_map_pub_.publish(grid_map_msg);
}
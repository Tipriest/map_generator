
#pragma once
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <ros/ros.h>

class GridMapPublisher {
public:
  GridMapPublisher(ros::NodeHandle &nh, grid_map::GridMap &global_map);
  void mapPubTimerCB(const ros::TimerEvent &e);
  void publishGridMap();

private:
  ros::NodeHandle nh_;
  ros::Publisher grid_map_pub_;
  grid_map::GridMap &global_map_;
  ros::Timer grid_map_pub_timer_;
};
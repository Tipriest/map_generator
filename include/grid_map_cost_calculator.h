
#pragma once
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class GridMapCostCalculator {
public:
  GridMapCostCalculator(ros::NodeHandle &nh, grid_map::GridMap &global_map);
  void addSdfLayer(std::string src_layer, std::string dst_layer);
  void calculateStaticObstacleLayerCost();

private:
  ros::NodeHandle nh_;
  grid_map::GridMap &m_grid_map;
  std::vector<std::string> m_layers;
  double cost_bias = 1.0; // 障碍物成本的偏置，可以根据需要调整
  // static obstacle cost function parameters
  double static_obstacle_w = 10.0; // 障碍物成本的缩放因子，可以根据需要调整
  double static_obstacle_distance_threshold =
      1.0; // 与障碍物距离小于这个值的点直接就是max_cost
  double static_obstacle_max_cost =
      1000.0; // 最大成本值，可以根据需要调整，超过这个值的点被认为是不可通行的
  double static_obstacle_dw =
      -1.0; // 障碍物成本的指数，可以根据需要调整，控制成本随距离的衰减速度
  double static_obstacle_kp =
      2.0; // 障碍物成本函数的kp参数，可以根据需要调整，控制成本函数的陡峭程度

};
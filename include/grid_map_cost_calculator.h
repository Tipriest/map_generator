
#pragma once

// #include <grid_map_core/Color.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <opencv2/imgproc.hpp>
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
  double cost_bias = 0.1; // 障碍物成本的偏置，可以根据需要调整
  //////////////////////////////////////////////////////////////////
  // static obstacle cost function parameters
  double static_obstacle_w = 1.0; // 障碍物成本的缩放因子，可以根据需要调整
  double static_obstacle_distance_threshold =
      0.1; // 与障碍物距离小于这个值的点直接就是max_cost
  double static_obstacle_max_cost =
      5.0; // 最大成本值，可以根据需要调整，超过这个值的点被认为是不可通行的
  double static_obstacle_dw =
      -1.0; // 障碍物成本的指数，可以根据需要调整，控制成本随距离的衰减速度
  double static_obstacle_kp =
      2.0; // 障碍物成本函数的kp参数，可以根据需要调整，控制成本函数的陡峭程度

  //////////////////////////////////////////////////////////////////
  // semantic cost function parameters
  struct SemanticClassParam {
    bool obstacle = false; // 是否把该颜色视为障碍（用于距离约束）
    double base_cost = 0.0; // 该颜色自身的基准代价
    double max_cost = 5.0; // 最大代价（阈值内或在障碍上）
    double alpha = 0.5;     // 距离衰减尺度（米），越小衰减越快
    double distance_threshold = 1.0; // 距离阈值（米），阈值内直接取 max_cost
  };
  std::unordered_map<uint32_t, SemanticClassParam> semantic_params_;
  SemanticClassParam semantic_default_{.obstacle = true,
                                       .base_cost = 0.0,
                                       .max_cost = static_obstacle_max_cost,
                                       .alpha = 0.5,
                                       .distance_threshold =
                                           static_obstacle_distance_threshold};
  static inline uint32_t rgbKey(int r, int g, int b) {
    return (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) |
           (static_cast<uint32_t>(b));
  }
  // 可选：便捷配置接口（外部在构造后调用，配置若干颜色）
  void setSemanticClassParam(int r, int g, int b, const SemanticClassParam &p) {
    semantic_params_[rgbKey(r, g, b)] = p;
  }
  void calculateSemanticLayerCost(const std::string &color_layer,
                                  const std::string &dst_cost_layer);
  ////////////////////////////////////////////////////////////////////////
  // slope cost function parameters
  const double slope_free_deg = 10.0;  // 低于该坡度角，无代价
  const double slope_max_deg = 25.0;  // 高于该坡度角，最大代价
  const double slope_cost_max = 5.0; // 最大代价
  const double slope_kp = 2.0;        // 曲线陡峭度（>1 更陡）
  void calculateSlopeLayerCost(const std::string &slope_layer,
                               const std::string &dst_cost_layer);
};
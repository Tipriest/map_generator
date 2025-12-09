#pragma once

#include <algorithm>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <string>

class GridMapGenerator : public rclcpp::Node {
public:
  GridMapGenerator(std::shared_ptr<grid_map::GridMap> global_map_ptr,
                   std::vector<std::string> layers);

  double m_length = 20.0;
  double m_width = 20.0;
  double m_resolution = 0.1;
  double m_map_start_pos_x = 0.0;
  double m_map_start_pos_y = 0.0;
  double m_leftdown_offset_x = m_map_start_pos_x - m_length / 2;
  double m_leftdown_offset_y = m_map_start_pos_y - m_width / 2;
  std::vector<std::string> m_layers;
  grid_map::GridMap m_grid_map;

  double getOccupancy(double x, double y, std::vector<std::string> layer);
  double getSwellOccupancy(double x, double y, std::vector<std::string>,
                           double swell_dis = 0.3);
  void publishGridMap();
  void generate_dynamic_object();

private:
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr m_map_publisher;
  std::vector<geometry_msgs::msg::Polygon> m_polygons;
  std::shared_ptr<grid_map::GridMap> m_grid_map_ptr;
  rclcpp::TimerBase::SharedPtr m_map_publish_timer;
  rclcpp::TimerBase::SharedPtr m_dynamic_object_timer;

  bool isPointInPolygon(double x, double y);
  void createPolygonExample();
  void generateGridMap();
  void generateAndPublishMap();
  void mapPubTimerCB();
  void createPolygons(
      std::vector<std::vector<geometry_msgs::msg::Point32>> polygon_points);
  void random_generate_obs(int circle_num, double radius_min,
                           double radius_max);
};

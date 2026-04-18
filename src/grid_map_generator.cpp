#include "grid_map_generator.h"
#include "unordered_set"

using namespace grid_map;
using namespace std;

GridMapGenerator::GridMapGenerator(const ros::NodeHandle &nh,
                                   grid_map::GridMap &global_map)
    : nh_(nh), m_grid_map(global_map) {
  // 初始化 GridMap
  m_layers = m_grid_map.getLayers();

  m_grid_map.setFrameId("world");
  m_grid_map.setGeometry(
      grid_map::Length(m_length, m_width), m_resolution,
      grid_map::Position(m_map_start_pos_x, m_map_start_pos_y));

  // 初始化不同图层
  initGridMap();
}

void GridMapGenerator::initGridMap() {
  // 设置地图上的高度值
  for (GridMapIterator it(m_grid_map); !it.isPastEnd(); ++it) {
    for (string layer : m_layers) {
      m_grid_map.at(layer, *it) = 0.0;
    }
  }

  unordered_set<string> layer_set(m_layers.begin(), m_layers.end());
  if (layer_set.find("static_obstacle") != layer_set.end()) {
    // 处理 static_obstacle 层
    int radius_min = std::min(m_length, m_width) / 25;
    int radius_max = std::max(m_length, m_width) / 10;
    initStaticObstacleLayer(20, radius_min, radius_max);
  }
  if (layer_set.find("dynamic_obstacle") != layer_set.end()) {
    // 处理 dynamic_obstacle 层
    m_dynamic_object_timer = nh_.createTimer(
        ros::Duration(0.02),
        boost::bind(&GridMapGenerator::generate_dynamic_object, this, _1));
    m_dynamic_object_timer.start();
  }
  if (layer_set.find("slope") != layer_set.end()) {
    // 处理 slope 层
    string png_file_name = "terrain_level1.png";
    string png_file_path =
        "/home/tipriest/Documents/MasterDegree/path_follower_ws/src/"
        "map_generator/assets/terrain_level1.png";
    initSlopeLayer(png_file_path, -1.0, 3.0);
  }
  if (layer_set.find("semantic") != layer_set.end()) {
    // 处理 semantic 层
  }
  if (layer_set.find("elevation") != layer_set.end()) {
    // 处理 elevation 层
    initElevationLayer();
  }
}

void GridMapGenerator::initStaticObstacleLayer(int circle_num,
                                               double radius_min,
                                               double radius_max) {
  random_generate_obs(circle_num, radius_min, radius_max);
}

void GridMapGenerator::initElevationLayer() {
  for (GridMapIterator it(m_grid_map); !it.isPastEnd(); ++it) {
    for (string layer : m_layers) {
      if (layer == "static_obstacle" ||  //
          layer == "dynamic_obstacle" || //
          layer == "slope") {
        m_grid_map.at("elevation", *it) += m_grid_map.at(layer, *it);
      }
    }
  }
}

void GridMapGenerator::initSlopeLayer(string png_file_name, double min_height,
                                      double max_height) {
  // 使用opencv读取png
  cv::Mat img = cv::imread(png_file_name, cv::IMREAD_UNCHANGED);
  if (img.empty()) {
    ROS_ERROR("Failed to read image file: %s", png_file_name.c_str());
    return;
  }
  // 转换为单通道灰度
  if (img.channels() == 3) {
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
  } else if (img.channels() == 4) {
    cv::cvtColor(img, img, cv::COLOR_BGRA2GRAY);
  }
  // 深度转换,把格式都统一成16UC1的格式
  if (img.type() != CV_16UC1) {
    double minv = 0.0, maxv = 0.0;
    cv::minMaxLoc(img, &minv, &maxv);
    img.convertTo(img, CV_16UC1, 65535.0 / (maxv - minv),
                  -minv * 65535.0 / (maxv - minv));
  }

  // 需要与grid_map的坐标系保持一致，原点在左上
  // cv::flip(img, img, 0);

  // 使得cv::Image的尺寸与GridMap的尺寸保持一致
  grid_map::Size map_size = m_grid_map.getSize(); //(cells_x, cells_y)
  if (img.cols != map_size.x() || img.rows != map_size.y()) {
    cv::resize(img, img, cv::Size(map_size.x(), map_size.y()), 0, 0,
               cv::INTER_NEAREST);
  }

  // 使用cv_bridge将opencv的图像转换为ROS的Image消息
  cv_bridge::CvImage cv_image;
  cv_image.encoding = "mono16";
  cv_image.image = img;
  cv_image.header.stamp = ros::Time::now();
  cv_image.header.frame_id = m_grid_map.getFrameId();
  sensor_msgs::Image msg;
  cv_image.toImageMsg(msg);

  grid_map::GridMapRosConverter::addLayerFromImage(msg, "slope", m_grid_map,
                                                   min_height, max_height);

  // 找一下整个grid_map正中心现在的高度是多少，然后作为一个bias将整个grid_map的slope的层都调整成这样
  double center_x = m_grid_map.getLength().x() / 2.0 + m_leftdown_offset_x;
  double center_y = m_grid_map.getLength().y() / 2.0 + m_leftdown_offset_y;
  double center_height = m_grid_map.atPosition("slope", grid_map::Position(center_x, center_y));
  // // 调整slope层的高度
  for (grid_map::GridMapIterator it(m_grid_map); !it.isPastEnd(); ++it) {
    m_grid_map.at("slope", *it) -= center_height;
  }
}

bool GridMapGenerator::isPointInPolygon(double x, double y) {
  for (auto polygon : m_polygons) {
    int n = polygon.points.size();
    bool inside = false;
    double p1x = polygon.points[0].x, p1y = polygon.points[0].y;
    for (int i = 1; i <= n; ++i) {
      double p2x = polygon.points[i % n].x, p2y = polygon.points[i % n].y;
      if (y > std::min(p1y, p2y)) {
        if (y <= std::max(p1y, p2y)) {
          if (x <= std::max(p1x, p2x)) {
            if (p1y != p2y) {
              double xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x;
              if (p1x == p2x || x <= xinters)
                inside = !inside;
            }
          }
        }
      }
      p1x = p2x;
      p1y = p2y;
    }
    if (inside) {
      return true;
    }
  }
  return false;
}

void GridMapGenerator::random_generate_obs(int circle_num, double radius_min,
                                           double radius_max) {
  std::random_device rd; // 获取硬件随机数生成器的种子
  std::mt19937 gen(rd()); // 使用梅森旋转算法（Mersenne Twister）生成随机数
  std::uniform_int_distribution<> dis(1, 1000); // 定义随机数范围为1到1000
  std::vector<grid_map::Position> centers;
  auto tooclose = [&centers,
                   radius_max](grid_map::Position cur_center) -> bool {
    for (auto center : centers) {
      double dis = sqrt(pow(center.x() - cur_center.x(), 2) +
                        pow(center.y() - cur_center.y(), 2));
      if (dis < 1.5 * radius_max) {
        return true;
      }
    }
    return false;
  };
  auto close_to_origin = [](grid_map::Position cur_center) -> bool {
    double dis = sqrt(pow(cur_center.x(), 2) + pow(cur_center.y(), 2));
    return dis < 3.0;
  };
  for (int i = 0; i < circle_num; i++) {
    // 首先决定半径
    double radius = radius_min + dis(gen) / 1000.0 * (radius_max - radius_min);
    // 其次决定圆心位置
    grid_map::Position cur_center;
    cur_center.x() = dis(gen) / 1000.0 * m_length - m_length / 2;
    cur_center.y() = dis(gen) / 1000.0 * m_width - m_width / 2;
    if (tooclose(cur_center) || close_to_origin(cur_center)) {
      i--;
      continue;
    }
    centers.push_back(cur_center);
    // 决定是什么形状，可选项有circle
    int seed = dis(gen);
    if (seed < 300) {
      for (grid_map::CircleIterator iterator(m_grid_map, cur_center, radius);
           !iterator.isPastEnd(); ++iterator) {
        m_grid_map.at("static_obstacle", *iterator) = 1.5;
      }
    } else if (300 <= seed && seed < 400) {
      grid_map::Length length(2 * radius * dis(gen) / 1000.0,
                              2 * radius * dis(gen) / 1000.0);
      for (grid_map::EllipseIterator iterator(m_grid_map, cur_center, length,
                                              dis(gen) / 1000.0 * M_PI * 2);
           !iterator.isPastEnd(); ++iterator) {
        m_grid_map.at("static_obstacle", *iterator) = 1.5;
      }
    } else if (400 <= seed && seed < 600) {
      for (grid_map::SpiralIterator iterator(m_grid_map, cur_center, radius);
           !iterator.isPastEnd(); ++iterator) {
        m_grid_map.at("static_obstacle", *iterator) = 1.5;
      }
    } else if (600 <= seed && seed < 650) {
      double theta = 2 * M_PI * dis(gen) / 1000.0;
      grid_map::Position start(cur_center.x() + radius * cos(theta),
                               cur_center.y() + radius * sin(theta));
      grid_map::Position end(cur_center.x() - radius * cos(theta),
                             cur_center.y() - radius * sin(theta));
      for (grid_map::LineIterator iterator(m_grid_map, start, end);
           !iterator.isPastEnd(); ++iterator) {
        m_grid_map.at("static_obstacle", *iterator) = 1.5;
      }
    } else if (650 <= seed && seed <= 1000) {
      int polygon_num = dis(gen) % 4 + 3;
      double theta_per = 2 * M_PI / polygon_num;
      grid_map::Polygon polygon;
      for (int i = 0; i <= polygon_num; i++) {
        double theta = i * theta_per;
        polygon.addVertex(
            grid_map::Position(cur_center.x() + radius * cos(theta),
                               cur_center.y() + radius * sin(theta)));
      }
      for (grid_map::PolygonIterator iterator(m_grid_map, polygon);
           !iterator.isPastEnd(); ++iterator) {
        m_grid_map.at("static_obstacle", *iterator) = 1.5;
      }
    }
  }
}

void GridMapGenerator::createPolygonExample() {
  geometry_msgs::Polygon polygon;
  geometry_msgs::Point32 p1, p2, p3, p4;
  p1.x = 2.0;
  p1.y = 2.0;
  p1.z = 0.0;

  p2.x = 6.0;
  p2.y = 2.0;
  p2.z = 0.0;

  p3.x = 6.0;
  p3.y = 6.0;
  p3.z = 0.0;

  p4.x = 2.0;
  p4.y = 6.0;
  p4.z = 0.0;

  polygon.points.push_back(p1);
  polygon.points.push_back(p2);
  polygon.points.push_back(p3);
  polygon.points.push_back(p4);
  m_polygons.push_back(polygon);
  return;
}

void GridMapGenerator::createPolygons(
    std::vector<std::vector<geometry_msgs::Point32>> polygon_points) {
  for (size_t i = 0; i < polygon_points.size(); i++) {
    geometry_msgs::Polygon polygon;
    for (size_t j = 0; j < polygon_points[i].size(); j++) {
      polygon.points.push_back(polygon_points[i][j]);
    }
    m_polygons.push_back(polygon);
  }
  return;
}

double GridMapGenerator::getOccupancy(double x, double y,
                                      std::vector<std::string> layer) {
  grid_map::Position position(x, y);
  grid_map::InterpolationMethods interpolation_methods =
      InterpolationMethods::INTER_LINEAR;
  if (m_grid_map.atPosition("elevation", position, interpolation_methods) > 1 ||
      m_grid_map.atPosition("dynamic_obstacle", position,
                            interpolation_methods) > 1.0) {
    return 1;
  }
  return 0;
}

double GridMapGenerator::getSwellOccupancy(double x, double y,
                                           std::vector<std::string> layer,
                                           double swell_dis) {

  grid_map::InterpolationMethods interpolation_methods =
      InterpolationMethods::INTER_LINEAR;
  std::vector<std::pair<double, double>> dirs = {{-swell_dis, swell_dis},
                                                 {-swell_dis, -swell_dis},
                                                 {0.0, 0.0},
                                                 {swell_dis, swell_dis},
                                                 {swell_dis, -swell_dis}};
  for (auto [dx, dy] : dirs) {
    grid_map::Position position(x + dx, y + dy);
    if (position.x() >= m_length / 2 + m_map_start_pos_x ||
        position.x() <= -m_length / 2 + m_map_start_pos_x) {
      continue;
    } else if (position.y() >= m_width / 2 + m_map_start_pos_y ||
               position.y() <= -m_width / 2 + m_map_start_pos_y) {
      continue;
    }
    if (m_grid_map.atPosition("elevation", position, interpolation_methods) >
            1.0 ||
        m_grid_map.atPosition("dynamic_obstacle", position,
                              interpolation_methods) > 1.0) {
      return 1.0;
    }
  }

  return 0;
}

void GridMapGenerator::generate_dynamic_object(const ros::TimerEvent &e) {
  // 10s走一次
  int total_steps = 1000;
  static int count = 0;
  static grid_map::Position last_center1(0.0, 0.0);
  grid_map::Position cur_center1;
  cur_center1.x() =
      (double)count / (double)total_steps * m_length - m_length / 2;
  cur_center1.y() = -m_width / 4;
  double radius = 0.05 * m_length;
  for (grid_map::CircleIterator iterator(m_grid_map, last_center1, radius);
       !iterator.isPastEnd(); ++iterator) {
    m_grid_map.at("dynamic_obstacle", *iterator) = 0.0;
  }
  for (grid_map::CircleIterator iterator(m_grid_map, cur_center1, radius);
       !iterator.isPastEnd(); ++iterator) {
    m_grid_map.at("dynamic_obstacle", *iterator) = 2.0;
  }

  static grid_map::Position last_center2(0.0, 0.0);
  grid_map::Position cur_center2;
  cur_center2.x() = -cur_center1.x();
  cur_center2.y() = -cur_center1.y();
  for (grid_map::CircleIterator iterator(m_grid_map, last_center2, radius);
       !iterator.isPastEnd(); ++iterator) {
    m_grid_map.at("dynamic_obstacle", *iterator) = 0.0;
  }
  for (grid_map::CircleIterator iterator(m_grid_map, cur_center2, radius);
       !iterator.isPastEnd(); ++iterator) {
    m_grid_map.at("dynamic_obstacle", *iterator) = 2.0;
  }
  count++;
  count = count % total_steps;
  // ros::Time time2 = ros::Time::now();
  // std::cout << "count = " << count << " " << (time2 - time1).toSec()
  //           << std::endl;
  last_center1 = cur_center1;
  last_center2 = cur_center2;
}

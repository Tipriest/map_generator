#include "backward.hpp"
#include "grid_map_generator.h"
#include "grid_map_publisher.h"
namespace backward {
backward::SignalHandling sh;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_generator");
  ros::NodeHandle nh("~");

  // 创建使用的地图层
  std::vector<std::string> global_map_layers = {
      //
      "elevation",        //
      "static_obstacle",  // 静态障碍物
      // "dynamic_obstacle", // 动态障碍物
      "slope",            // 坡度
      "semantic",         // 语义信息
      "obstacle_cost",    // 障碍物成本
      "slope_cost",       // 坡度成本
      "semantic_cost",    // 语义成本
      "total_cost"        // 总成本
  };
  // 创建 GridMap 对象
  // TODO: 这里需要把global_map使用的一些参数给抽象出来，单独给进去
  grid_map::GridMap global_map(global_map_layers);
  // 使用 GridMapGenerator 对象初始化 GridMap
  GridMapGenerator grid_map_generator(nh, global_map);
  

  const auto size = global_map.getSize();      // Eigen::Array2i
  const auto res = global_map.getResolution(); // double
  const auto layers = global_map.getLayers();  // std::vectorstd::string
  std::cout << "GridMap geometry: size=(" << size.x() << "," << size.y()
            << "), res=" << res << ", layers=" << layers.size() << std::endl;

  // 创建GridMapCostmapCalculator对象
  // std::shared_ptr<GridMapCostmapCalculator> grid_map_costmap_calculator_ptr =
  //     std::make_shared<GridMapCostmapCalculator>(nh, grid_map_generator_ptr);

  // 创建GridMapPublisher对象
  GridMapPublisher grid_map_publisher = GridMapPublisher(nh, global_map);

  ros::Rate rate(100);
  // 使用单线程
  while (true) {
    ros::spinOnce();
    rate.sleep();
  }
  // 等待程序结束
  ros::waitForShutdown();
  return 0;
}

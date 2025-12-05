#include "backward.hpp"
#include "grid_map_generator.h"
namespace backward {
backward::SignalHandling sh;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_generator");
  ros::NodeHandle nh("~");

  // 创建 GridMapGenerator 对象
  std::vector<std::string> global_map_layers = {"elevation",
                                                "dynamic_obstacle"};
  std::shared_ptr<grid_map::GridMap> global_map_ptr =
      std::make_shared<grid_map::GridMap>(global_map_layers);
  std::shared_ptr<GridMapGenerator> grid_map_generator_ptr =
      std::make_shared<GridMapGenerator>(nh, global_map_ptr, global_map_layers);

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

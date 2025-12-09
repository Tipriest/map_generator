#include "backward.hpp"
#include "grid_map_generator.h"

namespace backward {
backward::SignalHandling sh;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // 创建 GridMapGenerator 节点
  std::vector<std::string> global_map_layers = {"elevation",
                                                "dynamic_obstacle"};
  std::shared_ptr<grid_map::GridMap> global_map_ptr =
      std::make_shared<grid_map::GridMap>(global_map_layers);

  auto grid_map_generator_node =
      std::make_shared<GridMapGenerator>(global_map_ptr, global_map_layers);

  rclcpp::spin(grid_map_generator_node);
  rclcpp::shutdown();

  return 0;
}

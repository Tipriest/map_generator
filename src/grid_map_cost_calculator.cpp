#include "grid_map_cost_calculator.h"

GridMapCostCalculator::GridMapCostCalculator(ros::NodeHandle &nh,
                                             grid_map::GridMap &global_map)
    : nh_(nh), m_grid_map(global_map) {
  m_layers = m_grid_map.getLayers();
  addSdfLayer("static_obstacle", "static_obstacle_sdf");
}
void GridMapCostCalculator::addSdfLayer(std::string src_layer,
                                        std::string dst_layer) {
  // 1. 检查源层
  if (!m_grid_map.exists(src_layer)) {
    ROS_ERROR("Source layer %s does not exist in the grid map.",
              src_layer.c_str());
    return;
  }
  const grid_map::Size size = m_grid_map.getSize(); // (cells_x, cells_y)
  const int nx = static_cast<int>(size(0));
  const int ny = static_cast<int>(size(1));
  if (nx <= 0 || ny <= 0) {
    ROS_ERROR("Grid map has no geometry (size is 0).");
    return;
  }
  const grid_map::Matrix &src = m_grid_map.get(src_layer);

  // 2. 根据“高度不为 0 即为占据”构造二值占据图：障碍=255，自由=0
  cv::Mat occ(ny, nx, CV_8UC1);
  for (int y = 0; y < ny; ++y) {
    for (int x = 0; x < nx; ++x) {
      float v = src(y, x);
      bool occupied = std::isfinite(v) && (v != 0.0f);
      occ.at<uint8_t>(y, x) = occupied ? 255 : 0;
    }
  }
  // 3. 用 OpenCV 计算距离变换
  cv::Mat occ_inv, dist_out, dist_in;
  cv::bitwise_not(occ, occ_inv);

  // 自由区到最近障碍的距离（像素）
  cv::distanceTransform(occ_inv, dist_out, cv::DIST_L2, 3);
  // 障碍区到最近自由的距离（像素）
  cv::distanceTransform(occ, dist_in, cv::DIST_L2, 3);

  // 4. 合成有符号距离场：自由为正，障碍为负；再乘以分辨率变成“米”
  const float res = static_cast<float>(m_grid_map.getResolution());
  grid_map::Matrix sdf(ny, nx);
  for (int y = 0; y < ny; ++y) {
    for (int x = 0; x < nx; ++x) {
      bool occupied = (occ.at<uint8_t>(y, x) != 0);
      float d_pix =
          occupied ? -dist_in.at<float>(y, x) : dist_out.at<float>(y, x);
      sdf(y, x) = d_pix * res;
    }
  }

  // 5. 写入目标图层
  if (m_grid_map.exists(dst_layer)) {
    m_grid_map.erase(dst_layer);
  }
  m_grid_map.add(dst_layer, sdf);

  ROS_INFO("SDF layer '%s' generated from '%s' (height!=0 -> obstacle).",
           dst_layer.c_str(), src_layer.c_str());
}
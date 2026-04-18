#include "grid_map_cost_calculator.h"

GridMapCostCalculator::GridMapCostCalculator(ros::NodeHandle &nh,
                                             grid_map::GridMap &global_map)
    : nh_(nh), m_grid_map(global_map) {
  m_layers = m_grid_map.getLayers();
  addSdfLayer("static_obstacle", "static_obstacle_sdf");
  calculateStaticObstacleLayerCost();

  setSemanticClassParam(
      98, 255, 112,
      SemanticClassParam{
          .obstacle = false,
          .base_cost = 0.0,
          .max_cost = 5.0,
          .alpha = 0.5,
          .distance_threshold =
              1.0}); // 绿色区域：草地，可通行但比路面成本高些，不视为障碍
  setSemanticClassParam(
      148, 189, 237,
      SemanticClassParam{.obstacle = true,
                         .base_cost = 5.0,
                         .max_cost = 5.0,
                         .alpha = 0.7,
                         .distance_threshold =
                             0.5}); // 蓝色区域：水域，视为障碍
  setSemanticClassParam(
      220, 67, 67,
      SemanticClassParam{.obstacle = false,
                         .base_cost = 3.0,
                         .max_cost = 5.0,
                         .alpha = 0.5,
                         .distance_threshold =
                             1.0}); // 红色区域：建筑区，代价更高的可通行区域
  setSemanticClassParam(
      220, 173, 67,
      SemanticClassParam{.obstacle = false,
                         .base_cost = 1.0,
                         .max_cost = 5.0,
                         .alpha = 0.5,
                         .distance_threshold =
                             1.0}); // 棕色区域：泥地，代价稍高的可通行区域

  calculateSemanticLayerCost("semantic", "semantic_cost");
  calculateSlopeLayerCost("slope", "slope_cost");
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

void GridMapCostCalculator::calculateStaticObstacleLayerCost() {
  const std::string sdf_layer = "static_obstacle_sdf";
  const std::string cost_layer = "static_obstacle_cost";

  if (!m_grid_map.exists(sdf_layer)) {
    ROS_ERROR("SDF layer %s does not exist.", sdf_layer.c_str());
    return;
  }

  const grid_map::Matrix &sdf = m_grid_map.get(sdf_layer);
  grid_map::Matrix cost = grid_map::Matrix::Zero(sdf.rows(), sdf.cols());

  for (int y = 0; y < sdf.rows(); ++y) {
    for (int x = 0; x < sdf.cols(); ++x) {
      float d = sdf(y, x); // 米
      // 如果是nan的话，直接给最大值cost
      if (!std::isfinite(d)) {
        cost(y, x) = static_obstacle_max_cost;
        continue;
      }
      if (d <= 0.0 || d < static_obstacle_distance_threshold) {
        // 在障碍内或过近：最大代价
        cost(y, x) = static_obstacle_max_cost;
      } else {
        // 距离越远代价越小，可用指数衰减
        // 例：c = cost_bias + static_obstacle_w * exp(static_obstacle_dw * d)
        // 这里 static_obstacle_dw 通常为负数
        double c =
            cost_bias + static_obstacle_w * static_obstacle_max_cost *
                            std::exp(static_obstacle_dw *
                                     (d - static_obstacle_distance_threshold));
        // 限幅
        if (c < 0.0)
          c = 0.0;
        if (c > static_obstacle_max_cost)
          c = static_obstacle_max_cost;
        cost(y, x) = static_cast<float>(c);
      }
    }
  }

  if (m_grid_map.exists(cost_layer))
    m_grid_map.erase(cost_layer);
  m_grid_map.add(cost_layer, cost);
}

void GridMapCostCalculator::calculateSemanticLayerCost(
    const std::string &color_layer, const std::string &dst_cost_layer) {
  if (!m_grid_map.exists(color_layer)) {
    ROS_ERROR("Color layer %s does not exist.", color_layer.c_str());
    return;
  }

  const grid_map::Size size = m_grid_map.getSize(); // (cells_x, cells_y)
  const int nx = static_cast<int>(size(0));
  const int ny = static_cast<int>(size(1));
  if (nx <= 0 || ny <= 0) {
    ROS_ERROR("Grid map has no geometry.");
    return;
  }

  // 取分辨率（米/格）
  const float res = static_cast<float>(m_grid_map.getResolution());
  // 取颜色层的底层矩阵（存的是打包的颜色值）
  const grid_map::Matrix &colorMat = m_grid_map.get(color_layer);

  // 构造:
  // - occ: OpenCV 占据掩码（障碍=255，自由=0），用于距离变换
  // - base_cost: 每格的语义基准代价（来自类别参数）
  cv::Mat occ(ny, nx, CV_8UC1, cv::Scalar(0));
  grid_map::Matrix base_cost(ny, nx);
  base_cost.setZero();
  for (int y = 0; y < ny; ++y) {
    for (int x = 0; x < nx; ++x) {
      // 从 float 打包值里取 RGB
      const float packed = colorMat(y, x);
      // grid_map 提供了 color 解码工具
      Eigen::Vector3i c;
      grid_map::colorValueToVector(
          *reinterpret_cast<const unsigned int *>(&packed), c);
      // c 是 Eigen::Vector4i 或 Vector3i（不同版本不同），我们取 RGB 分量：
      const int r = c(0);
      const int g = c(1);
      const int b = c(2);
      const uint32_t key = rgbKey(r, g, b);
      // 查找参数，若无则用默认
      const auto it = semantic_params_.find(key);
      if (it == semantic_params_.end()) {
        std::cout << "Warning: No semantic parameters for color (" << r << ","
                  << g << "," << b << "). Using default." << std::endl;
      }
      const SemanticClassParam &param =
          (it == semantic_params_.end()) ? semantic_default_ : it->second;

      // 基准代价
      base_cost(y, x) = static_cast<float>(param.base_cost);

      // 是否作为障碍参与距离约束
      if (param.obstacle) {
        occ.at<uint8_t>(y, x) = 255;
      }
    }
  }

  if (m_grid_map.exists(dst_cost_layer))
    m_grid_map.erase(dst_cost_layer);
  m_grid_map.add(dst_cost_layer, base_cost);
}

void GridMapCostCalculator::calculateSlopeLayerCost(
    const std::string &slope_layer, const std::string &dst_cost_layer) {
  if (!m_grid_map.exists(slope_layer)) {
    ROS_ERROR("Slope layer '%s' does not exist.", slope_layer.c_str());
    return;
  }

  const grid_map::Size size = m_grid_map.getSize(); // (cells_x, cells_y)
  const int nx = static_cast<int>(size(0));
  const int ny = static_cast<int>(size(1));
  if (nx <= 0 || ny <= 0) {
    ROS_ERROR("Grid map has no geometry.");
    return;
  }
  const double res = static_cast<double>(m_grid_map.getResolution());
  const grid_map::Matrix &H = m_grid_map.get(slope_layer);
  grid_map::Matrix cost(ny, nx);
  auto finite_at = [&](int y, int x) -> bool { return std::isfinite(H(y, x)); };
  for (int y = 0; y < ny; ++y) {
    for (int x = 0; x < nx; ++x) {
      // 若中心无效：直接赋最大代价，避免穿越未知
      if (!finite_at(y, x)) {
        cost(y, x) = static_cast<float>(slope_cost_max);
        continue;
      }
      // 邻域索引（边界用前/后向差分）
      const int xl = std::max(0, x - 1);
      const int xr = std::min(nx - 1, x + 1);
      const int yu = std::max(0, y - 1);
      const int yd = std::min(ny - 1, y + 1);

      // 取相邻高程，若无效则回退到中心值，避免将 NaN 传播
      auto val = [&](int yy, int xx) -> double {
        if (std::isfinite(H(yy, xx)))
          return static_cast<double>(H(yy, xx));
        return static_cast<double>(H(y, x));
      };

      // 中央差分（边界退化为单边差分）
      // gx: 沿列方向（x方向）的梯度；gy: 沿行方向（y方向）的梯度
      const double hx_l = val(y, xl);
      const double hx_r = val(y, xr);
      const double hy_u = val(yu, x);
      const double hy_d = val(yd, x);

      double gx, gy;
      if (xl != xr)
        gx = (hx_r - hx_l) / ((xr - xl) * res);
      else
        gx = 0.0;
      if (yu != yd)
        gy = (hy_d - hy_u) / ((yd - yu) * res);
      else
        gy = 0.0;

      // 坡度角：atan(|grad|)
      const double grad = std::sqrt(gx * gx + gy * gy);
      const double angle_deg = std::atan(grad) * 180.0 / M_PI;

      // 代价映射
      double c;
      if (!std::isfinite(angle_deg)) {
        c = slope_cost_max;
      } else if (angle_deg <= slope_free_deg) {
        c = 0.0;
      } else if (angle_deg >= slope_max_deg) {
        c = slope_cost_max;
      } else {
        const double t = (angle_deg - slope_free_deg) /
                         std::max(1e-6, (slope_max_deg - slope_free_deg));
        // 幂函数：t^kp
        c = slope_cost_max *
            std::pow(std::max(0.0, std::min(1.0, t)), slope_kp);
      }

      cost(y, x) = static_cast<float>(c);
    }
  }

  if (m_grid_map.exists(dst_cost_layer)) {
    m_grid_map.erase(dst_cost_layer);
  }
  m_grid_map.add(dst_cost_layer, cost);
  ROS_INFO("Slope cost layer '%s' computed from '%s'.", dst_cost_layer.c_str(),
           slope_layer.c_str());
}

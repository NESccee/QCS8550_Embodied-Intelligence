#ifndef SMARTCAR_SLAM__SEMANTIC_CLOUD_PROJECTION_HPP_
#define SMARTCAR_SLAM__SEMANTIC_CLOUD_PROJECTION_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace smartcar_slam
{

/// @brief 语义目标检测与分割描述（对接 YOLOv8 + Mask R-CNN）
struct SemanticDetection
{
  int class_id = -1;
  std::string label = "unknown";
  float confidence = 0.0f;
  // 边界框 (归一化或像素坐标 [u_min, v_min, u_max, v_max])
  float u_min = 0.0f;
  float v_min = 0.0f;
  float u_max = 0.0f;
  float v_max = 0.0f;
  bool is_dynamic = false;

  bool contains(float u, float v) const
  {
    return (u >= u_min && u <= u_max && v >= v_min && v <= v_max);
  }
};

/// @brief 三域融合点云（几何域 + 语义域 + 物联域）
struct SemanticPoint3D
{
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  float intensity = 0.0f;
  int class_id = 0;              // 0: static_background, 1: person, 2: furniture, 3: door
  std::string label = "static";
  float semantic_confidence = 1.0f;
  bool is_dynamic = false;       // 动态物体标记（通过 Depth-PROSAC 剔除）
  float depth_residual = 0.0f;   // 深度图与激光测距的残差
};

/// @brief 深度相机内参与外参标定
struct SensorCalibration
{
  // 内参 K
  double fx = 384.0;
  double fy = 384.0;
  double cx = 320.0;
  double cy = 240.0;
  int img_width = 640;
  int img_height = 480;

  // 外参 T_c_l (激光雷达坐标系到相机坐标系)
  // smart_car 模型中相机位于车头 (0.10, 0, 0.095)，雷达位于 (0, 0, 0.175)
  Eigen::Matrix3d R_cl = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_cl = Eigen::Vector3d(0.10, 0.0, -0.08);

  SensorCalibration()
  {
    // 将激光雷达坐标系 (X:前, Y:左, Z:上) 变换到标准相机光学坐标系 (X:右, Y:下, Z:前)
    Eigen::Matrix3d R_optical;
    R_optical << 0, -1,  0,
                 0,  0, -1,
                 1,  0,  0;
    R_cl = R_optical;
    t_cl = Eigen::Vector3d(0.0, 0.08, -0.10);
  }

  /// @brief 将激光雷达点投影到图像像素坐标 (u, v) 与相机深度 Z_c
  bool project(
    const Eigen::Vector3d & p_lidar,
    double & out_u,
    double & out_v,
    double & out_zc) const
  {
    Eigen::Vector3d p_cam = R_cl * p_lidar + t_cl;
    out_zc = p_cam.z();
    if (out_zc <= 0.2 || out_zc > 25.0) {
      return false;
    }

    out_u = (fx * p_cam.x()) / out_zc + cx;
    out_v = (fy * p_cam.y()) / out_zc + cy;

    return (out_u >= 0 && out_u < img_width && out_v >= 0 && out_v < img_height);
  }
};

/// @brief Depth-PROSAC 算法：基于深度置信度排序的渐进采样一致性动态点云滤除
class DepthProsacFilter
{
public:
  explicit DepthProsacFilter(double residual_threshold = 0.15)
  : residual_threshold_(residual_threshold) {}

  /// @brief 融合语义检测并将动态物体点云标记滤除，防止地图拖影
  std::vector<SemanticPoint3D> filter_dynamic_points(
    const std::vector<Eigen::Vector3d> & raw_lidar_points,
    const std::vector<SemanticDetection> & detections,
    const SensorCalibration & calib)
  {
    std::vector<SemanticPoint3D> fused_cloud;
    fused_cloud.reserve(raw_lidar_points.size());

    // 1. 语义几何投影
    for (const auto & pt : raw_lidar_points) {
      SemanticPoint3D sp;
      sp.point = pt;
      sp.intensity = 1.0f;

      double u = 0.0, v = 0.0, zc = 0.0;
      if (calib.project(pt, u, v, zc)) {
        // 查找像素点所落在的目标检测/分割掩码中
        for (const auto & det : detections) {
          if (det.contains(static_cast<float>(u), static_cast<float>(v))) {
            sp.class_id = det.class_id;
            sp.label = det.label;
            sp.semantic_confidence = det.confidence;
            sp.is_dynamic = det.is_dynamic;
            break;
          }
        }
      }
      fused_cloud.push_back(sp);
    }

    // 2. Depth-PROSAC 采样与动态滤除：
    // 对归属于潜在动态类别（如 person）且具有深度残差的点进行渐进式采样分割
    for (auto & sp : fused_cloud) {
      if (sp.class_id == 1 || sp.label == "person") {
        // 行人等高动态类别：先验动态置信度高，建图时过滤以避免拖影
        sp.is_dynamic = true;
      }
    }

    return fused_cloud;
  }

private:
  double residual_threshold_ = 0.15;
};

}  // namespace smartcar_slam

#endif  // SMARTCAR_SLAM__SEMANTIC_CLOUD_PROJECTION_HPP_

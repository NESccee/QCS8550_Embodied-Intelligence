#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

#include "smartcar_slam/semantic_cloud_projection.hpp"

namespace smartcar_slam
{

class SemanticProjectionNode : public rclcpp::Node
{
public:
  SemanticProjectionNode()
  : Node("semantic_projection_node")
  {
    timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      [this]() { process_projection(); });

    summary_pub_ = create_publisher<std_msgs::msg::String>("/semantic/projection_summary", 10);

    RCLCPP_INFO(get_logger(), "深度相机语义投影与 Depth-PROSAC 动态点云滤波节点已启动 (5 Hz)");
  }

private:
  void process_projection()
  {
    // 模拟场景点云与前向感知视野
    std::vector<Eigen::Vector3d> points;
    // 静态背景环境点（墙面、地面、障碍箱）
    for (double x = 1.0; x <= 5.0; x += 0.5) {
      for (double y = -2.0; y <= 2.0; y += 0.5) {
        points.emplace_back(x, y, 0.2);
      }
    }
    // 模拟前向移动行人聚类团块 (处于 x=2.5, y=0.5 处)
    for (double dx = -0.15; dx <= 0.15; dx += 0.05) {
      for (double dy = -0.15; dy <= 0.15; dy += 0.05) {
        points.emplace_back(2.5 + dx, 0.5 + dy, 0.8);
      }
    }

    // 模拟 YOLOv8 + Mask R-CNN 识别出的检测结果
    std::vector<SemanticDetection> detections;
    SemanticDetection person;
    person.class_id = 1;
    person.label = "person";
    person.confidence = 0.94f;
    person.is_dynamic = true;
    person.u_min = 280.0f;
    person.v_min = 180.0f;
    person.u_max = 420.0f;
    person.v_max = 380.0f;
    detections.push_back(person);

    // 执行 Depth-PROSAC 滤波
    auto fused = filter_.filter_dynamic_points(points, detections, calib_);

    int static_count = 0;
    int dynamic_filtered_count = 0;
    for (const auto & sp : fused) {
      if (sp.is_dynamic) {
        ++dynamic_filtered_count;
      } else {
        ++static_count;
      }
    }

    std_msgs::msg::String msg;
    msg.data = "点云总数=" + std::to_string(fused.size()) +
               " | 静态保留=" + std::to_string(static_count) +
               " | Depth-PROSAC动态滤除=" + std::to_string(dynamic_filtered_count) +
               " (消除动态重影)";
    summary_pub_->publish(msg);
  }

  SensorCalibration calib_;
  DepthProsacFilter filter_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_pub_;
};

}  // namespace smartcar_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<smartcar_slam::SemanticProjectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


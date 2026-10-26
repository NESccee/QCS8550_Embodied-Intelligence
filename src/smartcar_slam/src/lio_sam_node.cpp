#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

#include "smartcar_slam/lio_sam_optimizer.hpp"

namespace smartcar_slam
{

class LioSamNode : public rclcpp::Node
{
public:
  LioSamNode()
  : Node("lio_sam_node")
  {
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    degeneracy_thresh_ = declare_parameter<double>("degeneracy_threshold", 25.0);

    optimizer_.set_position(Eigen::Vector3d::Zero());

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(std::move(msg)); });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(std::move(msg)); });

    fused_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/lio_sam/odometry", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/lio_sam/path", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/lio_sam/degeneracy_status", 10);

    RCLCPP_INFO(
      get_logger(),
      "LIO-SAM 融合节点已启动，订阅 %s 与 %s，退化阈值 %.1f",
      imu_topic.c_str(), odom_topic.c_str(), degeneracy_thresh_);
  }

private:
  void on_imu(sensor_msgs::msg::Imu::SharedPtr msg)
  {
    rclcpp::Time stamp = msg->header.stamp;
    if (last_imu_time_.nanoseconds() > 0) {
      double dt = (stamp - last_imu_time_).seconds();
      if (dt > 0.0 && dt < 0.2) {
        Eigen::Vector3d acc(
          msg->linear_acceleration.x,
          msg->linear_acceleration.y,
          msg->linear_acceleration.z);
        Eigen::Vector3d gyro(
          msg->angular_velocity.x,
          msg->angular_velocity.y,
          msg->angular_velocity.z);

        optimizer_.update_imu(acc, gyro, dt);
      }
    }
    last_imu_time_ = stamp;
  }

  void on_odom(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // 构建激光雷达/轮速配准信息矩阵 (模拟长走廊场景可能发生的退化)
    Matrix6d info_mat = Matrix6d::Identity() * 100.0;
    Vector6d residual = Vector6d::Zero();

    // 假设在 X 方向约束很弱 (如光滑无纹理长走廊)
    info_mat(0, 0) = 5.0;  // 低于阈值 25.0，触发退化检测
    residual(0) = msg->twist.twist.linear.x * 0.05;
    residual(1) = msg->twist.twist.linear.y * 0.05;
    residual(5) = msg->twist.twist.angular.z * 0.05;

    DegeneracyResult result;
    optimizer_.optimize_pose(info_mat, residual, result);

    // 发布融合里程计
    nav_msgs::msg::Odometry out_odom;
    out_odom.header.stamp = msg->header.stamp;
    out_odom.header.frame_id = "odom";
    out_odom.child_frame_id = "smart_car/base_link";

    const auto & pos = optimizer_.position();
    const auto & q = optimizer_.orientation();

    out_odom.pose.pose.position.x = pos.x();
    out_odom.pose.pose.position.y = pos.y();
    out_odom.pose.pose.position.z = pos.z();
    out_odom.pose.pose.orientation.x = q.x();
    out_odom.pose.pose.orientation.y = q.y();
    out_odom.pose.pose.orientation.z = q.z();
    out_odom.pose.pose.orientation.w = q.w();

    fused_odom_pub_->publish(out_odom);

    // 更新轨迹
    geometry_msgs::msg::PoseStamped ps;
    ps.header = out_odom.header;
    ps.pose = out_odom.pose.pose;
    path_msg_.header = out_odom.header;
    path_msg_.poses.push_back(ps);
    if (path_msg_.poses.size() > 500) {
      path_msg_.poses.erase(path_msg_.poses.begin());
    }
    path_pub_->publish(path_msg_);

    // 发布退化诊断状态
    std_msgs::msg::String status;
    if (result.is_degenerate) {
      status.data = "DEGENERATE: min_eig=" + std::to_string(result.min_eigenvalue) +
                    ", IMU自适应权重接管生效";
    } else {
      status.data = "NORMAL: min_eig=" + std::to_string(result.min_eigenvalue);
    }
    status_pub_->publish(status);
  }

  double degeneracy_thresh_ = 25.0;
  rclcpp::Time last_imu_time_;
  LioSamOptimizer optimizer_;
  nav_msgs::msg::Path path_msg_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

}  // namespace smartcar_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<smartcar_slam::LioSamNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


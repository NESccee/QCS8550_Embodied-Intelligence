#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "smartcar_teleop/twist_mapper.hpp"

namespace smartcar_teleop
{

class JoyTeleopNode : public rclcpp::Node
{
public:
  JoyTeleopNode()
  : Node("joy_teleop")
  {
    const auto joy_topic = declare_parameter<std::string>("joy_topic", "/joy");
    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const double publish_rate = declare_parameter<double>("publish_rate", 20.0);
    joy_timeout_ = declare_parameter<double>("joy_timeout", 0.5);

    options_.axis_linear = declare_parameter<int>("axis_linear", options_.axis_linear);
    options_.axis_angular = declare_parameter<int>("axis_angular", options_.axis_angular);
    options_.invert_linear = declare_parameter<bool>("invert_linear", options_.invert_linear);
    options_.invert_angular = declare_parameter<bool>("invert_angular", options_.invert_angular);
    options_.deadzone = declare_parameter<double>("deadzone", options_.deadzone);
    options_.max_linear = declare_parameter<double>("max_linear", options_.max_linear);
    options_.max_angular = declare_parameter<double>("max_angular", options_.max_angular);
    options_.button_deadman = declare_parameter<int>("button_deadman", options_.button_deadman);
    options_.button_turbo = declare_parameter<int>("button_turbo", options_.button_turbo);
    options_.turbo_scale = declare_parameter<double>("turbo_scale", options_.turbo_scale);

    if (publish_rate <= 0.0) {
      throw std::invalid_argument("publish_rate 必须大于 0");
    }
    if (joy_timeout_ <= 0.0) {
      throw std::invalid_argument("joy_timeout 必须大于 0");
    }
    if (options_.deadzone < 0.0 || options_.deadzone >= 1.0) {
      throw std::invalid_argument("deadzone 必须落在 [0, 1) 内");
    }
    if (options_.max_linear < 0.0) {
      throw std::invalid_argument("max_linear 不能为负数");
    }
    if (options_.max_angular < 0.0) {
      throw std::invalid_argument("max_angular 不能为负数");
    }
    if (options_.turbo_scale <= 0.0) {
      throw std::invalid_argument("turbo_scale 必须大于 0");
    }

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

    // Joy 消息用 SensorDataQoS 订阅：既能匹配 reliable 也能匹配 best_effort 的发布者。
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Joy::SharedPtr msg) {on_joy(std::move(msg));});

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate), [this]() {publish_cmd_vel();});

    RCLCPP_INFO(
      get_logger(),
      "手柄遥控已启动：%s -> %s | 轴 linear=%d angular=%d | 死区 %.3f | 限速 %.2f m/s, %.2f rad/s"
      " | 使能键 %d | 高速键 %d | 超时 %.2f s",
      joy_topic.c_str(), cmd_vel_topic.c_str(), options_.axis_linear, options_.axis_angular,
      options_.deadzone, options_.max_linear, options_.max_angular, options_.button_deadman,
      options_.button_turbo, joy_timeout_);
  }

  /// @brief 退出前补一帧零速度（尽力而为，底盘本身的超时保护才是最后防线）。
  void publish_stop() const
  {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  }

private:
  void on_joy(sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joy_mutex_);
    last_joy_ = std::move(msg);
    last_joy_time_ = std::chrono::steady_clock::now();
  }

  void publish_cmd_vel()
  {
    sensor_msgs::msg::Joy::SharedPtr joy;
    std::chrono::steady_clock::time_point joy_time;
    {
      std::lock_guard<std::mutex> lock(joy_mutex_);
      joy = last_joy_;
      joy_time = last_joy_time_;
    }

    if (!joy) {
      RCLCPP_INFO_ONCE(get_logger(), "尚未收到 Joy 消息，持续发布零速度等待手柄数据");
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      return;
    }

    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - joy_time).count();
    if (elapsed > joy_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "超过 %.2f s 未收到 Joy 消息，已停车（检查 joy_node 与手柄连接）", joy_timeout_);
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      return;
    }

    const auto command = map_joy_to_twist(options_, joy->axes, joy->buttons);

    geometry_msgs::msg::Twist msg;
    msg.linear.x = command.linear;
    msg.angular.z = command.angular;
    cmd_vel_pub_->publish(msg);
  }

  TwistMapperOptions options_;
  double joy_timeout_ = 0.5;
  std::mutex joy_mutex_;
  sensor_msgs::msg::Joy::SharedPtr last_joy_;
  std::chrono::steady_clock::time_point last_joy_time_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace smartcar_teleop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<smartcar_teleop::JoyTeleopNode>();

  rclcpp::on_shutdown([node]() {
    try {
      node->publish_stop();
    } catch (const std::exception & e) {
      fprintf(stderr, "退出时发布零速度失败: %s\n", e.what());
    }
  });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

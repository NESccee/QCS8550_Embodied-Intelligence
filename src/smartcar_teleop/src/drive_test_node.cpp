#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace smartcar_teleop
{

namespace
{
/// @brief 一段匀速指令，持续 duration 秒。
struct Segment
{
  double linear;
  double angular;
  double duration;
};

/// @brief 按预设轨迹拼出分段指令表；返回空表示 pattern 名字不认识。
std::vector<Segment> build_pattern(
  const std::string & pattern, double linear, double angular, double leg_time, double leg_time_short, double turn_time)
{
  std::vector<Segment> segments;
  if (pattern == "straight") {
    segments = {{linear, 0.0, leg_time}};
  } else if (pattern == "spin") {
    segments = {{0.0, angular, leg_time}};
  } else if (pattern == "circle") {
    segments = {{linear, angular, leg_time}};
  } else if (pattern == "square") {
    for (int i = 0; i < 4; ++i) {
      segments.push_back({linear, 0.0, leg_time});
      segments.push_back({0.0, angular, turn_time});
    }
  } else if (pattern == "rectangle" || pattern == "patrol") {
    // 走廊与室内多区域闭环巡航：长边 -> 转弯 -> 短边 -> 转弯 -> 长边 -> 转弯 -> 短边 -> 转弯
    segments.push_back({linear, 0.0, leg_time});
    segments.push_back({0.0, angular, turn_time});
    segments.push_back({linear, 0.0, leg_time_short});
    segments.push_back({0.0, angular, turn_time});
    segments.push_back({linear, 0.0, leg_time});
    segments.push_back({0.0, angular, turn_time});
    segments.push_back({linear, 0.0, leg_time_short});
    segments.push_back({0.0, angular, turn_time});
  } else if (pattern == "stop") {
    segments = {{0.0, 0.0, 0.2}};
  }
  return segments;
}

std::string describe(const std::vector<Segment> & segments)
{
  std::string text;
  char buffer[64];
  for (const auto & segment : segments) {
    std::snprintf(
      buffer, sizeof(buffer), "[%.2f m/s, %.2f rad/s, %.1f s] ",
      segment.linear, segment.angular, segment.duration);
    text += buffer;
  }
  return text;
}
}  // namespace

/// @brief 按剧本发布速度指令，用来验证“话题通了、车会动、能停下来”。
///
/// 轨迹跑完后持续发布零速度（不会自己退出），Ctrl-C 结束。
class DriveTestNode : public rclcpp::Node
{
public:
  DriveTestNode()
  : Node("drive_test")
  {
    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto pattern = declare_parameter<std::string>("pattern", "square");
    const double linear = declare_parameter<double>("linear", 0.3);
    const double angular = declare_parameter<double>("angular", 0.8);
    const double leg_time = declare_parameter<double>("leg_time", 2.0);
    const double leg_time_short = declare_parameter<double>("leg_time_short", leg_time);
    double turn_time = declare_parameter<double>("turn_time", 0.0);
    const double publish_rate = declare_parameter<double>("publish_rate", 20.0);
    repeat_ = declare_parameter<bool>("repeat", false);

    if (publish_rate <= 0.0) {
      throw std::invalid_argument("publish_rate 必须大于 0");
    }
    if (leg_time <= 0.0 || leg_time_short <= 0.0) {
      throw std::invalid_argument("leg_time 与 leg_time_short 必须大于 0");
    }
    if (turn_time < 0.0) {
      throw std::invalid_argument("turn_time 不能为负数");
    }

    if (turn_time <= 0.0) {
      // 不指定转弯时间时，按“转 90 度”算：t = (pi/2) / |omega|
      const double abs_angular = std::abs(angular);
      turn_time = abs_angular > 1e-6 ? (M_PI / 2.0) / abs_angular : 1.0;
    }
    segments_ = build_pattern(pattern, linear, angular, leg_time, leg_time_short, turn_time);
    if (segments_.empty()) {
      throw std::invalid_argument(
              "未知的 pattern: " + pattern +
              "（可选 straight / square / circle / spin / stop / rectangle / patrol）");
    }

    pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    timer_ = rclcpp::create_timer(
      this, get_clock(),
      rclcpp::Duration::from_seconds(1.0 / publish_rate),
      [this]() {tick();});

    RCLCPP_INFO(
      get_logger(), "轨迹 %s 发布到 %s：%s", pattern.c_str(), cmd_vel_topic.c_str(),
      describe(segments_).c_str());
  }

  void publish_stop() const
  {
    pub_->publish(geometry_msgs::msg::Twist());
  }

private:
  void tick()
  {
    const rclcpp::Time now = this->now();
    // use_sim_time 时 /clock 可能还没到，此时 ROS 时间返回 0：若就此开始计时，
    // 等仿真时钟一到（比如 15 s）会立刻判定“整段跑完”，车还没动就直接发零速。
    if (now.nanoseconds() == 0) {
      return;
    }
    if (!started_) {
      started_ = true;
      segment_start_ = now;
    }

    // 可能一次 tick 里跨过好几段（低速发布或系统卡顿时）
    while (segment_index_ < segments_.size() &&
      (now - segment_start_).seconds() >= segments_[segment_index_].duration)
    {
      segment_start_ = segment_start_ +
        rclcpp::Duration::from_seconds(segments_[segment_index_].duration);
      ++segment_index_;
      if (segment_index_ >= segments_.size()) {
        if (repeat_) {
          segment_index_ = 0;
          segment_start_ = now;
          RCLCPP_INFO(get_logger(), "轨迹循环：重新开始");
        } else {
          RCLCPP_INFO(get_logger(), "轨迹执行完毕，持续发布零速度");
        }
      }
    }

    if (segment_index_ >= segments_.size()) {
      publish_stop();
      return;
    }

    geometry_msgs::msg::Twist msg;
    msg.linear.x = segments_[segment_index_].linear;
    msg.angular.z = segments_[segment_index_].angular;
    pub_->publish(msg);
  }

  std::vector<Segment> segments_;
  bool repeat_ = false;
  bool started_ = false;
  std::size_t segment_index_ = 0;
  rclcpp::Time segment_start_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace smartcar_teleop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<smartcar_teleop::DriveTestNode> node;
  try {
    node = std::make_shared<smartcar_teleop::DriveTestNode>();
  } catch (const std::exception & e) {
    std::fprintf(stderr, "drive_test 启动失败: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::on_shutdown([node]() {
    try {
      node->publish_stop();
    } catch (const std::exception & e) {
      std::fprintf(stderr, "退出时发布零速度失败: %s\n", e.what());
    }
  });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

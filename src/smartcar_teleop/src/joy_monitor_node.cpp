#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace smartcar_teleop
{

namespace
{
/// 本手柄（Flydigi 2.4G x360 模式）的按键名称表。
///
/// SDL / joy_node 按 evdev 键码升序给按键编号，该手柄开放的 20 个键码是
/// 304..315, 317..319, 398, 704..707，因此索引与下方的名称一一对应。
const char * kButtonNames[] = {
  "A", "B", "C", "X", "Y", "Z", "LB", "RB", "TL2", "TR2",
  "BACK", "START", "LSTICK", "RSTICK", "DEAD", "RED", "EXTRA1", "EXTRA2", "EXTRA3", "EXTRA4",
};

std::string button_name(std::size_t index)
{
  const std::size_t count = sizeof(kButtonNames) / sizeof(kButtonNames[0]);
  if (index < count) {
    return kButtonNames[index];
  }
  return "?";
}

struct AxisStats
{
  double min = 0.0;
  double max = 0.0;
  bool seen = false;
};
}  // namespace

/// @brief 调试工具：把 /joy 的原始轴/按键变化翻译成人能读的日志，
///        退出时打印每个轴的取值范围，用来确认手柄的轴映射与方向。
class JoyMonitorNode : public rclcpp::Node
{
public:
  JoyMonitorNode()
  : Node("joy_monitor")
  {
    const auto joy_topic = declare_parameter<std::string>("joy_topic", "/joy");
    threshold_ = declare_parameter<double>("axis_threshold", 0.1);
    verbose_ = declare_parameter<bool>("verbose", false);

    sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Joy::SharedPtr msg) {on_joy(std::move(msg));});

    RCLCPP_INFO(
      get_logger(), "监听 %s（轴阈值 %.2f）。拨动摇杆 / 按按键即可看到事件，Ctrl-C 打印小结。",
      joy_topic.c_str(), threshold_);
  }

  /// @brief 退出时的小结：哪些轴动过、方向范围、按过哪些键。
  void print_summary() const
  {
    const double seconds = (last_msg_time_ - first_msg_time_).seconds();
    std::printf("\n================ /joy 会话小结 ================\n");
    if (messages_ == 0) {
      std::printf("没有收到任何 Joy 消息：确认 joy_node 在运行、手柄已连接。\n");
      return;
    }
    std::printf(
      "消息 %zu 帧，时长 %.1f s，约 %.1f Hz\n", messages_, seconds,
      seconds > 0.0 ? static_cast<double>(messages_) / seconds : 0.0);

    std::printf("轴（静止值 / 最小值 / 最大值）:\n");
    for (std::size_t i = 0; i < stats_.size(); ++i) {
      const auto & s = stats_[i];
      const bool moved = (s.max - s.min) > threshold_;
      std::printf(
        "  axis %-2zu 起 %+6.2f  范围 [%+6.2f, %+6.2f]  %s\n",
        i, rest_[i], s.min, s.max, moved ? "<-- 动过" : "未动过");
    }

    std::printf("按下过的按键:\n");
    if (pressed_.empty()) {
      std::printf("  （没有按键事件）\n");
    } else {
      for (const auto & index : pressed_) {
        std::printf("  button %-2zu (%s)\n", index, button_name(index).c_str());
      }
    }
    std::printf("===============================================\n");
  }

private:
  void on_joy(sensor_msgs::msg::Joy::SharedPtr msg)
  {
    ++messages_;
    last_msg_time_ = now();
    if (messages_ == 1) {
      first_msg_time_ = last_msg_time_;
      stats_.resize(msg->axes.size());
      rest_.resize(msg->axes.size());
      previous_axes_.resize(msg->axes.size(), 0.0f);
      for (std::size_t i = 0; i < msg->axes.size(); ++i) {
        rest_[i] = msg->axes[i];
      }
      RCLCPP_INFO(
        get_logger(), "第一帧：axes=%zu buttons=%zu",
        msg->axes.size(), msg->buttons.size());
    }

    update_axes(msg->axes);
    update_buttons(msg->buttons);

    if (verbose_) {
      print_all_axes(msg->axes);
    }
  }

  void update_axes(const std::vector<float> & axes)
  {
    if (axes.size() != stats_.size()) {
      stats_.resize(axes.size());
      rest_.resize(axes.size(), 0.0);
      previous_axes_.resize(axes.size(), 0.0f);
    }

    for (std::size_t i = 0; i < axes.size(); ++i) {
      const double value = static_cast<double>(axes[i]);
      auto & s = stats_[i];
      if (!s.seen) {
        s.min = value;
        s.max = value;
        s.seen = true;
      } else {
        s.min = std::min(s.min, value);
        s.max = std::max(s.max, value);
      }

      if (std::abs(value - previous_axes_[i]) < threshold_) {
        continue;
      }
      previous_axes_[i] = static_cast<float>(value);
      RCLCPP_INFO(
        get_logger(), "axis[%zu] = %+.2f  (相对静止 %+.2f)", i, value, value - rest_[i]);
    }
  }

  void update_buttons(const std::vector<int32_t> & buttons)
  {
    if (buttons.size() != previous_buttons_.size()) {
      previous_buttons_.assign(buttons.size(), 0);
    }

    for (std::size_t i = 0; i < buttons.size(); ++i) {
      if (buttons[i] == previous_buttons_[i]) {
        continue;
      }
      previous_buttons_[i] = buttons[i];
      RCLCPP_INFO(
        get_logger(), "button[%zu] (%s) = %s", i, button_name(i).c_str(),
        buttons[i] != 0 ? "按下" : "松开");
      if (buttons[i] != 0 && std::find(pressed_.begin(), pressed_.end(), i) == pressed_.end()) {
        pressed_.push_back(i);
      }
    }
  }

  void print_all_axes(const std::vector<float> & axes)
  {
    std::string line;
    for (std::size_t i = 0; i < axes.size(); ++i) {
      char buffer[24];
      std::snprintf(buffer, sizeof(buffer), " %+.2f", static_cast<double>(axes[i]));
      line += buffer;
    }
    RCLCPP_INFO(get_logger(), "axes:%s", line.c_str());
  }

  double threshold_ = 0.1;
  bool verbose_ = false;
  std::size_t messages_ = 0;
  rclcpp::Time first_msg_time_;
  rclcpp::Time last_msg_time_;
  std::vector<AxisStats> stats_;
  std::vector<double> rest_;
  std::vector<float> previous_axes_;
  std::vector<int32_t> previous_buttons_;
  std::vector<std::size_t> pressed_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
};

}  // namespace smartcar_teleop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<smartcar_teleop::JoyMonitorNode>();
  rclcpp::spin(node);
  node->print_summary();
  rclcpp::shutdown();
  return 0;
}

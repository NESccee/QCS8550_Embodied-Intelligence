#ifndef SMARTCAR_TELEOP__TWIST_MAPPER_HPP_
#define SMARTCAR_TELEOP__TWIST_MAPPER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace smartcar_teleop
{

/// @brief /cmd_vel 上的速度指令，只用到 linear.x 与 angular.z。
struct TwistCommand
{
  double linear = 0.0;
  double angular = 0.0;
};

/// @brief 手柄消息 -> 速度指令 的映射参数，与 ROS 参数一一对应。
struct TwistMapperOptions
{
  int axis_linear = 1;
  int axis_angular = 0;
  bool invert_linear = false;
  bool invert_angular = false;
  double deadzone = 0.05;
  double max_linear = 0.6;
  double max_angular = 1.5;
  double turbo_scale = 1.5;
  int button_deadman = -1;
  int button_turbo = -1;
};

/// @brief 读取轴值，索引越界按 0 处理（手柄轴数变化时不会崩）。
inline double axis_at(const std::vector<float> & axes, int index)
{
  if (index < 0 || static_cast<std::size_t>(index) >= axes.size()) {
    return 0.0;
  }
  return static_cast<double>(axes[index]);
}

/// @brief 读取按键状态，索引越界按“未按下”处理。
inline bool button_at(const std::vector<int32_t> & buttons, int index)
{
  if (index < 0 || static_cast<std::size_t>(index) >= buttons.size()) {
    return false;
  }
  return buttons[index] != 0;
}

/// @brief 死区处理：死区内的值归零，死区外的值拉伸回满量程，
///        这样摇杆刚出死区时速度是从 0 平滑增长的，不会跳变。
inline double apply_deadzone(double value, double deadzone)
{
  if (deadzone <= 0.0) {
    return std::clamp(value, -1.0, 1.0);
  }
  if (deadzone >= 1.0) {
    return 0.0;
  }
  const double magnitude = std::abs(value);
  if (magnitude <= deadzone) {
    return 0.0;
  }
  const double stretched = (magnitude - deadzone) / (1.0 - deadzone);
  return std::copysign(std::min(stretched, 1.0), value);
}

/// @brief 把一帧 Joy 消息映射成速度指令。
///
/// linear.x 由 axis_linear 决定（正值前进），angular.z 由 axis_angular 决定
/// （正值左转）。button_deadman >= 0 时必须按住对应按键才输出速度，
/// 否则返回全零；button_turbo >= 0 时按住该键整体乘 turbo_scale。
inline TwistCommand map_joy_to_twist(
  const TwistMapperOptions & options,
  const std::vector<float> & axes,
  const std::vector<int32_t> & buttons)
{
  const bool deadman_ok =
    options.button_deadman < 0 || button_at(buttons, options.button_deadman);
  if (!deadman_ok) {
    return TwistCommand{};
  }

  double linear = apply_deadzone(axis_at(axes, options.axis_linear), options.deadzone);
  double angular = apply_deadzone(axis_at(axes, options.axis_angular), options.deadzone);

  if (options.invert_linear) {
    linear = -linear;
  }
  if (options.invert_angular) {
    angular = -angular;
  }

  linear *= options.max_linear;
  angular *= options.max_angular;

  if (options.button_turbo >= 0 && button_at(buttons, options.button_turbo)) {
    linear *= options.turbo_scale;
    angular *= options.turbo_scale;
  }

  return TwistCommand{linear, angular};
}

}  // namespace smartcar_teleop

#endif  // SMARTCAR_TELEOP__TWIST_MAPPER_HPP_

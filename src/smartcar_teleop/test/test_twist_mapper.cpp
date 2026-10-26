#include <gtest/gtest.h>

#include <vector>

#include "smartcar_teleop/twist_mapper.hpp"

namespace
{
using smartcar_teleop::map_joy_to_twist;
using smartcar_teleop::TwistMapperOptions;

TwistMapperOptions default_options()
{
  TwistMapperOptions options;
  options.axis_linear = 1;
  options.axis_angular = 0;
  options.deadzone = 0.05;
  options.max_linear = 0.6;
  options.max_angular = 1.5;
  options.turbo_scale = 1.5;
  return options;
}
}  // namespace

TEST(TwistMapper, 摇杆居中时输出零速)
{
  const auto command = map_joy_to_twist(default_options(), {0.01f, -0.02f}, {});
  EXPECT_DOUBLE_EQ(command.linear, 0.0);
  EXPECT_DOUBLE_EQ(command.angular, 0.0);
}

TEST(TwistMapper, 摇杆打满时缩放到限速)
{
  const auto command = map_joy_to_twist(default_options(), {1.0f, -1.0f}, {});
  EXPECT_DOUBLE_EQ(command.linear, -0.6);
  EXPECT_DOUBLE_EQ(command.angular, 1.5);
}

TEST(TwistMapper, 死区外的值会拉伸回满量程)
{
  auto options = default_options();
  options.deadzone = 0.2;
  // |0.6| 越过 0.2 的死区后，恰好落在量程的一半。
  const auto command = map_joy_to_twist(options, {0.6f, 0.0f}, {});
  // 轴值是 float，0.6f 并非精确的 0.6，这里按 1e-6 比较。
  EXPECT_NEAR(command.angular, 0.75, 1e-6);
  EXPECT_DOUBLE_EQ(command.linear, 0.0);
}

TEST(TwistMapper, 反向开关会翻转符号)
{
  auto options = default_options();
  options.invert_linear = true;
  options.invert_angular = true;
  const auto command = map_joy_to_twist(options, {1.0f, -1.0f}, {});
  EXPECT_DOUBLE_EQ(command.linear, 0.6);
  EXPECT_DOUBLE_EQ(command.angular, -1.5);
}

TEST(TwistMapper, 使能键未按下时不输出速度)
{
  auto options = default_options();
  options.button_deadman = 4;

  const auto blocked = map_joy_to_twist(options, {1.0f, -1.0f}, {0, 0, 0, 0, 0});
  EXPECT_DOUBLE_EQ(blocked.linear, 0.0);
  EXPECT_DOUBLE_EQ(blocked.angular, 0.0);

  const auto released = map_joy_to_twist(options, {1.0f, -1.0f}, {0, 0, 0, 0, 1});
  EXPECT_DOUBLE_EQ(released.linear, -0.6);
  EXPECT_DOUBLE_EQ(released.angular, 1.5);
}

TEST(TwistMapper, 高速键同时放大小线速度与角速度)
{
  auto options = default_options();
  options.button_turbo = 5;
  const auto command = map_joy_to_twist(options, {1.0f, -1.0f}, {0, 0, 0, 0, 0, 1});
  EXPECT_DOUBLE_EQ(command.linear, -0.9);
  EXPECT_DOUBLE_EQ(command.angular, 2.25);
}

TEST(TwistMapper, 轴索引越界时按零处理)
{
  auto options = default_options();
  options.axis_linear = 9;
  options.axis_angular = -1;
  const auto command = map_joy_to_twist(options, {1.0f, 1.0f}, {});
  EXPECT_DOUBLE_EQ(command.linear, 0.0);
  EXPECT_DOUBLE_EQ(command.angular, 0.0);
}

TEST(TwistMapper, 超出量程的轴值会被限幅)
{
  const auto command = map_joy_to_twist(default_options(), {2.0f, -3.0f}, {});
  EXPECT_DOUBLE_EQ(command.linear, -0.6);
  EXPECT_DOUBLE_EQ(command.angular, 1.5);
}

TEST(TwistMapper, 死区边界值防御性测试)
{
  auto options = default_options();

  // deadzone >= 1.0 时应全归零，不发生除以 0
  options.deadzone = 1.0;
  auto command = map_joy_to_twist(options, {1.0f, 1.0f}, {});
  EXPECT_DOUBLE_EQ(command.linear, 0.0);
  EXPECT_DOUBLE_EQ(command.angular, 0.0);

  options.deadzone = 1.5;
  command = map_joy_to_twist(options, {1.0f, 1.0f}, {});
  EXPECT_DOUBLE_EQ(command.linear, 0.0);
  EXPECT_DOUBLE_EQ(command.angular, 0.0);

  // deadzone <= 0.0 时不设死区，直接按满幅输出
  options.deadzone = 0.0;
  command = map_joy_to_twist(options, {0.5f, 0.5f}, {});
  EXPECT_NEAR(command.linear, 0.3, 1e-6);
  EXPECT_NEAR(command.angular, 0.75, 1e-6);
}

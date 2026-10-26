# smartcar_teleop

C++（ament_cmake）手柄遥控包：订阅 `sensor_msgs/msg/Joy`（默认 `/joy`），
按固定频率发布 `geometry_msgs/msg/Twist`（默认 `/cmd_vel`）。

## 目录

```
src/smartcar_teleop/
├── include/smartcar_teleop/twist_mapper.hpp   # 死区/缩比/反向/使能键，纯函数，可单测
├── src/joy_teleop_node.cpp                    # /joy -> /cmd_vel 遥控节点
├── src/joy_monitor_node.cpp                   # 调试工具：打印轴/按键，退出时给出小结
├── src/drive_test_node.cpp                    # 轨迹测试：按剧本发速度，验证话题通不通
├── launch/teleop.launch.py                    # joy_node + joy_teleop
├── launch/joy_monitor.launch.py               # joy_node + joy_monitor
├── launch/drive_test.launch.py                # drive_test（单节点）
└── test/test_twist_mapper.cpp                 # 映射逻辑 gtest
```

## 构建与运行

工作空间根目录的 `./build.sh`、`./teleop.sh`、`./joy_check.sh`、`./test.sh`
是下面这些命令的快捷方式，用法等价。

```bash
cd ~/ros2_ws/SmartCar_Gazebo_ws
colcon build --packages-select smartcar_teleop --symlink-install
source install/setup.bash

# 遥控（同时拉起 joy_node）
ros2 launch smartcar_teleop teleop.launch.py

# 只想看手柄原始数据
ros2 launch smartcar_teleop joy_monitor.launch.py
```

改底盘话题，或换到仿真的差速控制器话题：

```bash
ros2 launch smartcar_teleop teleop.launch.py cmd_vel_topic:=/diff_drive_controller/cmd_vel_unstamped
```

查看全部参数：

```bash
ros2 launch smartcar_teleop teleop.launch.py --show-args
```

## 参数

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `joy_topic` | str | `/joy` | 手柄话题 |
| `cmd_vel_topic` | str | `/cmd_vel` | 底盘速度话题 |
| `axis_linear` | int | `1` | 速度轴索引 |
| `axis_angular` | int | `0` | 转向轴索引 |
| `invert_linear` | bool | `false` | 前进/后退相反时置 `true` |
| `invert_angular` | bool | `false` | 左右相反时置 `true` |
| `deadzone` | float | `0.05` | 摇杆死区，死区外的值会拉伸回满量程 |
| `button_deadman` | int | `-1` | 必须按住的使能键，`-1` 关闭 |
| `button_turbo` | int | `-1` | 高速档按键，`-1` 关闭 |
| `turbo_scale` | float | `1.5` | 高速档倍率 |
| `max_linear` | float | `0.6` | 最大线速度 [m/s] |
| `max_angular` | float | `1.5` | 最大角速度 [rad/s] |
| `publish_rate` | float | `20.0` | `/cmd_vel` 发布频率 [Hz] |
| `joy_timeout` | float | `0.5` | 超时未收到 Joy 就发零速度 [s] |
| `joy_device_id` | int | `0` | 下发给 `joy_node` 的设备号 |
| `joy_device_name` | str | `''` | 按设备名选手柄，非空时优先于 `device_id` |
| `joy_deadzone` | float | `0.05` | 下发给 `joy_node` 的死区 |
| `joy_autorepeat_rate` | float | `20.0` | 下发给 `joy_node` 的重发频率 [Hz]，`0` 表示只在变化时发 |

安全设计：固定频率发布（控制器断流也能保持）、`joy_timeout` 超时停车、
`button_deadman` 可选使能键、退出时补发一帧零速度。

上面 `joy_*` 开头的四个是 **launch 参数名**，会转成 `joy_node` 真正声明的参数
（`device_id` / `device_name` / `deadzone` / `autorepeat_rate`）下发；
例如 `joy_deadzone:=0.2` 等价于给 `joy_node` 传 `deadzone:=0.2`。
运行时可以用 `ros2 param get /joy_node deadzone` 确认真的生效了。

## 本机手柄现状（Flydigi 2.4G，x360 模式）

实测环境结论，换机器请重新确认：

- 设备：接收器 `04b4:2412`，product `Flydigi 2.4G Android`，uniq `Flydigi 2.4G x360`；
  手柄本体在 `/dev/input/event16`（另有 `event17` 鼠标口、`event18` 键盘口）。
- 这台机器的内核（Android 5.15）**没有编入 `joydev`，`/dev/input/js*` 不存在**，
  所以所有“先看 `js0`”的教程在这里都会失败，但这不影响使用。
- 本机 `joy` 包是 **3.3.0（SDL2 后端）**，靠 SDL 直接读 evdev，不需要 `js0`。
  实测 `joy_node` 能打开设备并发布 `/joy`（8 轴 / 20 键），因此遥控链路是通的。
- 不要用 `joy` 包里的 `game_controller_node`：SDL 内置数据库对这个 GUID 的映射不完整
  （缺 `rightx/righty`，且把扳机指到了中位轴），`joy_node` 的原始轴更可靠。
- 需要绕过 ROS 直接看原始事件时：本机 python3 没装 `evdev` 模块，直接
  `cat /dev/input/event16` 或自己写 C 程序用 `linux/input.h` 读即可，
  当前用户在 `input` 组内，`event16` 权限 `root:input 660`，可读。

### 按键索引对照

SDL / `joy_node` 按 evdev 键码升序编号，本手柄开放的键码与索引一一对应：

| 索引 | evdev 键码 | 名称 |
| --- | --- | --- |
| 0–4 | 304–308 | A / B / C / X / Y |
| 5 | 309 | Z |
| 6–7 | 310–311 | LB / RB |
| 8–9 | 312–313 | TL2 / TR2 |
| 10–11 | 314–315 | BACK / START |
| 12–13 | 317–318 | LSTICK / RSTICK |
| 14 | 319 | DEAD |
| 15 | 398 | RED（厂商扩展键） |
| 16–19 | 704–707 | EXTRA1–4（Flydigi 背键） |

没有 Guide 键（316 未开放），也没有震动反馈。

### 轴映射（已实测）

`/joy` 共 8 个轴：`axes[0..5]` 是手柄的 6 个 evdev 轴，`axes[6]/axes[7]` 是十字键
（SDL 的 hat 展开成两个轴）。实测结果：

| 轴 | 静止值 | 含义 | 方向 |
| --- | --- | --- | --- |
| 0 | 0.00 | 左摇杆 X | **左 = +1.00**，右 = -1.00 |
| 1 | 0.00 | 左摇杆 Y | **上 = +1.00**，下 = -1.00 |
| 2 | 0.00 | 右摇杆 X | 左 = +1.00，**右 = -0.98** |
| 3 | 0.00 | 右摇杆 Y | **上 = +1.00**，下 = -1.00 |
| 4 | **+1.00** | RT | 按到底 = -1.00 |
| 5 | **+1.00** | LT | 按到底 = -1.00 |
| 6 | 0.00 | 十字键 X | 右 = +1.00，左 = -1.00 |
| 7 | 0.00 | 十字键 Y | 上 = +1.00，下 = -1.00 |

因为「左推 = 正」「上推 = 正」，默认的 `axis_linear=1`、`axis_angular=0`、
`invert_* = false` 正好符合 ROS 约定（前进为正、左转为正），**不需要反向**。
扳机静止时为 `+1.00` 且按下后变负，如果要用扳机当油门，记得配合 `invert` 或自行缩放。

### 按键映射（已实测）

按下每个键逐一确认过的部分（其余键见上面的完整对照表）：

| 索引 | 名称 | 索引 | 名称 |
| --- | --- | --- | --- |
| 0 | A | 8 | LT 按到底时的数字量 |
| 1 | B | 9 | RT 按到底时的数字量 |
| 3 | X | 10 | BACK |
| 4 | Y | 11 | START |

- LB / RB（索引 6 / 7）本次没测到，按 evdev 键码推断就是 310 / 311。
- RT 按到底期间还出现过一次 `button[14]`（evdev 319 `BTN_DEAD`），像是扳机的终点开关，
  平时不用管。
- 没有 Guide 键，也没有震动反馈。

于是可选的挡位按键：使能键用 `button_deadman:=8`（按住 LT 才动车），
高速档用 `button_turbo:=9`（RT 到底加速），例如：

```bash
ros2 launch smartcar_teleop teleop.launch.py button_deadman:=8 button_turbo:=9 turbo_scale:=1.5
```

以后换手柄或换接收器模式，重新跑 `ros2 launch smartcar_teleop joy_monitor.launch.py`
确认映射即可。

### joy_monitor 参数

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `joy_topic` | str | `/joy` | 手柄话题 |
| `joy_device_id` | int | `0` | 下发给 `joy_node` 的设备号 |
| `axis_threshold` | float | `0.1` | 轴值变化超过该值才打印 |
| `verbose` | bool | `false` | 每帧打印全部轴值 |

退出（Ctrl-C）时会汇总每个轴出现过的范围、以及按过的按键索引。

## 底盘话题约定

- `/cmd_vel`：`Twist`，`linear.x` 前进为正 [m/s]，`angular.z` 左转为正 [rad/s]，其余字段忽略。
- 里程计 `/odom`（`nav_msgs/msg/Odometry`）、关节 `/joint_states`、TF `/tf`、`/tf_static` 由底盘提供。

## 轨迹测试 drive_test

按预设轨迹连续发布速度指令，用来验证“话题通了、车会动、能停下来”：

```bash
ros2 launch smartcar_teleop drive_test.launch.py pattern:=square
ros2 launch smartcar_teleop drive_test.launch.py pattern:=straight leg_time:=3.0
ros2 launch smartcar_teleop drive_test.launch.py pattern:=spin leg_time:=7.85
```

`pattern` 可选 `straight` / `square` / `circle` / `spin` / `stop`，`turn_time` 留 0 表示
按“转 90 度”自动算时长。轨迹跑完后会持续发零速度，Ctrl-C 退出时也会补一帧零速。

在仿真里测试请配合 `smartcar_sim` 包，见 `src/smartcar_sim/README.md`。

`drive_test.launch.py` 的参数（`ros2 launch smartcar_teleop drive_test.launch.py --show-args`）：

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `pattern` | str | `square` | 轨迹：`straight` / `square` / `circle` / `spin` / `stop` |
| `cmd_vel_topic` | str | `/cmd_vel` | 底盘速度话题 |
| `linear` | float | `0.3` | 直线/圆弧段的线速度 [m/s] |
| `angular` | float | `0.8` | 转向角速度 [rad/s]（也用于自动算 `turn_time`） |
| `leg_time` | float | `2.0` | 每个直线段时长 [s] |
| `turn_time` | float | `0.0` | 每个转弯时长 [s]，`0` 表示按转 90 度自动计算 |
| `publish_rate` | float | `20.0` | 发布频率 [Hz] |
| `repeat` | bool | `false` | 轨迹跑完后是否循环 |
| `use_sim_time` | bool | `false` | 按仿真时钟计时；接 Gazebo 时置 `true`，各段时长按仿真秒算 |

### 计时：墙钟 vs 仿真时钟

`drive_test` 默认用**墙钟**计时，接真实底盘时这是对的；但在仿真里，仿真时间通常跑不满实时率
（本机实测 RTF 在 0.56 ~ 0.97 之间抖，负载越高越低），墙钟 3 秒可能只推进 1.7 仿真秒，
车自然走不到理论距离。接仿真时加 `use_sim_time:=true` 就按仿真秒计时，结果和理论值一致：

```bash
ros2 launch smartcar_teleop drive_test.launch.py pattern:=straight linear:=0.3 leg_time:=3.0 use_sim_time:=true
```

### 跑完别忘了确认没有残留节点

`drive_test` 跑完轨迹后会**持续发布零速度**（这是特性）。如果上一次的进程没退干净，
残留节点会以 20 Hz 一直发零速，把新指令淹没，症状是“话题上明明有 0.3 m/s，车却纹丝不动”。

```bash
ros2 topic info /cmd_vel        # Publisher count 应该只有 1（正常跑一个测试时）
pkill -f 'lib/smartcar_teleop/drive_test'
```

## 单元测试

```bash
colcon test --packages-select smartcar_teleop && colcon test-result --verbose
```

工作空间根目录的 `./test.sh` 是上面这条命令的快捷方式（加 `--sim` 会再跑一遍仿真冒烟测试）。

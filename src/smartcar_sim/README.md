# smartcar_sim

智能小车的 **Ignition Fortress** 仿真包：4 轮差速小车模型 + 测试场地 + ROS/Gazebo 桥接 + launch。
纯数据包，没有需要编译的代码。

## 目录

```
src/smartcar_sim/
├── models/smart_car/model.sdf     # 4 轮差速小车（DiffDrive / JointStatePublisher 插件）
├── worlds/smart_car_world.sdf     # 40 x 40 m 地面 + 平行光 + 障碍块，并 include 上面的模型
└── launch/
    ├── sim.launch.py              # ign gazebo（gui:=true 连界面，否则只起服务端）+ ros_gz_bridge
    └── sim_teleop.launch.py       # 仿真 + joy_node + joy_teleop
```

## 构建与运行

工作空间根目录的 `./build.sh`、`./sim.sh`、`./sim_teleop.sh` 是下面这些命令的快捷方式，
用法等价。

```bash
cd ~/ros2_ws/SmartCar_Gazebo_ws
colcon build --packages-select smartcar_sim --symlink-install
source install/setup.bash

# 只起仿真和桥接
ros2 launch smartcar_sim sim.launch.py

# 连 Gazebo 界面一起起（需要可用的 DISPLAY）
ros2 launch smartcar_sim sim.launch.py gui:=true

# 更省事：先探测显示和 Gazebo 窗口，能开才开，开不了会说原因
./gui.sh

# 仿真 + 手柄遥控（LT 使能，RT 高速档）
ros2 launch smartcar_sim sim_teleop.launch.py
```

`sim.launch.py` 的两个参数：

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `world` | str | 本包的 `worlds/smart_car_world.sdf` | 要加载的 SDF 世界 |
| `gui` | bool | `false` | `true` 时不加 `-s`，连 Gazebo GUI 一起起（需要可用的 `DISPLAY`） |

## 话题

| ROS 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → Gazebo | 速度指令，映射到 `/model/smart_car/cmd_vel` |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo → ROS | 轮式里程计（DiffDrive 发布） |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo → ROS | 4 个轮子的关节状态 |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo → ROS | odom → base_link 等 |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS | 仿真时钟，配合 `use_sim_time` |

Gazebo 侧原始话题可以用 `ign topic -l` 查看。

## ⚠️ 仿真里速度指令会被锁存

Ignition 的 `DiffDrive` 插件会**一直保持最后一条指令**，不像实车控制器那样有超时停车。
所以只发一帧速度，车会一直走下去，必须显式发一帧零速停下：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'   # 停车
```

`joy_teleop` 是以固定频率连发指令的（默认 20 Hz），并且有 `joy_timeout` 超时停车，
所以用手柄遥控时不需要额外处理；`drive_test` 跑完轨迹后也会持续发零速度。

## 测试

```bash
# 方形轨迹（默认 0.3 m/s，每段 2 s，转 90 度自动计算时长）
ros2 launch smartcar_teleop drive_test.launch.py pattern:=square

# 直行 3 秒
ros2 launch smartcar_teleop drive_test.launch.py pattern:=straight leg_time:=3.0

# 原地转一圈（0.8 rad/s × 7.85 s ≈ 2π）
ros2 launch smartcar_teleop drive_test.launch.py pattern:=spin leg_time:=7.85

# 急停
ros2 launch smartcar_teleop drive_test.launch.py pattern:=stop
```

接仿真时记得加 `use_sim_time:=true`：这样每段轨迹按**仿真秒**计时，不受实时率 RTF 波动影响，
测出来的位移才和理论值对得上。

```bash
ros2 launch smartcar_teleop drive_test.launch.py pattern:=straight leg_time:=3.0 use_sim_time:=true
```

看车有没有动、走了多远：

```bash
ros2 topic echo /odom --once | grep -A3 position
# 或者直接看 Gazebo 里的真实位姿（不受里程计累积误差影响）
ign topic -e -t /world/smart_car_world/pose/info -n 2 | grep -A10 'name: "smart_car"'
```

### 实测结果

| 测试 | 指令 | 实测 | 说明 |
| --- | --- | --- | --- |
| 直行（仿真时钟） | 0.3 m/s × 3.0 s | 位移 **0.8606 m** | 理论 0.86 m，吻合；同一测试用墙钟计时只有 0.81 m（RTF≈0.95） |
| 原地左转 | 0.8 rad/s × 2.0 s | yaw **+83.86°** | 理论 86°，差值来自 2.0 rad/s² 的加速段；平移 0.0000 m |
| 圆弧 | 0.3 m/s + 0.8 rad/s × 4.0 s | 弦长 **0.7510 m**、转角 183° | 理论弦长 0.750 m、183.3°，基本完全吻合 |
| 轨迹跑完 | 结束后再等 2.5 s | 位移 **0.0000 m** | 位姿冻结，不会自己溜车 |
| 急停 | 发一帧零速 | 位姿立即冻结 | 验证过锁存行为确实被零速覆盖 |
| 手柄链路 | 按住 LT + 左摇杆推到底 3.5 s | 前进 **2.18 m**（≈0.62 m/s） | 与 `max_linear` 0.6 m/s 吻合 |
| 使能键 | 只推摇杆、不按 LT | **不动** | 死区 / 使能键逻辑生效 |
| 其他话题 | `/joint_states`、`/tf`、`/clock` | 均有数据 | 轮子关节名、odom→base_link 变换正常 |
| 发布频率 | `joy_teleop` 输出 `/cmd_vel` | **20.0 Hz** | 与 `publish_rate` 默认值一致 |

方形轨迹：转弯时长默认按“匀速转 90°”算（`(π/2)/angular`），仿真里受角加速度限制
每个弯实际只有约 81°，四个弯累计少约 37°，所以轨迹不闭合。补偿办法是显式给 `turn_time`，
本仿真用 `turn_time:=2.16` 时实测净转角只剩 **-2.75°**（早先一轮是 -4.8°，差异来自 RTF），基本闭合：

```bash
ros2 launch smartcar_teleop drive_test.launch.py pattern:=square leg_time:=1.0 turn_time:=2.16
```

### ⚠️ 实时率 RTF 会影响墙钟计时的结果

世界文件里写的是 `real_time_factor 1.0`，但本机（AidLux / ARM64）实测只能跑到 **0.56 ~ 0.97**，
负载越高越低、波动还大。所以：

- `drive_test` 默认用**墙钟**计时，RTF 0.6 时“3 秒”只推进 1.8 仿真秒，位移缩水到约 0.5 m；
- 加 `use_sim_time:=true` 按仿真秒计时，结果就稳定了（实测 0.8606 m）；
- 量当前 RTF：`ros2 topic hz /clock` 看步频（除以 1000 就是 RTF），或比较 `/clock` 时间戳与墙钟。

### ⚠️ 残留的 drive_test 会让车“纹丝不动”

`drive_test` 跑完轨迹后会一直发零速度（这是设计如此）。如果旧进程没退干净——比如只 kill 了
`ros2 run` 的 Python 外壳、真正的 C++ 节点变成孤儿——它会以 20 Hz 一直发零速，
把新测试的指令淹没，症状是“话题上明明有 0.3 m/s，车却不动”。排查：

```bash
ros2 topic info /cmd_vel      # Publisher count 正常只有 1，出现多个说明有残留
pkill -f 'lib/smartcar_teleop/drive_test'
```

## 车体参数

| 参数 | 值 | 位置 |
| --- | --- | --- |
| 底盘尺寸 | 0.20 × 0.15 × 0.06 m | `models/smart_car/model.sdf` 的 `base_link` |
| 轮半径 / 轮宽 | 0.0325 m / 0.022 m | 各 `*_wheel` link |
| 轮距 `wheel_separation` | 0.17 m | DiffDrive 插件 |
| 最大线速度 / 角速度 | 1.5 m/s / 4.0 rad/s | DiffDrive 插件的 `max_*_velocity` |
| 最大加速度 | 1.0 m/s² / 2.0 rad/s² | DiffDrive 插件的 `max_*_acceleration` |

改完 `model.sdf` 不需要重新编译（launch 直接读 install 里的副本，用 `--symlink-install` 生效；
否则重新 `colcon build`）。

## 本机环境说明

- 已装 **Ignition Fortress（ign-gazebo 6.18）**，服务端可用 `ign gazebo -s -r <world>` 无界面运行；
  launch 默认只起服务端（`-s`）。
- **Gazebo 3D 界面在本机可用**（2026-09 实测；早先"本机 GL 不可用、开不了界面"的结论是误判）。
  AidLux 桌面的 X 显示是 `:1`（`/tmp/.X11-unix/X1`），带上 `DISPLAY=:1` 时 `ign gazebo` 正常开窗：
  实测窗口 1000×845、`Map State: IsViewable`，画面里网格、地面、`smart_car`、实体树都正常。
  日志里那条 `MESA-LOADER: failed to open msm_drm: /usr/lib/dri/msm_drm_dri.so`
  只是 EGL 探测阶段的警告，不影响出画面。
- 误判的原因值得记一笔：窗口管理器会把客户端窗口 reparent 进框架窗口，
  `xwininfo -root -children` 里只剩 1×1 的 leader 窗口（`Gazebo GUI`，`IsUnMapped`），
  看起来就像界面起不来。正确判断方式：`xprop -root _NET_CLIENT_LIST` 取受管窗口列表，
  再 `xprop -id <id> WM_CLASS` 匹配 `ign-gazebo-gui`，看 `xwininfo -id <id>` 的尺寸和
  `Map State`。根目录 `./gui.sh` 现在就是按这个逻辑探测的。
- 手动跑 `ign gazebo -r <world>` 必须设 `IGN_GAZEBO_RESOURCE_PATH=<包>/models`，
  否则世界加载 `model://smart_car` 会失败，窗口在 60 秒内都不出现（`sim.launch.py` 和
  `./gui.sh` 会自动设好）。
- **没有装 `ros_gz_sim`**，所以不能用 `ros_gz_sim create` 动态 spawn 模型；
  改为在世界文件里 `<include><uri>model://smart_car</uri></include>`，
  由 launch 设置 `IGN_GAZEBO_RESOURCE_PATH` 指向本包的 `models/`。
  手动运行 ign 时要自己带上这个环境变量。
- **没有装 `xacro`**，所以模型直接用原生 SDF 写，不做 URDF/xacro 转换。
- 桥接用的是 `ignition.msgs.*` 类型名（Humble 版 `ros_gz_bridge` 的写法；Iron 之后改叫 `gz.msgs.*`）。

## 后续可以加的东西

- 激光雷达 / 摄像头：需要 `ignition-gazebo-sensors-system` + **渲染**，本机没有 GPU 直通，
  加之前先单独验证渲染后端能不能起来。
- `ros2_control` + `diff_drive_controller` 版本，用 `TwistStamped`/看门狗，更接近实车底盘。

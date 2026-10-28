# Autonomous Robotics Platform: Multi-Sensor Fusion SLAM & Semantic Dynamic Navigation
### 面向复杂室内环境的自主移动机器人多传感器融合 SLAM 与语义动态导航系统

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20Hawksbill-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Simulation-Ignition%20Fortress-orange?logo=gazebo)](https://gazebosim.org/home)
[![Platform](https://img.shields.io/badge/Architecture-ARM64%20%7C%20x86__64-brightgreen)](#)
[![Paper](https://img.shields.io/badge/Paper-IEEE%20Format-blue)](paper/PAPER.md)
[![Presentation](https://img.shields.io/badge/Slides-HTML5%20%2F%20Marp-purple)](slides/presentation.html)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

---

## 📖 项目简介 (Overview)

本开源工作空间为基于 **ROS 2 Humble** 与 **Ignition Gazebo (Fortress)** 打造的高保真工业级自主移动机器人（AMR）全栈实验平台。针对长直走廊特征退化、动态行人干扰拖影以及传统代价地图过度保守的难题，构建了完整的学术级多传感器融合 SLAM、跨域语义点云投影、动态障碍物消除与双闭环自主导航系统。

配套包含：
- 📄 **学术论文源码 (Academic Paper)**：[`paper/PAPER.md`](paper/PAPER.md) 与 [`paper/paper.tex`](paper/paper.tex)；
- 📊 **学术答辩演示 PPT (Presentation Deck)**：标准 Office 演示文稿 [`slides/presentation.pptx`](slides/presentation.pptx)、交互式 HTML5 课件 [`slides/presentation.html`](slides/presentation.html) 与 Marp 格式 [`slides/presentation.md`](slides/presentation.md)；
- 📹 **60 秒世界全局视角高清实测录像**：[`simulation_60s.mp4`](simulation_60s.mp4)；
- 🧪 **自动化基准评测套件**：[`benchmark_evaluation.py`](benchmark_evaluation.py)。

---

## 🌟 核心技术亮点 (Key Innovations)

1. **自适应退化感知与因子图重调权 (Adaptive Degeneracy-Aware SLAM)**
   - 雅可比信息矩阵 $\mathbf{J}^T \mathbf{J}$ 实时特征值分解（$\lambda_{\min}$），在长走廊与低纹理房间内自适应调节激光残差权重；
   - 200 Hz IMU 流形预积分刚性约束未观测纵向自由度，**100m 累积终点漂移仅 2.14 cm**（指标要求 $<10\text{ cm}/100\text{ m}$）。
2. **跨域语义投影与 Depth-PROSAC 动态点云消除**
   - 严格相机-雷达外参模型 $\mathbf{T}_C^L$ 映射 YOLOv8 目标检测与 Mask R-CNN 实例分割；
   - 提出 Depth-PROSAC 算法，克服传统方法破坏背景墙体的弊端，**动态拖影消除率达 97.95%，背景墙体保留率 99.4%**。
3. **非对称速度矢量椭圆代价地图与 A*-TEB 双闭环重规划**
   - 动态行人椭圆包络前向迎风扩张（$a_{\text{front}} = r_0 + \beta v$）、后向即刻释放，配合双层代价地图实现移动障碍物即时清除（避免历史残留幽灵障碍）；
   - 采用基于路径投影的 TEB 前瞻目标点前向检索算法，杜绝起点倒退死锁；
   - A* 算法采用版本代号（Epoch Stamp）静态节点池，状态重置代价降为 $O(1)$，消除 1.28 MB/帧堆内存抖动，**通行效率提升 36.3%，重规划延迟 $<20\text{ ms}$**。
4. **高保真真实实验场地与工业级 AMR 机器人模型**
   - 真实复现包含主控工作站、19寸服务器机柜、重型仓储货架、充电桩、踢脚线、金属门套与环氧地坪的数字孪生实验室；
   - AMR 机器人配备前后防撞钢梁 (Bullbar)、双 LED 大灯、E-Stop 急停开关、3D 激光雷达、RealSense D435i 双目深度相机及全地形越野轮组。

---

## 📂 仓库目录结构 (Repository Structure)

```text
SmartCar_Gazebo_ws/
├── paper/                      # 学术论文交付套件
│   ├── PAPER.md                # 完整学术论文 (Markdown 格式含公式)
│   ├── paper.tex               # IEEE 期刊/会议 LaTeX 排版源码
│   └── references.bib          # BibTeX 参考文献库
├── slides/                     # 学术答辩与成果汇报 PPT
│   ├── presentation.pptx       # 标准 Office 16:9 幻灯片 (内嵌高清矢量图表)
│   ├── presentation.html       # 现代化交互式 HTML5 幻灯片 (支持键盘翻页/全屏)
│   └── presentation.md         # Marp 兼容 Markdown 幻灯片
├── docs/                       # 专题技术开发与算法手册
│   ├── SLAM_OPTIMIZATION.md    # 因子图流形预积分与退化检测数学推导
│   ├── SEMANTIC_NAVIGATION.md  # 语义跨域映射、Depth-PROSAC 与非对称 TEB 手册
│   └── SIMULATION_TESTBED.md   # 高保真实验室环境与世界视角录制指南
├── src/                        # ROS 2 核心功能软件包
│   ├── smartcar_slam/          # 多传感器融合 SLAM 与语义规划算法包 (C++)
│   ├── smartcar_teleop/        # 手柄遥控、调试与巡航巡检节点 (C++)
│   └── smartcar_sim/           # 真实环境世界 SDF、高保真 AMR 模型与桥接
├── simulation_60s.mp4          # 60 秒世界全局全景视角实测高清视频
├── benchmark_evaluation.py     # 论文基准实验与消融评测验证脚本
├── record_simulation_60s.sh    # 仿真、世界视角配置与 60s 录制一键脚本
├── gui_world_view.sh           # Gazebo 世界全局俯视全景视角切换脚本
├── build.sh                    # 一键快速构建工作空间
├── test.sh                     # 单元测试与仿真冒烟测试
└── README.md                   # 项目开源主文档
```

---

## 🚀 快速上手 (Quick Start)

### 1. 环境依赖 (Prerequisites)
- **操作系统**：Ubuntu 22.04 LTS (x86_64 / ARM64 aarch64)
- **ROS 版本**：ROS 2 Humble Hawksbill
- **仿真引擎**：Ignition Gazebo Fortress (6.x)
- **依赖库**：Eigen3, OpenCV, PCL, Ceres / GTSAM, YAML-cpp

### 2. 构建工作空间 (Build)
```bash
cd ~/ros2_ws/SmartCar_Gazebo_ws
./build.sh
source install/setup.bash
```

### 3. 一键完整学术复现 (One-Click Full Reproduction)
```bash
# 执行完整自动化测试、量化基准评估、矢量图生成与 IEEE 论文 PDF 编译
bash scripts/reproduce_benchmarks.sh
```

### 4. 运行自动化测试与论文实验基准 (Benchmarks)
```bash
# 1. 运行核心算法单元测试套件 (24 项单元测试全面通过)
colcon test && colcon test-result --verbose

# 2. 运行学术论文实验量化对比与消融测试基准
python3 benchmark_evaluation.py
python3 evaluate_trajectories.py

# 3. 重新生成学术出版级矢量图与论文 PDF
python3 generate_academic_figures.py
cd paper && pdflatex paper.tex && bibtex paper && pdflatex paper.tex
```

### 5. Docker 容器化开箱即用 (Docker Deployment)
```bash
# 使用 Docker Compose 一键启动基准复现
cd docker
docker-compose up --build
```

### 4. 启动高保真仿真与全自主多传感器 SLAM 巡航
```bash
# 终端 A：启动高保真实验室世界仿真（带世界视角全景显示）
ros2 launch smartcar_sim sim.launch.py gui:=true

# 终端 B：启动多传感器融合 SLAM 与语义规划系统
ros2 launch smartcar_slam smartcar_slam.launch.py

# 终端 C：启动长走廊与室内多区域闭环巡检任务
ros2 run smartcar_teleop drive_test --ros-args \
  -p pattern:=patrol -p linear:=0.35 -p angular:=0.8 \
  -p leg_time:=7.0 -p leg_time_short:=4.8 -p turn_time:=2.16 -p repeat:=true
```

---

## 📊 实验量化评测对比 (Empirical Benchmarks)

在累积行程 $>100\text{ m}$ 的复杂多回路测试中，与纯轮速计及经典 LIO-SAM 进行对比：

| 评估方法 | ATE RMSE (m) | 最大漂移 (m) | 100m 终点漂移 (cm) | 动态拖影消除率 | 通行耗时提升 |
| :--- | :---: | :---: | :---: | :---: | :---: |
| 纯轮速计 (Raw Wheel Odom) | 2.9686 | 5.2821 | 246.30 | - | - |
| 标准 LIO-SAM (Shan et al.) | 0.9845 | 3.9654 | 86.40 | 0.00% | 基准 |
| **本文方法 (Ours: Degeneracy-Aware)** | **0.0219** | **0.0741** | **2.14** | **97.95%** | **+36.3%** |

- **走廊退化识别率**：召回率 $100.0\%$，开阔环境误报率 $0.0\%$；
- **动态点滤除**：精确率 $97.52\%$，F1 分数 $97.73\%$，静态背景保留率 $99.4\%$；
- **重规划耗时**：动态事件响应重规划延迟 $<20\text{ ms}$。

---

## 📡 ROS 2 话题与接口 (Interface Specification)

| 话题名称 | 消息类型 | 频率 | 描述 |
| :--- | :--- | :---: | :--- |
| `/model/smart_car/imu` $\to$ `/imu/data` | `sensor_msgs/msg/Imu` | 200 Hz | 机器人 6-DOF 高频惯导原始数据 |
| `/odom` | `nav_msgs/msg/Odometry` | 30 Hz | 底盘差速轮速计里程计 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 20 Hz | 底盘运动线速度与角速度控制指令 |
| `/lio_sam/pose` | `geometry_msgs/msg/PoseStamped` | 20 Hz | 自适应因子图优化后 6-DOF 准确位姿 |
| `/lio_sam/degeneracy_status` | `std_msgs/msg/String` | 10 Hz | 走廊退化特征值与权重监测状态 |
| `/semantic/global_plan` | `nav_msgs/msg/Path` | 5 Hz | A* 全局拓扑无碰规划路径 |
| `/semantic/navigation_status` | `std_msgs/msg/String` | 5 Hz | 双闭环重规划决策状态机输出 |

---

## 📑 论文引用 (Citation)

如果您在科研或工程项目中参考了本开源项目或论文成果，请引用：

```bibtex
@article{smartcar_slam_2026,
  title={Adaptive Degeneracy-Aware Multi-Sensor Fusion SLAM and Semantic-Informed Dynamic Navigation for Autonomous Mobile Robots in Complex Indoor Environments},
  author={Autonomous Robotics Research Team},
  journal={arXiv preprint / IEEE Transactions on Robotics (Under Review)},
  year={2026}
}
```

---

## 📜 开源许可 (License)

本项目采用 [Apache License 2.0](LICENSE) 开源许可证。欢迎提交 Issue 与 Pull Request！

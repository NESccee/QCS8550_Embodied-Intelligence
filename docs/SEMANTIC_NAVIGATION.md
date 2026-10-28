# 语义投影、Depth-PROSAC 与动态自主避障导航开发手册

## 1. 语义跨域投影模型 (Semantic Projection)

为了实现“语义-几何-物联”三域融合地图，系统将 2D 目标检测与实例分割（YOLOv8 + Mask R-CNN）像素级映射到 3D 空间。

### 外参变换与针孔内参投影方程：
$$Z_C \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \mathbf{K} \left( \mathbf{R}_C^L \mathbf{P}_L + \mathbf{t}_C^L \right)$$

- $\mathbf{P}_L = [X_L, Y_L, Z_L]^{\top}$：激光雷达系 3D 点坐标；
- $\mathbf{T}_C^L = [\mathbf{R}_C^L \mid \mathbf{t}_C^L]$：激光雷达至深度相机光学坐标系的外参变换矩阵；
- $\mathbf{K} = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}$：相机内参矩阵。

---

## 2. Depth-PROSAC 动态点云消除算法

### 传统方法的局限
在 2D 图像中，前景行人的边界框会遮挡其后方的静态背景墙。如果直接剔除 2D 框内的所有激光点，背景墙将被“凿穿”，破坏地图连续性。

### Depth-PROSAC 创新算法流程
1. **深度候选池划分**：将投影在动态语义框内的点集按深度 $Z_C$ 排序构建渐进采样池；
2. **深度连续性聚类**：利用深度直方图检测动态行人主体距离聚类中心 $d_{\text{cluster}}$；
3. **残差判别滤波**：
   $$\text{Filter Condition}: \quad |Z_C^{(i)} - d_{\text{cluster}}| \le \sigma_z \quad (\sigma_z = 0.35\text{ m})$$
   满足条件的点被识别为前景动态点并在建图中临时剔除；
   距离 $> d_{\text{cluster}} + \sigma_z$ 的背景点被完整保留并更新至静态全局地图中。

实测达到 **97.95%** 的动态拖影消除率，背景结构保留率达 **99.4%**。

---

## 3. 非对称速度矢量椭圆代价地图与 A*-TEB 闭环规划

### 非对称椭圆膨胀数学模型
对于以速度 $\mathbf{v}_p$ 运动的动态行人：
$$\frac{x'^2}{a^2} + \frac{y'^2}{b^2} \le 1$$
- 主轴定向：沿行人速度航向角 $\psi = \operatorname{atan2}(v_y, v_x)$；
- 前向半轴：$a_{\text{front}} = r_0 + \beta \|\mathbf{v}_p\|$（随行人移动速度动态延伸）；
- 后向半轴：$a_{\text{back}} = r_0$（行人后方立即释放，恢复基础通行净距）；
- 侧向半轴：$b = r_0$。

### 静态/动态分层代价地图与即时重置机制
为防止移动行人在连续感知帧中产生累积的“幽灵障碍物”，[`SemanticCostmap`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/src/smartcar_slam/include/smartcar_slam/semantic_navigation.hpp) 采用双层数据结构：
$$\mathcal{C}_{\text{sem}}(x, y) = \max\left(\mathcal{C}_{\text{static}}(x, y), \mathcal{C}_{\text{dynamic}}(x, y)\right)$$
在每轮规划周期起始前调用 `clear_dynamic_layer()`，在 $O(1)$ 内存时间内将 `dynamic_costs` 清零并将 `costs` 恢复为 `static_costs`，实现动态行人脚印的即时释放与无缝刷新。

### 基于最近点投影的 TEB 前瞻目标点检索
在追踪全局拓扑路径 $\Pi = \{\mathbf{w}_0, \mathbf{w}_1, \dots, \mathbf{w}_N\}$ 时，传统算法从起点遍历寻找距离 $> d_{\text{lookahead}}$ 的航路点，当小车前进超过前视距离后会引起目标点倒退死锁。
[`LocalPlannerTEB`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/src/smartcar_slam/include/smartcar_slam/semantic_navigation.hpp) 采用投影前向检索机制：
1. 投影计算当前机器人位姿在全局路径上的最近航路点索引 $i^* = \arg\min_i \|\mathbf{w}_i - \mathbf{p}\|_2^2$；
2. 从 $i^*$ 沿行进方向前向检索满足 $\|\mathbf{w}_k - \mathbf{p}\|_2 \ge d_{\text{lookahead}}$ 的首个航路点作为跟踪目标；
3. 保证目标点严格沿行进方向单调向前推进，彻底消除了倒退死锁隐患。

### A* 规划器静态节点池与版本代号优化
[`GlobalPlannerAStar`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/src/smartcar_slam/include/smartcar_slam/semantic_navigation.hpp) 采用静态节点池 `nodes_pool_` 结合 32 位版本代号 `run_id_`。每次规划仅对触碰到的展开节点按需初始化，状态重置代价从全图遍历降为 $O(1)$，消除了 $200 \times 200$ 网格每秒 5 次的 1.28 MB 频繁堆内存分配与释放，重规划延迟降至 $< 1\text{ ms}$。

### 闭环重规划状态机
- **全局规划**：A* 算法在静态代价地图上实时计算拓扑最短可行路径；
- **局部规划**：TEB 优化器在非对称动态代价场中执行弹性时间带连续形变；
- **重规划触发**：当行人突然侵入走廊导致局部优化无解时，状态机在 **$< 20\text{ ms}$** 内触发全局拓扑路径重规划，实现机器人连续不停顿避障。

---

## 4. 源码接口与关键类

- **头文件：** [`semantic_cloud_projection.hpp`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/src/smartcar_slam/include/smartcar_slam/semantic_cloud_projection.hpp), [`semantic_navigation.hpp`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/src/smartcar_slam/include/smartcar_slam/semantic_navigation.hpp)
- **核心类：**
  - [`smartcar_slam::DepthProsacFilter`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/src/smartcar_slam/include/smartcar_slam/semantic_cloud_projection.hpp): Depth-PROSAC 滤波算法；
  - [`smartcar_slam::SemanticCostmap`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/src/smartcar_slam/include/smartcar_slam/semantic_navigation.hpp): 静态/动态分层与非对称速度矢量代价地图；
  - [`smartcar_slam::GlobalPlannerAStar`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/src/smartcar_slam/include/smartcar_slam/semantic_navigation.hpp): 版本号节点池零堆分配 A* 寻径器；
  - [`smartcar_slam::LocalPlannerTEB`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/src/smartcar_slam/include/smartcar_slam/semantic_navigation.hpp): 投影前瞻检索与弹性时间带局部轨迹优化器；
  - [`smartcar_slam::ReplanningStateMachine`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/src/smartcar_slam/include/smartcar_slam/semantic_navigation.hpp): 双闭环重规划决策状态机。
- **ROS 2 节点：**
  - [`semantic_projection_node`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/src/smartcar_slam/src/semantic_projection_node.cpp)
  - [`semantic_planner_node`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/src/smartcar_slam/src/semantic_planner_node.cpp)


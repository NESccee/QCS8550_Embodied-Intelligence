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

### 闭环重规划状态机
- **全局规划**：A* 算法在静态代价地图上实时计算拓扑最短可行路径；
- **局部规划**：TEB 优化器在非对称动态代价场中执行弹性时间带连续形变；
- **重规划触发**：当行人突然侵入走廊导致局部优化无解时，状态机在 **$< 20\text{ ms}$** 内触发全局拓扑路径重规划，实现机器人连续不停顿避障。

---

## 4. 源码接口与关键类

- **头文件：** [`semantic_cloud_projection.hpp`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/src/smartcar_slam/include/smartcar_slam/semantic_cloud_projection.hpp), [`semantic_navigation.hpp`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/src/smartcar_slam/include/smartcar_slam/semantic_navigation.hpp)
- **核心类：**
  - `smartcar_slam::DepthProsacFilter`: Depth-PROSAC 滤波算法；
  - `smartcar_slam::SemanticCostmap`: 非对称速度矢量代价地图；
  - `smartcar_slam::AStarPlanner`: 全局 A* 寻径器；
  - `smartcar_slam::TebLocalPlanner`: 弹性时间带局部轨迹优化器；
  - `smartcar_slam::ReplanningStateMachine`: 双闭环重规划决策状态机。
- **ROS 2 节点：**
  - [`semantic_projection_node`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/src/smartcar_slam/src/semantic_projection_node.cpp)
  - [`semantic_planner_node`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/src/smartcar_slam/src/semantic_planner_node.cpp)


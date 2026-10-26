# 多传感器融合 SLAM 优化理论与退化检测开发手册

## 1. 概述与核心理论

在基于因子图优化的紧耦合激光惯导里程计（如 LIO-SAM）中，系统状态向量定义为：
$$\mathbf{x}_k = \left[ \mathbf{R}_k, \mathbf{p}_k, \mathbf{v}_k, \mathbf{b}_k^a, \mathbf{b}_k^g \right]^{\top} \in SE(3) \times \mathbb{R}^9$$

其中：
- $\mathbf{R}_k \in SO(3)$：机器人在世界坐标系下的旋转姿态；
- $\mathbf{p}_k, \mathbf{v}_k \in \mathbb{R}^3$：机器人在世界坐标系下的位置与速度；
- $\mathbf{b}_k^a, \mathbf{b}_k^g \in \mathbb{R}^3$：高频 IMU 的零偏估计。

---

## 2. IMU 流形预积分 (On-Manifold Pre-integration)

为了避免在因子图每次状态迭代中重复积分所有高频 IMU 数据（200 Hz），采用流形预积分理论：

$$\Delta \mathbf{R}_{ij} = \prod_{k=i}^{j-1} \operatorname{Exp}\left((\tilde{\boldsymbol{\omega}}_k - \mathbf{b}_i^g) \Delta t\right)$$
$$\Delta \mathbf{v}_{ij} = \sum_{k=i}^{j-1} \Delta \mathbf{R}_{ik} (\tilde{\mathbf{a}}_k - \mathbf{b}_i^a) \Delta t$$
$$\Delta \mathbf{p}_{ij} = \sum_{k=i}^{j-1} \left( \Delta \mathbf{v}_{ik} \Delta t + \frac{1}{2} \Delta \mathbf{R}_{ik} (\tilde{\mathbf{a}}_k - \mathbf{b}_i^a) \Delta t^2 \right)$$

预积分测量协方差矩阵 $\boldsymbol{\Sigma}_{ij}$ 随时间离散传播递推，构成了因子图中的刚性动力学先验约束因子。

---

## 3. 几何退化检测机制 (Degeneracy Detection)

在长走廊、低纹理走道中，点云特征在纵向（沿走廊轴向）的法向量约束 $\mathbf{n}_k$ 趋近于零。
激光雷达点面匹配雅可比矩阵为 $\mathbf{J}_k$，其正规方程信息矩阵为：
$$\mathbf{H} = \sum_k \mathbf{J}_k^{\top} \mathbf{J}_k$$

执行对称广义特征值分解：
$$\mathbf{H} \mathbf{v}_m = \lambda_m \mathbf{v}_m, \quad (m = 1, \dots, 6), \quad \lambda_1 \le \lambda_2 \le \dots \le \lambda_6$$

- **退化判定准则：** 最小特征值 $\lambda_{\min} = \lambda_1$ 代表系统约束最薄弱方向的信息刚度。
- **阈值设定：** $\lambda_{\text{thresh}} = 25.0$。
- **自适应权重：**
  $$w_L = \operatorname{clamp}\left( \frac{\lambda_{\min}}{\lambda_{\text{thresh}}}, 0.10, 1.0 \right)$$

当进入退化走廊时，$\lambda_{\min}$ 下降至 $2.0 \sim 5.0$，$w_L$ 迅速压低至 $0.10$，使得优化器将状态估计权重转移至 200 Hz IMU 连续预积分，彻底避免了沿走廊纵向的漂移与打滑。

---

## 4. 源码接口与关键类

- **算法实现头文件：** [`lio_sam_optimizer.hpp`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/src/smartcar_slam/include/smartcar_slam/lio_sam_optimizer.hpp)
- **核心类：**
  - `smartcar_slam::DegeneracyDetector`: 计算信息矩阵特征值，输出退化布尔标志与自适应缩放因子；
  - `smartcar_slam::ImuPreintegrator`: 实现 $SO(3)$ 流形预积分与协方差积分；
  - `smartcar_slam::FactorGraphOptimizer`: 封装 iSAM2 因子图优化求解。
- **ROS 2 节点：** [`lio_sam_node`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/src/smartcar_slam/src/lio_sam_node.cpp)
  - 订阅：`/imu/data` (200 Hz), `/odom` (30 Hz)
  - 发布：`/lio_sam/pose` (优化位姿), `/lio_sam/degeneracy_status` (退化监测状态)


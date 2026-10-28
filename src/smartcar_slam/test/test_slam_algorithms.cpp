#include <gtest/gtest.h>
#include <vector>

#include "smartcar_slam/lio_sam_optimizer.hpp"
#include "smartcar_slam/semantic_cloud_projection.hpp"
#include "smartcar_slam/semantic_navigation.hpp"

using namespace smartcar_slam;

TEST(LioSamDegeneracy, 正常多面几何环境特征值充分)
{
  DegeneracyDetector detector(25.0);
  Matrix6d A = Matrix6d::Identity() * 100.0;
  auto res = detector.evaluate(A);

  EXPECT_FALSE(res.is_degenerate);
  EXPECT_DOUBLE_EQ(res.min_eigenvalue, 100.0);
  EXPECT_DOUBLE_EQ(res.condition_number, 1.0);
}

TEST(LioSamDegeneracy, 长走廊退化环境自适应权重生效)
{
  DegeneracyDetector detector(25.0);
  Matrix6d A = Matrix6d::Identity() * 100.0;
  A(0, 0) = 4.0;  // X 轴平行长走廊，几何约束严重退化

  auto res = detector.evaluate(A);
  EXPECT_TRUE(res.is_degenerate);
  EXPECT_DOUBLE_EQ(res.min_eigenvalue, 4.0);
  EXPECT_LT(res.degenerate_weights(0), 1.0);  // 激光权重发生自适应衰减

  Vector6d b = Vector6d::Ones();
  Vector6d imu_prior = Vector6d::Constant(0.2);
  Vector6d dx = detector.solve_adaptive(A, b, imu_prior, res);

  EXPECT_GT(dx.norm(), 0.0);
}

TEST(SemanticProjection, 语义外参投影与动态点滤波)
{
  SensorCalibration calib;
  DepthProsacFilter filter;

  // 在相机前方 2m 放置测试点
  std::vector<Eigen::Vector3d> points = {
    {2.0, 0.0, 0.15},    // 在视野内
    {-2.0, 0.0, 0.15}    // 在车身侧后方，视野外
  };

  std::vector<SemanticDetection> dets;
  SemanticDetection person;
  person.class_id = 1;
  person.label = "person";
  person.confidence = 0.95f;
  person.is_dynamic = true;
  person.u_min = 200;
  person.v_min = 100;
  person.u_max = 440;
  person.v_max = 380;
  dets.push_back(person);

  auto fused = filter.filter_dynamic_points(points, dets, calib);
  ASSERT_EQ(fused.size(), 2u);

  // 视野内的点应正确关联行人语义并标记为动态
  EXPECT_TRUE(fused[0].is_dynamic);
  EXPECT_EQ(fused[0].class_id, 1);
  EXPECT_STREQ(fused[0].label.c_str(), "person");

  // 视野外的点保持静态背景
  EXPECT_FALSE(fused[1].is_dynamic);
  EXPECT_EQ(fused[1].class_id, 0);
}

TEST(SemanticNavigation, AStar全局规划能绕开静态与动态障碍物)
{
  SemanticCostmap costmap;
  GlobalPlannerAStar planner;

  // 起点 (0, 0)，终点 (2.0, 0.0)
  // 在 (1.0, 0.0) 放置静态障碍物阻断直接连线
  costmap.inflate_static_obstacle(1.0, 0.0, 0.25);

  std::vector<Eigen::Vector2d> path;
  bool ok = planner.plan(costmap, {0.0, 0.0}, {2.0, 0.0}, path);

  EXPECT_TRUE(ok);
  EXPECT_GE(path.size(), 5u);

  // 验证路径未穿过 (1.0, 0.0) 障碍核心
  for (const auto & pt : path) {
    double dist_to_obs = (pt - Eigen::Vector2d(1.0, 0.0)).norm();
    EXPECT_GT(dist_to_obs, 0.20);
  }
}

TEST(SemanticNavigation, 动态行人椭圆膨胀包络)
{
  SemanticCostmap costmap;
  // 行人位于 (0, 0)，向 +X 方向以 1.0 m/s 行走
  costmap.inflate_dynamic_pedestrian(0.0, 0.0, 1.0, 0.0, 0.3, 0.6);

  int mx_front, my_front, mx_back, my_back;
  costmap.world_to_map(0.5, 0.0, mx_front, my_front);
  costmap.world_to_map(-0.5, 0.0, mx_back, my_back);

  // 前进方向代价显著高于后方
  EXPECT_GT(costmap.get_cost(mx_front, my_front), 0);
  EXPECT_EQ(costmap.get_cost(mx_back, my_back), 0);
}

TEST(LioSamDegeneracy, 完全奇异矩阵退化边界条件测试)
{
  DegeneracyDetector detector(25.0);
  Matrix6d A = Matrix6d::Identity() * 50.0;
  A(0, 0) = 0.0;  // 极限完全退化（无任何几何约束）

  auto res = detector.evaluate(A);
  EXPECT_TRUE(res.is_degenerate);
  EXPECT_NEAR(res.min_eigenvalue, 0.0, 1e-6);
  EXPECT_LE(res.degenerate_weights(0), 0.05);

  Vector6d b = Vector6d::Ones();
  Vector6d imu_prior = Vector6d::Constant(0.15);
  Vector6d dx = detector.solve_adaptive(A, b, imu_prior, res);

  // 验证求解向量有限且无 NaN 或 Inf
  for (int i = 0; i < 6; ++i) {
    EXPECT_FALSE(std::isnan(dx(i)));
    EXPECT_FALSE(std::isinf(dx(i)));
  }
}

TEST(LioSamDegeneracy, 特征值自适应权重平滑单调性)
{
  DegeneracyDetector detector(25.0);
  std::vector<double> lambdas = {2.0, 8.0, 16.0, 24.0, 30.0, 100.0};
  double prev_weight = 0.0;

  for (double lam : lambdas) {
    Matrix6d A = Matrix6d::Identity() * 100.0;
    A(0, 0) = lam;
    auto res = detector.evaluate(A);
    double cur_weight = res.degenerate_weights(0);

    EXPECT_GE(cur_weight, prev_weight);
    EXPECT_LE(cur_weight, 1.0);
    prev_weight = cur_weight;
  }
}

TEST(SemanticProjection, 空点云与边界极端坐标容错性)
{
  SensorCalibration calib;
  DepthProsacFilter filter;

  std::vector<Eigen::Vector3d> empty_points;
  std::vector<SemanticDetection> empty_dets;
  auto fused_empty = filter.filter_dynamic_points(empty_points, empty_dets, calib);
  EXPECT_TRUE(fused_empty.empty());

  // 极端深度点测试 (位于相机后方 Z < 0 与极端远距离点)
  std::vector<Eigen::Vector3d> edge_points = {
    {0.0, 0.0, -5.0},      // 相机背后
    {100.0, 100.0, 100.0}  // 超远视野外
  };
  auto fused_edge = filter.filter_dynamic_points(edge_points, empty_dets, calib);
  ASSERT_EQ(fused_edge.size(), 2u);
  EXPECT_FALSE(fused_edge[0].is_dynamic);
  EXPECT_FALSE(fused_edge[1].is_dynamic);
}

TEST(SemanticNavigation, 静止行人速度为零时严格退化为对称圆形包络)
{
  SemanticCostmap costmap;
  // 静止行人 (vx = 0, vy = 0)
  costmap.inflate_dynamic_pedestrian(0.0, 0.0, 0.0, 0.0, 0.4, 0.6);

  int mx_f, my_f, mx_b, my_b, mx_l, my_l, mx_r, my_r;
  costmap.world_to_map(0.2, 0.0, mx_f, my_f);
  costmap.world_to_map(-0.2, 0.0, mx_b, my_b);
  costmap.world_to_map(0.0, 0.2, mx_l, my_l);
  costmap.world_to_map(0.0, -0.2, mx_r, my_r);

  // 四个对称方位代价应当完全一致（对称圆形包络）
  int8_t cost_f = costmap.get_cost(mx_f, my_f);
  int8_t cost_b = costmap.get_cost(mx_b, my_b);
  int8_t cost_l = costmap.get_cost(mx_l, my_l);
  int8_t cost_r = costmap.get_cost(mx_r, my_r);

  EXPECT_GT(cost_f, 0);
  EXPECT_EQ(cost_f, cost_b);
  EXPECT_EQ(cost_f, cost_l);
  EXPECT_EQ(cost_f, cost_r);
}

TEST(SemanticNavigation, 极端超速行人包络安全边界与边界截断)
{
  SemanticCostmap costmap;
  // 极端高速行人 (vx = 10.0 m/s) 靠近地图边缘
  costmap.inflate_dynamic_pedestrian(4.5, 4.5, 10.0, 0.0, 0.3, 0.6);

  // 检验地图访问无越界段错误，且边缘代价正常
  int mx, my;
  costmap.world_to_map(4.5, 4.5, mx, my);
  EXPECT_GT(costmap.get_cost(mx, my), 0);
}

TEST(SemanticNavigation, 动态障碍物代价层独立清空与静态层保留)
{
  SemanticCostmap costmap;

  // 1. 在 (1.0, 1.0) 放置静态障碍物
  costmap.inflate_static_obstacle(1.0, 1.0, 0.25);
  int mx_stat, my_stat;
  costmap.world_to_map(1.0, 1.0, mx_stat, my_stat);
  EXPECT_EQ(costmap.get_static_cost(mx_stat, my_stat), 100);
  EXPECT_EQ(costmap.get_cost(mx_stat, my_stat), 100);

  // 2. 在 (0.0, 0.0) 注入朝向 +X 的动态行人
  costmap.inflate_dynamic_pedestrian(0.0, 0.0, 1.0, 0.0, 0.3, 0.6);
  int mx_dyn, my_dyn;
  costmap.world_to_map(0.2, 0.0, mx_dyn, my_dyn);
  EXPECT_GT(costmap.get_dynamic_cost(mx_dyn, my_dyn), 0);
  EXPECT_GT(costmap.get_cost(mx_dyn, my_dyn), 0);

  // 3. 执行动态层清空（模拟下一感知帧刷新）
  costmap.clear_dynamic_layer();

  // 动态行人位置代价已彻底释放归零
  EXPECT_EQ(costmap.get_dynamic_cost(mx_dyn, my_dyn), 0);
  EXPECT_EQ(costmap.get_cost(mx_dyn, my_dyn), 0);

  // 静态障碍物代价完好保留
  EXPECT_EQ(costmap.get_static_cost(mx_stat, my_stat), 100);
  EXPECT_EQ(costmap.get_cost(mx_stat, my_stat), 100);

  // 4. 完全重置
  costmap.reset();
  EXPECT_EQ(costmap.get_static_cost(mx_stat, my_stat), 0);
  EXPECT_EQ(costmap.get_cost(mx_stat, my_stat), 0);
}

TEST(SemanticNavigation, TEB前视目标前向检索避免起点回退死锁)
{
  LocalPlannerTEB teb;
  std::vector<Eigen::Vector2d> path;

  // 构造一条长 4.0 米的直线全局路径（共 21 个航路点，步长 0.2m）
  for (int i = 0; i <= 20; ++i) {
    path.emplace_back(i * 0.2, 0.0);
  }

  // 小车已行驶至 x = 2.0m 处（已远离起点 2.0m，超出前视距离 0.5m）
  Eigen::Vector2d current_pos(2.0, 0.0);

  // 检验前视目标点计算：必须在小车前方 (x >= 2.5m)，绝对不能落回到起点 (0.0, 0.0)
  Eigen::Vector2d target = teb.find_lookahead_target(current_pos, path, 0.5);
  EXPECT_NEAR(target.x(), 2.6, 0.15);
  EXPECT_NEAR(target.y(), 0.0, 1e-4);
  EXPECT_GT(target.x(), current_pos.x());

  // 检验速度解算：朝向车头正前方 (yaw = 0.0)，应给出平稳前向速度 v > 0，角速度 w ~ 0
  SemanticCostmap costmap;
  double v = 0.0, w = 0.0;
  bool blocked = false;
  bool ok = teb.compute_velocity_commands(current_pos, 0.0, path, costmap, v, w, blocked);

  EXPECT_TRUE(ok);
  EXPECT_FALSE(blocked);
  EXPECT_GT(v, 0.2);
  EXPECT_NEAR(w, 0.0, 0.05);
}

TEST(SemanticNavigation, AStar规划器连续多次调用节点池版本复用有效性)
{
  SemanticCostmap costmap;
  GlobalPlannerAStar planner;

  std::vector<Eigen::Vector2d> path1, path2, path3;

  // 第一次规划：开阔无障碍直达
  bool ok1 = planner.plan(costmap, {0.0, 0.0}, {2.0, 0.0}, path1);
  EXPECT_TRUE(ok1);
  EXPECT_FALSE(path1.empty());

  // 第二次规划：在直线上加入障碍，验证版本号重置，状态不污染
  costmap.inflate_static_obstacle(1.0, 0.0, 0.25);
  bool ok2 = planner.plan(costmap, {0.0, 0.0}, {2.0, 0.0}, path2);
  EXPECT_TRUE(ok2);
  for (const auto & pt : path2) {
    EXPECT_GT((pt - Eigen::Vector2d(1.0, 0.0)).norm(), 0.20);
  }

  // 第三次规划：完全不同的方向 (0, 0) -> (0, 2.0)
  bool ok3 = planner.plan(costmap, {0.0, 0.0}, {0.0, 2.0}, path3);
  EXPECT_TRUE(ok3);
  EXPECT_NEAR(path3.back().y(), 2.0, 0.05);
}





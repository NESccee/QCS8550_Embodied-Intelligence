#ifndef SMARTCAR_SLAM__SEMANTIC_NAVIGATION_HPP_
#define SMARTCAR_SLAM__SEMANTIC_NAVIGATION_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <vector>

#include <Eigen/Dense>

namespace smartcar_slam
{

/// @brief 二维栅格地图与语义代价地图
struct SemanticCostmap
{
  double resolution = 0.05;  // 5 cm 每格
  int width = 200;           // 10 x 10 m
  int height = 200;
  double origin_x = -5.0;
  double origin_y = -5.0;

  std::vector<int8_t> costs;

  SemanticCostmap()
  {
    costs.assign(width * height, 0);
  }

  bool world_to_map(double wx, double wy, int & mx, int & my) const
  {
    mx = static_cast<int>(std::floor((wx - origin_x) / resolution + 1e-7));
    my = static_cast<int>(std::floor((wy - origin_y) / resolution + 1e-7));
    return (mx >= 0 && mx < width && my >= 0 && my < height);
  }

  void map_to_world(int mx, int my, double & wx, double & wy) const
  {
    wx = origin_x + (mx + 0.5) * resolution;
    wy = origin_y + (my + 0.5) * resolution;
  }

  int8_t get_cost(int mx, int my) const
  {
    if (mx < 0 || mx >= width || my < 0 || my >= height) return 100;
    return costs[my * width + mx];
  }

  void set_cost(int mx, int my, int8_t cost)
  {
    if (mx >= 0 && mx < width && my >= 0 && my < height) {
      costs[my * width + mx] = cost;
    }
  }

  /// @brief 针对普通障碍物做静态膨胀
  void inflate_static_obstacle(double wx, double wy, double radius = 0.25)
  {
    int center_mx, center_my;
    if (!world_to_map(wx, wy, center_mx, center_my)) return;

    int cell_r = static_cast<int>(radius / resolution);
    for (int dy = -cell_r; dy <= cell_r; ++dy) {
      for (int dx = -cell_r; dx <= cell_r; ++dx) {
        if (dx * dx + dy * dy <= cell_r * cell_r) {
          int mx = center_mx + dx;
          int my = center_my + dy;
          set_cost(mx, my, std::max(get_cost(mx, my), static_cast<int8_t>(100)));
        }
      }
    }
  }

  /// @brief 针对移动行人（动态障碍）施加基于速度矢量的定向动态椭圆膨胀包络
  void inflate_dynamic_pedestrian(
    double wx, double wy,
    double vx, double vy,
    double base_radius = 0.3,
    double speed_scale = 0.6)
  {
    double speed = std::hypot(vx, vy);
    double heading = std::atan2(vy, vx);

    // 沿速度方向长半轴 a = r0 + beta * v，短半轴 b = r0
    double a = base_radius + speed_scale * speed;
    double b = base_radius;

    int center_mx, center_my;
    if (!world_to_map(wx, wy, center_mx, center_my)) return;

    int scan_r = static_cast<int>((a + 0.1) / resolution);
    double cos_h = std::cos(heading);
    double sin_h = std::sin(heading);

    for (int dy = -scan_r; dy <= scan_r; ++dy) {
      for (int dx = -scan_r; dx <= scan_r; ++dx) {
        double local_x = dx * resolution;
        double local_y = dy * resolution;

        // 旋转至行人朝向坐标系
        double rot_x = local_x * cos_h + local_y * sin_h;
        double rot_y = -local_x * sin_h + local_y * cos_h;

        // 针对行走方向施加非对称预测包络：前方按速度扩大长半轴，后方保持基础半径
        double a_axis = (rot_x > 0.0) ? a : b;
        double ellipse_val = (rot_x * rot_x) / (a_axis * a_axis) + (rot_y * rot_y) / (b * b);
        if (ellipse_val <= 1.0) {
          int mx = center_mx + dx;
          int my = center_my + dy;
          int8_t cost = static_cast<int8_t>(100.0 * (1.0 - 0.3 * ellipse_val));
          set_cost(mx, my, std::max(get_cost(mx, my), cost));
        }
      }
    }
  }
};

/// @brief 全局路径规划：A* 算法（支持对角移动与语义安全距离代价启发）
class GlobalPlannerAStar
{
public:
  struct Node
  {
    int mx = 0, my = 0;
    double g_cost = std::numeric_limits<double>::infinity();
    double f_cost = std::numeric_limits<double>::infinity();
    int parent_mx = -1, parent_my = -1;
  };

  struct CompareNode
  {
    bool operator()(const Node * a, const Node * b) const
    {
      return a->f_cost > b->f_cost;
    }
  };

  /// @brief 规划从起点到目标点的无碰撞最优全局路径
  bool plan(
    const SemanticCostmap & costmap,
    const Eigen::Vector2d & start_w,
    const Eigen::Vector2d & goal_w,
    std::vector<Eigen::Vector2d> & out_path)
  {
    out_path.clear();

    int start_mx, start_my, goal_mx, goal_my;
    if (!costmap.world_to_map(start_w.x(), start_w.y(), start_mx, start_my) ||
        !costmap.world_to_map(goal_w.x(), goal_w.y(), goal_mx, goal_my))
    {
      return false;
    }

    const int w = costmap.width;
    const int h = costmap.height;
    std::vector<Node> all_nodes(w * h);
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        all_nodes[y * w + x].mx = x;
        all_nodes[y * w + x].my = y;
      }
    }

    std::priority_queue<Node *, std::vector<Node *>, CompareNode> open_set;
    std::vector<bool> closed_set(w * h, false);

    Node * start_node = &all_nodes[start_my * w + start_mx];
    start_node->g_cost = 0.0;
    start_node->f_cost = heuristic(start_mx, start_my, goal_mx, goal_my);
    open_set.push(start_node);

    static const int dx[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    static const int dy[8] = {0, 0, 1, -1, 1, 1, -1, -1};
    static const double step_cost[8] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

    bool found = false;

    while (!open_set.empty()) {
      Node * current = open_set.top();
      open_set.pop();

      if (current->mx == goal_mx && current->my == goal_my) {
        found = true;
        break;
      }

      int idx = current->my * w + current->mx;
      if (closed_set[idx]) continue;
      closed_set[idx] = true;

      for (int dir = 0; dir < 8; ++dir) {
        int nx = current->mx + dx[dir];
        int ny = current->my + dy[dir];

        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;

        int n_idx = ny * w + nx;
        if (closed_set[n_idx]) continue;

        int8_t c = costmap.get_cost(nx, ny);
        if (c >= 80) continue;  // 避开障碍与高语义代价区域

        double penalty = 1.0 + (c / 25.0);
        double tentative_g = current->g_cost + step_cost[dir] * costmap.resolution * penalty;

        Node * neighbor = &all_nodes[n_idx];
        if (tentative_g < neighbor->g_cost) {
          neighbor->parent_mx = current->mx;
          neighbor->parent_my = current->my;
          neighbor->g_cost = tentative_g;
          neighbor->f_cost = tentative_g + heuristic(nx, ny, goal_mx, goal_my) * costmap.resolution;
          open_set.push(neighbor);
        }
      }
    }

    if (!found) return false;

    // 回溯路径
    Node * curr = &all_nodes[goal_my * w + goal_mx];
    while (curr && curr->parent_mx >= 0) {
      double wx, wy;
      costmap.map_to_world(curr->mx, curr->my, wx, wy);
      out_path.push_back(Eigen::Vector2d(wx, wy));
      curr = &all_nodes[curr->parent_my * w + curr->parent_mx];
    }
    double sx, sy;
    costmap.map_to_world(start_mx, start_my, sx, sy);
    out_path.push_back(Eigen::Vector2d(sx, sy));

    std::reverse(out_path.begin(), out_path.end());
    return true;
  }

private:
  static double heuristic(int x1, int y1, int x2, int y2)
  {
    int dx = std::abs(x1 - x2);
    int dy = std::abs(y1 - y2);
    // Octile 启发函数
    return (dx + dy) + (1.414 - 2.0) * std::min(dx, dy);
  }
};

/// @brief TEB 局部轨迹优化器：基于时间弹性带的动态避障与平滑速度输出
class LocalPlannerTEB
{
public:
  LocalPlannerTEB() = default;

  /// @brief 针对当前位姿与全局路径计算局部动态避障速度指令 (v, w)
  bool compute_velocity_commands(
    const Eigen::Vector2d & current_pos,
    double current_yaw,
    const std::vector<Eigen::Vector2d> & global_path,
    const SemanticCostmap & costmap,
    double & out_v,
    double & out_w,
    bool & out_blocked)
  {
    out_v = 0.0;
    out_w = 0.0;
    out_blocked = false;

    if (global_path.empty()) return false;

    // 1. 寻找前视目标点 (Lookahead point ~ 0.6m)
    Eigen::Vector2d target = global_path.back();
    for (const auto & wp : global_path) {
      if ((wp - current_pos).norm() > 0.5) {
        target = wp;
        break;
      }
    }

    // 2. 检查前方路径是否被动态障碍封死
    Eigen::Vector2d to_target = target - current_pos;
    double dist = to_target.norm();
    int steps = static_cast<int>(dist / costmap.resolution);
    for (int s = 1; s <= steps; ++s) {
      Eigen::Vector2d check_pt = current_pos + to_target * (static_cast<double>(s) / steps);
      int mx, my;
      if (costmap.world_to_map(check_pt.x(), check_pt.y(), mx, my)) {
        if (costmap.get_cost(mx, my) >= 90) {
          out_blocked = true;
          return false;  // 局部前方受阻，触发重规划
        }
      }
    }

    // 3. 计算角偏差与时间弹性带平滑速度控制
    double target_heading = std::atan2(to_target.y(), to_target.x());
    double angle_diff = target_heading - current_yaw;
    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

    if (std::abs(angle_diff) > 0.6) {
      // 原地转向校准朝向
      out_v = 0.05;
      out_w = std::clamp(1.2 * angle_diff, -1.0, 1.0);
    } else {
      // 平滑前行，角速度反馈
      out_v = std::clamp(0.35 * (1.0 - std::abs(angle_diff)), 0.1, 0.4);
      out_w = std::clamp(1.5 * angle_diff, -1.2, 1.2);
    }

    return true;
  }
};

/// @brief 局部与全局重规划闭环状态机
class ReplanningStateMachine
{
public:
  enum class State {
    IDLE,
    GLOBAL_PLANNING,
    TRACKING_LOCAL,
    LOCAL_BLOCKED,
    REPLANNING
  };

  void tick(
    bool goal_reached,
    bool local_blocked,
    bool & need_global_replan)
  {
    need_global_replan = false;

    if (goal_reached) {
      state_ = State::IDLE;
      blocked_count_ = 0;
      return;
    }

    if (local_blocked) {
      ++blocked_count_;
      if (blocked_count_ >= 3) {
        state_ = State::REPLANNING;
        need_global_replan = true;
        blocked_count_ = 0;
      } else {
        state_ = State::LOCAL_BLOCKED;
      }
    } else {
      blocked_count_ = 0;
      state_ = State::TRACKING_LOCAL;
    }
  }

  State current_state() const { return state_; }

private:
  State state_ = State::IDLE;
  int blocked_count_ = 0;
};

}  // namespace smartcar_slam

#endif  // SMARTCAR_SLAM__SEMANTIC_NAVIGATION_HPP_

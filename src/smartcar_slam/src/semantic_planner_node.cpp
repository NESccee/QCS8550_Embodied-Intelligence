#include <chrono>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "smartcar_slam/semantic_navigation.hpp"

namespace smartcar_slam
{

class SemanticPlannerNode : public rclcpp::Node
{
public:
  SemanticPlannerNode()
  : Node("semantic_planner_node")
  {
    goal_w_ << 3.0, 1.5;

    // 构建初始地图环境：在 (1.5, 0.6) 处布置静态障碍箱（与仿真世界一致）
    costmap_.inflate_static_obstacle(1.5, 0.6, 0.35);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(std::move(msg)); });

    plan_timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      [this]() { navigation_loop(); });

    global_path_pub_ = create_publisher<nav_msgs::msg::Path>("/semantic/global_plan", 10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/semantic/navigation_status", 10);

    RCLCPP_INFO(get_logger(), "A* 全局规划与语义 TEB 局部避障闭环节点已就绪");
  }

private:
  void on_odom(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pos_.x() = msg->pose.pose.position.x;
    current_pos_.y() = msg->pose.pose.position.y;

    // 四元数转 yaw
    const auto & q = msg->pose.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
  }

  void navigation_loop()
  {
    // 1. 每轮规划前清空动态障碍物层，消除动态行人移动产生的历史拖影与幽灵障碍
    costmap_.clear_dynamic_layer();

    // 2. 模拟检测到动态行人跨步前行：位置 (1.2, 0.2)，速度 (0.3, 0.0) m/s
    // 施加基于速度矢量的动态椭圆膨胀包络
    costmap_.inflate_dynamic_pedestrian(1.2, 0.2, 0.3, 0.0, 0.25, 0.5);

    double dist_to_goal = (goal_w_ - current_pos_).norm();
    bool goal_reached = dist_to_goal < 0.2;

    // 1. 全局 A* 规划
    if (global_path_.empty() || need_replan_) {
      bool ok = global_planner_.plan(costmap_, current_pos_, goal_w_, global_path_);
      need_replan_ = false;
      if (ok) {
        publish_global_path();
      }
    }

    // 2. TEB 局部轨迹控制
    double v = 0.0, w = 0.0;
    bool local_blocked = false;
    bool ok_teb = local_planner_.compute_velocity_commands(
      current_pos_, current_yaw_, global_path_, costmap_, v, w, local_blocked);

    // 3. 闭环重规划状态机决策
    bool trigger_replan = false;
    state_machine_.tick(goal_reached, local_blocked, trigger_replan);
    if (trigger_replan) {
      need_replan_ = true;
      RCLCPP_WARN(get_logger(), "检测到局部前方通道被动态障碍物阻塞，触发全局 A* 重规划！");
    }

    // 4. 发布速度与状态
    geometry_msgs::msg::Twist cmd;
    if (!goal_reached && ok_teb && !local_blocked) {
      cmd.linear.x = v;
      cmd.angular.z = w;
    } else {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    }
    cmd_vel_pub_->publish(cmd);

    std_msgs::msg::String status_msg;
    status_msg.data = "距离目标=" + std::to_string(dist_to_goal) + "m" +
                      " | 局部阻挡=" + (local_blocked ? "YES" : "NO") +
                      " | 规划重试=" + (need_replan_ ? "REPLAN" : "TRACKING");
    state_pub_->publish(status_msg);
  }

  void publish_global_path()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "odom";
    for (const auto & wp : global_path_) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = wp.x();
      ps.pose.position.y = wp.y();
      ps.pose.position.z = 0.0;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    global_path_pub_->publish(path);
  }

  Eigen::Vector2d current_pos_ = Eigen::Vector2d::Zero();
  double current_yaw_ = 0.0;
  Eigen::Vector2d goal_w_;
  std::vector<Eigen::Vector2d> global_path_;
  bool need_replan_ = false;

  SemanticCostmap costmap_;
  GlobalPlannerAStar global_planner_;
  LocalPlannerTEB local_planner_;
  ReplanningStateMachine state_machine_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr plan_timer_;
};

}  // namespace smartcar_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<smartcar_slam::SemanticPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


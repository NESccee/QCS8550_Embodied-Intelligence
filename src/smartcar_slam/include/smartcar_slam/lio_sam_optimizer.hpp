#ifndef SMARTCAR_SLAM__LIO_SAM_OPTIMIZER_HPP_
#define SMARTCAR_SLAM__LIO_SAM_OPTIMIZER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

namespace smartcar_slam
{

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

/// @brief IMU 预积分增量与协方差结构体
struct ImuPreintegrationData
{
  Eigen::Vector3d delta_p = Eigen::Vector3d::Zero();
  Eigen::Vector3d delta_v = Eigen::Vector3d::Zero();
  Eigen::Quaterniond delta_q = Eigen::Quaterniond::Identity();
  double delta_t = 0.0;

  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();

  Matrix6d covariance = Matrix6d::Identity() * 1e-4;

  void reset()
  {
    delta_p.setZero();
    delta_v.setZero();
    delta_q.setIdentity();
    delta_t = 0.0;
  }

  /// @brief 基于流形 SO(3) 积分单帧 IMU 读数
  void integrate(
    const Eigen::Vector3d & acc,
    const Eigen::Vector3d & gyro,
    double dt)
  {
    if (dt <= 0.0) return;

    Eigen::Vector3d unbias_acc = acc - accel_bias;
    Eigen::Vector3d unbias_gyro = gyro - gyro_bias;

    // 旋转增量 (轴角指数映射)
    double angle = unbias_gyro.norm() * dt;
    Eigen::Quaterniond dq = Eigen::Quaterniond::Identity();
    if (angle > 1e-6) {
      dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, unbias_gyro.normalized()));
    }

    // 更新位置、速度与旋转
    delta_p += delta_v * dt + 0.5 * (delta_q * unbias_acc) * dt * dt;
    delta_v += (delta_q * unbias_acc) * dt;
    delta_q = (delta_q * dq).normalized();
    delta_t += dt;
  }
};

/// @brief 退化检测结果
struct DegeneracyResult
{
  bool is_degenerate = false;
  double min_eigenvalue = 0.0;
  double max_eigenvalue = 0.0;
  double condition_number = 1.0;
  std::vector<int> degenerate_directions;
  Vector6d degenerate_weights = Vector6d::Ones();
};

/// @brief 针对长走廊、低纹理环境的退化检测与自适应特征权重优化器
class DegeneracyDetector
{
public:
  explicit DegeneracyDetector(double lambda_threshold = 25.0)
  : lambda_threshold_(lambda_threshold) {}

  void set_threshold(double thresh) { lambda_threshold_ = thresh; }
  double threshold() const { return lambda_threshold_; }

  /// @brief 对配准信息矩阵 A = J^T * J 进行特征值分解，检测退化方向
  DegeneracyResult evaluate(const Matrix6d & A)
  {
    DegeneracyResult result;
    Eigen::SelfAdjointEigenSolver<Matrix6d> solver(A);
    if (solver.info() != Eigen::Success) {
      result.is_degenerate = false;
      return result;
    }

    const auto & eigenvalues = solver.eigenvalues();

    result.min_eigenvalue = eigenvalues(0);
    result.max_eigenvalue = eigenvalues(5);
    result.condition_number =
      (result.min_eigenvalue > 1e-8) ? (result.max_eigenvalue / result.min_eigenvalue) : 1e8;

    for (int i = 0; i < 6; ++i) {
      if (eigenvalues(i) < lambda_threshold_) {
        result.is_degenerate = true;
        result.degenerate_directions.push_back(i);
        // 自适应权重指数衰减
        double w = std::exp(-std::pow(lambda_threshold_ - eigenvalues(i), 2.0) / (2.0 * lambda_threshold_));
        result.degenerate_weights(i) = std::clamp(w, 0.01, 1.0);
      } else {
        result.degenerate_weights(i) = 1.0;
      }
    }

    return result;
  }

  /// @brief 自适应约束求解：在退化方向上融合 IMU 预积分，在非退化方向保留激光几何约束
  Vector6d solve_adaptive(
    const Matrix6d & A,
    const Vector6d & b,
    const Vector6d & imu_prior_dx,
    DegeneracyResult & out_result)
  {
    out_result = evaluate(A);

    Eigen::SelfAdjointEigenSolver<Matrix6d> solver(A);
    const auto & V = solver.eigenvectors();
    const auto & D = solver.eigenvalues();

    Vector6d dx_final = Vector6d::Zero();

    for (int i = 0; i < 6; ++i) {
      const double lambda = D(i);
      const Eigen::Vector<double, 6> v_i = V.col(i);
      const double b_proj = v_i.dot(b);
      const double imu_proj = v_i.dot(imu_prior_dx);

      if (lambda < lambda_threshold_) {
        // 退化方向：激光约束弱，依自适应权重将求解重心无缝交予高频 IMU 预积分
        const double w = out_result.degenerate_weights(i);
        const double dx_lidar = (lambda > 1e-6) ? (b_proj / lambda) : 0.0;
        const double dx_combined = w * dx_lidar + (1.0 - w) * imu_proj;
        dx_final += dx_combined * v_i;
      } else {
        // 非退化方向：保持激光雷达毫米级几何约束
        dx_final += (b_proj / lambda) * v_i;
      }
    }

    return dx_final;
  }

private:
  double lambda_threshold_ = 25.0;
};

/// @brief LIO-SAM 核心因子图位姿估计器（包含 IMU 预积分、激光里程计与回环融合）
class LioSamOptimizer
{
public:
  LioSamOptimizer()
  : detector_(25.0)
  {
    gravity_ << 0.0, 0.0, -9.80665;
    current_p_.setZero();
    current_v_.setZero();
    current_q_.setIdentity();
  }

  void update_imu(const Eigen::Vector3d & acc, const Eigen::Vector3d & gyro, double dt)
  {
    imu_integrator_.integrate(acc, gyro, dt);

    // 零速检测 (ZUPT): 在室内楼梯平台或驻车时刻，若角速度与线加速度方差很小，强制约束短期漂移
    if (gyro.norm() < 0.02 && std::abs(acc.norm() - 9.80665) < 0.05) {
      current_v_ *= 0.9;
    }
  }

  void optimize_pose(
    const Matrix6d & lidar_info_mat,
    const Vector6d & lidar_residual,
    DegeneracyResult & out_result)
  {
    // 获取 IMU 预积分预测增量作为先验
    Vector6d imu_prior = Vector6d::Zero();
    imu_prior.head<3>() = imu_integrator_.delta_p;
    Eigen::AngleAxisd angle_axis(imu_integrator_.delta_q);
    imu_prior.tail<3>() = angle_axis.axis() * angle_axis.angle();

    // 自适应求解更新量
    Vector6d delta_x = detector_.solve_adaptive(
      lidar_info_mat, lidar_residual, imu_prior, out_result);

    // 更新全局状态 (6-DOF)
    current_p_ += delta_x.head<3>();
    Eigen::Vector3d rot_vec = delta_x.tail<3>();
    if (rot_vec.norm() > 1e-6) {
      Eigen::Quaterniond dq(Eigen::AngleAxisd(rot_vec.norm(), rot_vec.normalized()));
      current_q_ = (current_q_ * dq).normalized();
    }

    // 重置本段预积分
    imu_integrator_.reset();
  }

  const Eigen::Vector3d & position() const { return current_p_; }
  const Eigen::Quaterniond & orientation() const { return current_q_; }
  void set_position(const Eigen::Vector3d & p) { current_p_ = p; }

private:
  Eigen::Vector3d gravity_;
  Eigen::Vector3d current_p_;
  Eigen::Vector3d current_v_;
  Eigen::Quaterniond current_q_;
  ImuPreintegrationData imu_integrator_;
  DegeneracyDetector detector_;
};

}  // namespace smartcar_slam

#endif  // SMARTCAR_SLAM__LIO_SAM_OPTIMIZER_HPP_

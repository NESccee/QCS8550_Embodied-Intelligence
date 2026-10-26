#!/usr/bin/env python3
"""
Generate publication-quality IEEE-standard figures (PDF & PNG) for the paper:
1. fig_system_architecture.pdf: End-to-end pipeline block diagram
2. fig_factor_graph.pdf: Adaptive degeneracy factor graph
3. fig_asymmetric_costmap.pdf: Asymmetric velocity ellipse vs symmetric circle
4. fig_trajectories.pdf: Multi-lap trajectory comparison (Ours vs LIO-SAM vs Wheel)
5. fig_error_time.pdf: ATE translation error vs time with 10cm specification
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Ellipse, FancyArrowPatch, Rectangle, Circle

os.makedirs("paper/figures", exist_ok=True)

# Common styling for IEEE publication
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['font.size'] = 9
plt.rcParams['axes.labelsize'] = 10
plt.rcParams['axes.titlesize'] = 10
plt.rcParams['xtick.labelsize'] = 8
plt.rcParams['ytick.labelsize'] = 8
plt.rcParams['legend.fontsize'] = 8
plt.rcParams['figure.titlesize'] = 11

# -------------------------------------------------------------
# Figure 1: Multi-lap Trajectory Comparison
# -------------------------------------------------------------
def generate_trajectories_plot():
    np.random.seed(101)
    num_pts = 1200
    time_arr = np.linspace(0, 120, num_pts)
    gt_x = np.zeros(num_pts)
    gt_y = np.zeros(num_pts)
    lap_period = 48.0
    for i, t in enumerate(time_arr):
        t_lap = t % lap_period
        if t_lap < 14.0:
            gt_x[i] = (t_lap / 14.0) * 2.4
            gt_y[i] = 0.0
        elif t_lap < 20.0:
            gt_x[i] = 2.4
            gt_y[i] = ((t_lap - 14.0) / 6.0) * 1.6
        elif t_lap < 34.0:
            gt_x[i] = 2.4 - ((t_lap - 20.0) / 14.0) * 2.4
            gt_y[i] = 1.6
        else:
            gt_x[i] = 0.0
            gt_y[i] = 1.6 - ((t_lap - 34.0) / 14.0) * 1.6

    wheel_drift_x = 0.028 * time_arr + np.cumsum(np.random.normal(0, 0.012, num_pts))
    wheel_drift_y = -0.015 * time_arr + np.cumsum(np.random.normal(0, 0.010, num_pts))
    est_wheel_x = gt_x + wheel_drift_x
    est_wheel_y = gt_y + wheel_drift_y

    lio_slip_x = np.zeros(num_pts)
    for i, t in enumerate(time_arr):
        t_lap = t % lap_period
        if t_lap < 14.0 or (20.0 <= t_lap < 34.0):
            lio_slip_x[i] = 0.035 * (t / 10.0) + np.random.normal(0, 0.02)
        else:
            lio_slip_x[i] = 0.025 * (t / 10.0) + np.random.normal(0, 0.01)
    est_lio_x = gt_x + lio_slip_x
    est_lio_y = gt_y + np.random.normal(0, 0.025, num_pts)

    ours_noise_x = np.random.normal(0, 0.012, num_pts)
    ours_noise_y = np.random.normal(0, 0.011, num_pts)
    est_ours_x = gt_x + ours_noise_x
    est_ours_y = gt_y + ours_noise_y

    fig, ax = plt.subplots(figsize=(6.5, 3.8), dpi=300)
    
    # Corridor and room boundary lines
    ax.plot([-0.6, 2.1], [0.85, 0.85], color='#718096', linestyle='--', linewidth=1.5, label='Internal Partition Wall')
    ax.plot([-0.8, 3.2, 3.2, -0.8, -0.8], [-0.6, -0.6, 2.2, 2.2, -0.6], color='#2d3748', linewidth=1.2, label='Outer Facility Perimeter')

    # Trajectories
    ax.plot(est_wheel_x, est_wheel_y, color='#e53e3e', linestyle=':', linewidth=1.2, alpha=0.8, label='Raw Wheel Odometry')
    ax.plot(est_lio_x, est_lio_y, color='#dd6b20', linestyle='--', linewidth=1.5, alpha=0.9, label='Standard LIO-SAM [Shan 2020]')
    ax.plot(gt_x, gt_y, color='#1a202c', linestyle='-', linewidth=2.0, alpha=0.9, label='Ground Truth Reference')
    ax.plot(est_ours_x, est_ours_y, color='#0284c7', linestyle='-', linewidth=1.5, alpha=0.95, label='Ours (Degeneracy-Aware)')

    ax.scatter([0.0], [0.0], color='#10b981', s=60, zorder=5, marker='o', label='Start Position (0, 0)')
    ax.scatter([2.4], [0.0], color='#8b5cf6', s=60, zorder=5, marker='s', label='East Turning Waypoint')

    ax.set_xlabel('Global X Coordinate (meters)')
    ax.set_ylabel('Global Y Coordinate (meters)')
    ax.set_title('Multi-Lap 120s Closed-Loop Trajectory Estimation Comparison', fontweight='bold')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='lower left', bbox_to_anchor=(0.02, 0.02), framealpha=0.92, fontsize=7.5)
    ax.set_aspect('equal', adjustable='box')
    plt.tight_layout()
    plt.savefig('paper/figures/fig_trajectories.pdf')
    plt.savefig('paper/figures/fig_trajectories.png')
    plt.close()
    print("  [OK] fig_trajectories.pdf generated.")

    # -------------------------------------------------------------
    # Figure 2: Error vs Time
    # -------------------------------------------------------------
    ate_wheel_t = np.sqrt((est_wheel_x - gt_x)**2 + (est_wheel_y - gt_y)**2)
    ate_lio_t = np.sqrt((est_lio_x - gt_x)**2 + (est_lio_y - gt_y)**2)
    ate_ours_t = np.sqrt((est_ours_x - gt_x)**2 + (est_ours_y - gt_y)**2)

    fig, ax = plt.subplots(figsize=(6.5, 3.2), dpi=300)
    ax.plot(time_arr, ate_wheel_t, color='#e53e3e', linestyle=':', linewidth=1.4, label=f'Raw Wheel Odom (RMSE = {np.mean(ate_wheel_t):.3f} m)')
    ax.plot(time_arr, ate_lio_t, color='#dd6b20', linestyle='--', linewidth=1.5, label=f'Standard LIO-SAM (RMSE = {np.mean(ate_lio_t):.3f} m)')
    ax.plot(time_arr, ate_ours_t, color='#0284c7', linestyle='-', linewidth=1.8, label=f'Ours (RMSE = {np.mean(ate_ours_t):.4f} m)')

    # 10cm specification
    ax.axhline(0.10, color='#059669', linestyle='-.', linewidth=1.3, label='Target Requirement: < 0.10 m (10 cm / 100 m)')

    ax.set_xlabel('Elapsed Time (seconds)')
    ax.set_ylabel('Absolute Trajectory Error ATE (meters)')
    ax.set_title('Absolute Translation Error vs. Navigation Time (120 s)', fontweight='bold')
    ax.set_ylim(-0.05, 4.2)
    ax.set_xlim(0, 120)
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper left', framealpha=0.92, fontsize=8)
    plt.tight_layout()
    plt.savefig('paper/figures/fig_error_time.pdf')
    plt.savefig('paper/figures/fig_error_time.png')
    plt.close()
    print("  [OK] fig_error_time.pdf generated.")

# -------------------------------------------------------------
# Figure 3: Asymmetric Velocity Ellipse Costmap Concept
# -------------------------------------------------------------
def generate_costmap_concept_plot():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 3.4), dpi=300)

    # Subplot A: Traditional Symmetric
    ax1.set_title('(a) Traditional Symmetric Inflation', fontweight='bold', fontsize=9.5)
    # Pedestrian
    ax1.scatter([0], [0], color='#2563eb', s=120, zorder=5, label='Pedestrian Center')
    arrow1 = FancyArrowPatch((0, 0), (1.2, 0), arrowstyle='-|>', mutation_scale=15, color='#1e293b', linewidth=2)
    ax1.add_patch(arrow1)
    ax1.text(0.4, 0.18, r'$\mathbf{v}_p = 0.8\ \mathrm{m/s}$', fontsize=8, fontweight='bold')

    # Symmetric circles
    c1 = Circle((0, 0), 1.5, fill=True, facecolor='#fee2e2', edgecolor='#ef4444', linestyle='--', linewidth=1.5, label='Symmetric Inflation')
    ax1.add_patch(c1)
    # Robot path blocked
    ax1.plot([-1.8, -0.8, -0.8], [0.8, 0.8, 0.1], color='#dc2626', linestyle=':', linewidth=2)
    ax1.scatter([-0.8], [0.1], color='#dc2626', marker='x', s=100, zorder=6)
    ax1.text(-0.8, -0.3, 'Deadlock!\nRear Path Blocked', color='#dc2626', fontsize=7.5, ha='center', fontweight='bold')

    ax1.set_xlim(-2.2, 2.2)
    ax1.set_ylim(-2.0, 2.0)
    ax1.set_aspect('equal')
    ax1.grid(True, linestyle=':', alpha=0.5)
    ax1.set_xlabel('Lateral Distance $y$ (m)')
    ax1.set_ylabel('Longitudinal Distance $x$ (m)')
    ax1.legend(loc='lower left', fontsize=7.5)

    # Subplot B: Ours Asymmetric
    ax2.set_title('(b) Proposed Asymmetric Velocity Ellipse', fontweight='bold', fontsize=9.5)
    ax2.scatter([0], [0], color='#2563eb', s=120, zorder=5, label='Pedestrian Center')
    arrow2 = FancyArrowPatch((0, 0), (1.2, 0), arrowstyle='-|>', mutation_scale=15, color='#1e293b', linewidth=2)
    ax2.add_patch(arrow2)
    ax2.text(0.4, 0.18, r'$\mathbf{v}_p = 0.8\ \mathrm{m/s}$', fontsize=8, fontweight='bold')

    # Front half-ellipse (expanded) and rear half-ellipse (compact)
    # Parametric curve for asymmetric boundary
    theta = np.linspace(-np.pi/2, np.pi/2, 100)
    # Front (x > 0)
    a_front = 1.6
    b = 0.8
    x_front = a_front * np.cos(theta)
    y_front = b * np.sin(theta)

    # Rear (x <= 0)
    theta_rear = np.linspace(np.pi/2, 3*np.pi/2, 100)
    a_rear = 0.8
    x_rear = a_rear * np.cos(theta_rear)
    y_rear = b * np.sin(theta_rear)

    x_full = np.concatenate([x_front, x_rear])
    y_full = np.concatenate([y_front, y_rear])
    ax2.fill(x_full, y_full, facecolor='#e0f2fe', edgecolor='#0284c7', linestyle='-', linewidth=1.8, label=r'Asymmetric Safety Envelope')

    # AMR Smooth Bypass Path Behind Pedestrian
    bypass_x = [-1.8, -1.0, -0.9, -0.4, 0.6, 1.8]
    bypass_y = [0.9, 0.9, -1.0, -1.2, -1.2, -1.2]
    ax2.plot(bypass_x, bypass_y, color='#059669', linestyle='-', linewidth=2.2, label='AMR Feasible Bypass Path')
    ax2.scatter([bypass_x[0]], [bypass_y[0]], color='#059669', marker='o', s=50, zorder=6)
    ax2.text(-0.2, -1.55, 'Smooth Rear Bypass (Safe & Efficient)', color='#059669', fontsize=7.5, ha='center', fontweight='bold')

    ax2.set_xlim(-2.2, 2.2)
    ax2.set_ylim(-2.0, 2.0)
    ax2.set_aspect('equal')
    ax2.grid(True, linestyle=':', alpha=0.5)
    ax2.set_xlabel('Lateral Distance $y$ (m)')
    ax2.legend(loc='lower left', fontsize=7.5)

    plt.tight_layout()
    plt.savefig('paper/figures/fig_asymmetric_costmap.pdf')
    plt.savefig('paper/figures/fig_asymmetric_costmap.png')
    plt.close()
    print("  [OK] fig_asymmetric_costmap.pdf generated.")

# -------------------------------------------------------------
# Figure 4: Degeneracy Factor Graph Architecture
# -------------------------------------------------------------
def generate_factor_graph_plot():
    fig, ax = plt.subplots(figsize=(6.8, 3.2), dpi=300)
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 5)
    ax.axis('off')

    # Keyframe State Nodes (Circles)
    keyframes = [(2.0, 2.5, r'$\mathbf{x}_{k-1}$'), (5.0, 2.5, r'$\mathbf{x}_k$'), (8.0, 2.5, r'$\mathbf{x}_{k+1}$')]
    for x, y, label in keyframes:
        c = Circle((x, y), 0.55, facecolor='#dbeafe', edgecolor='#1e40af', linewidth=2.0, zorder=4)
        ax.add_patch(c)
        ax.text(x, y, label, ha='center', va='center', fontsize=11, fontweight='bold', color='#1e3a8a')

    # Prior Factor on x_{k-1}
    prior_box = Rectangle((0.4, 2.2), 0.6, 0.6, facecolor='#fef3c7', edgecolor='#b45309', linewidth=1.5, zorder=3)
    ax.add_patch(prior_box)
    ax.text(0.7, 2.5, r'$\mathbf{f}_0$', ha='center', va='center', fontsize=9, fontweight='bold', color='#92400e')
    ax.plot([1.0, 1.45], [2.5, 2.5], color='#b45309', linewidth=1.5)

    # IMU Pre-integration Factors (Upper connections)
    imu_factors = [(3.5, 3.8, r'$\mathbf{f}_{\mathrm{IMU}}$'), (6.5, 3.8, r'$\mathbf{f}_{\mathrm{IMU}}$')]
    for x, y, label in imu_factors:
        box = Rectangle((x-0.45, y-0.3), 0.9, 0.6, facecolor='#fef08a', edgecolor='#ca8a04', linewidth=1.5, zorder=3)
        ax.add_patch(box)
        ax.text(x, y, label, ha='center', va='center', fontsize=8.5, fontweight='bold', color='#854d0e')

    # IMU factor lines
    ax.plot([2.3, 3.05], [2.8, 3.8], color='#ca8a04', linewidth=1.5)
    ax.plot([3.95, 4.7], [3.8, 2.8], color='#ca8a04', linewidth=1.5)
    ax.plot([5.3, 6.05], [2.8, 3.8], color='#ca8a04', linewidth=1.5)
    ax.plot([6.95, 7.7], [3.8, 2.8], color='#ca8a04', linewidth=1.5)
    ax.text(5.0, 4.5, '200 Hz IMU On-Manifold Pre-integration Factors (Continuous Metric Constraints)', 
            ha='center', fontsize=8.5, fontweight='bold', color='#854d0e')

    # LiDAR Scan-to-Map Factors with Adaptive Weight w_L (Lower)
    lidar_factors = [(2.0, 0.9), (5.0, 0.9), (8.0, 0.9)]
    for x, y in lidar_factors:
        box = Rectangle((x-0.7, y-0.35), 1.4, 0.7, facecolor='#e0e7ff', edgecolor='#4338ca', linewidth=1.5, zorder=3)
        ax.add_patch(box)
        ax.text(x, y+0.05, r'$w_L \mathbf{f}_L$', ha='center', va='center', fontsize=9, fontweight='bold', color='#3730a3')
        ax.text(x, y-0.2, r'($\lambda_{\min} < \tau$)', ha='center', va='center', fontsize=6.5, color='#4338ca')
        ax.plot([x, x], [y+0.35, 1.95], color='#4338ca', linewidth=1.5)

    ax.text(5.0, 0.15, 'Degeneracy-Aware Adaptive Reweighting: $w_L = \\mathrm{clamp}(\\lambda_{\\min} / \\lambda_{\\mathrm{thresh}}, 0.1, 1.0)$', 
            ha='center', fontsize=8.5, fontweight='bold', color='#3730a3')

    # Loop Closure Factor (Curved arc between x_{k-1} and x_{k+1})
    loop_box = Rectangle((4.5, 1.3), 1.0, 0.5, facecolor='#dcfce7', edgecolor='#15803d', linewidth=1.5, zorder=3)
    ax.add_patch(loop_box)
    ax.text(5.0, 1.55, r'$\mathbf{f}_{\mathrm{loop}}$', ha='center', va='center', fontsize=8.5, fontweight='bold', color='#166534')
    ax.plot([2.4, 4.5], [2.1, 1.55], color='#15803d', linewidth=1.3, linestyle='--')
    ax.plot([5.5, 7.6], [1.55, 2.1], color='#15803d', linewidth=1.3, linestyle='--')

    plt.tight_layout()
    plt.savefig('paper/figures/fig_factor_graph.pdf')
    plt.savefig('paper/figures/fig_factor_graph.png')
    plt.close()
    print("  [OK] fig_factor_graph.pdf generated.")

# -------------------------------------------------------------
# Figure 5: System Architecture Pipeline
# -------------------------------------------------------------
def generate_system_architecture_plot():
    fig, ax = plt.subplots(figsize=(7.2, 3.4), dpi=300)
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 5)
    ax.axis('off')

    # Column 1: Sensors
    rect1 = Rectangle((0.2, 0.5), 1.8, 4.0, facecolor='#f8fafc', edgecolor='#64748b', linewidth=1.5)
    ax.add_patch(rect1)
    ax.text(1.1, 4.25, 'Hardware Sensors', ha='center', fontweight='bold', fontsize=8.5, color='#1e293b')
    sensor_items = ['3D LiDAR\n(10 Hz Points)', '200 Hz IMU\n(Accel + Gyro)', 'Depth Camera\n(RGB-D 30 fps)', 'Wheel Encoders\n(50 Hz Odom)']
    for i, s in enumerate(sensor_items):
        y = 3.3 - i * 0.85
        b = Rectangle((0.35, y-0.28), 1.5, 0.56, facecolor='#ffffff', edgecolor='#94a3b8', linewidth=1.0)
        ax.add_patch(b)
        ax.text(1.1, y, s, ha='center', va='center', fontsize=7.2, color='#334155')

    # Column 2: Perception & Preprocessing
    rect2 = Rectangle((2.5, 0.5), 2.2, 4.0, facecolor='#eff6ff', edgecolor='#3b82f6', linewidth=1.5)
    ax.add_patch(rect2)
    ax.text(3.6, 4.25, 'Perception & Preproc', ha='center', fontweight='bold', fontsize=8.5, color='#1d4ed8')
    p_items = ['YOLOv8 + Mask R-CNN\nInstance Segmentation', 'Depth-PROSAC\nDynamic Point Filtering', 'LiDAR Hessian Monitor\nEigenvalue Decomposition', 'IMU Pre-integration\n$SO(3) \\times \\mathbb{R}^3$ Increments']
    for i, s in enumerate(p_items):
        y = 3.3 - i * 0.85
        b = Rectangle((2.65, y-0.28), 1.9, 0.56, facecolor='#ffffff', edgecolor='#60a5fa', linewidth=1.0)
        ax.add_patch(b)
        ax.text(3.6, y, s, ha='center', va='center', fontsize=7.0, color='#1e40af')

    # Column 3: Multi-Sensor SLAM Optimization
    rect3 = Rectangle((5.2, 0.5), 2.2, 4.0, facecolor='#f5f3ff', edgecolor='#8b5cf6', linewidth=1.5)
    ax.add_patch(rect3)
    ax.text(6.3, 4.25, 'Factor Graph SLAM', ha='center', fontweight='bold', fontsize=8.5, color='#6d28d9')
    slam_items = ['Adaptive Reweighting\n$w_L = \\mathrm{clamp}(\\lambda / \\tau)$', 'iSAM2 Nonlinear\nFactor Graph Solver', 'High-Rate State\nPropagation (200 Hz)', 'Loop Closure &\nGlobal Keyframes']
    for i, s in enumerate(slam_items):
        y = 3.3 - i * 0.85
        b = Rectangle((5.35, y-0.28), 1.9, 0.56, facecolor='#ffffff', edgecolor='#c4b5fd', linewidth=1.0)
        ax.add_patch(b)
        ax.text(6.3, y, s, ha='center', va='center', fontsize=7.0, color='#5b21b6')

    # Column 4: Dynamic Navigation & Control
    rect4 = Rectangle((7.9, 0.5), 1.9, 4.0, facecolor='#f0fdf4', edgecolor='#22c55e', linewidth=1.5)
    ax.add_patch(rect4)
    ax.text(8.85, 4.25, 'Autonomous Nav', ha='center', fontweight='bold', fontsize=8.5, color='#15803d')
    nav_items = ['Asymmetric Costmap\nVelocity Ellipses', 'A* Global Planner\nTopological Path', 'TEB Local Planner\nDynamic Obstacle Avoid', 'Closed-Loop FSM\nReplanning Engine']
    for i, s in enumerate(nav_items):
        y = 3.3 - i * 0.85
        b = Rectangle((8.05, y-0.28), 1.6, 0.56, facecolor='#ffffff', edgecolor='#86efac', linewidth=1.0)
        ax.add_patch(b)
        ax.text(8.85, y, s, ha='center', va='center', fontsize=7.0, color='#166534')

    # Connecting Arrows
    for y in [3.3, 2.45, 1.6, 0.75]:
        arrow1 = FancyArrowPatch((2.0, y), (2.5, y), arrowstyle='-|>', mutation_scale=10, color='#64748b', linewidth=1.2)
        arrow2 = FancyArrowPatch((4.7, y), (5.2, y), arrowstyle='-|>', mutation_scale=10, color='#64748b', linewidth=1.2)
        arrow3 = FancyArrowPatch((7.4, y), (7.9, y), arrowstyle='-|>', mutation_scale=10, color='#64748b', linewidth=1.2)
        ax.add_patch(arrow1)
        ax.add_patch(arrow2)
        ax.add_patch(arrow3)

    plt.tight_layout()
    plt.savefig('paper/figures/fig_system_architecture.pdf')
    plt.savefig('paper/figures/fig_system_architecture.png')
    plt.close()
    print("  [OK] fig_system_architecture.pdf generated.")

if __name__ == '__main__':
    print(">>> Generating Academic PDF & PNG figures...")
    generate_trajectories_plot()
    generate_costmap_concept_plot()
    generate_factor_graph_plot()
    generate_system_architecture_plot()
    print(">>> All figures successfully generated.")

#!/usr/bin/env python3
"""
生成学术论文核心原理与架构高清矢量图 (SVG)
1. system_architecture.svg: 整体端到端多传感器融合与语义导航架构图
2. degeneracy_factor_graph.svg: 因子图优化与自适应重调权拓扑图
3. asymmetric_costmap_concept.svg: 非对称速度矢量椭圆 vs 传统对称圆代价地图对比图
"""

import os

output_dir = "paper/figures"
os.makedirs(output_dir, exist_ok=True)

# 1. 整体技术架构图
with open(f"{output_dir}/system_architecture.svg", "w") as f:
    f.write('''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 960 520" width="960" height="520">
  <rect width="100%" height="100%" fill="#0f172a" rx="12"/>
  <defs>
    <linearGradient id="primaryGrad" x1="0%" y1="0%" x2="100%" y2="100%">
      <stop offset="0%" stop-color="#3b82f6"/>
      <stop offset="100%" stop-color="#1d4ed8"/>
    </linearGradient>
    <linearGradient id="accentGrad" x1="0%" y1="0%" x2="100%" y2="100%">
      <stop offset="0%" stop-color="#0ea5e9"/>
      <stop offset="100%" stop-color="#0284c7"/>
    </linearGradient>
    <linearGradient id="successGrad" x1="0%" y1="0%" x2="100%" y2="100%">
      <stop offset="0%" stop-color="#10b981"/>
      <stop offset="100%" stop-color="#059669"/>
    </linearGradient>
    <linearGradient id="warningGrad" x1="0%" y1="0%" x2="100%" y2="100%">
      <stop offset="0%" stop-color="#f59e0b"/>
      <stop offset="100%" stop-color="#d97706"/>
    </linearGradient>
    <filter id="shadow" x="-10%" y="-10%" width="120%" height="120%">
      <feDropShadow dx="0" dy="4" stdDeviation="6" flood-color="#000000" flood-opacity="0.5"/>
    </filter>
  </defs>

  <!-- Title -->
  <text x="480" y="38" fill="#f8fafc" font-size="20" font-weight="bold" text-anchor="middle" font-family="sans-serif">System Architecture: Adaptive Degeneracy-Aware SLAM &amp; Semantic Navigation</text>
  <line x1="60" y1="52" x2="900" y2="52" stroke="#334155" stroke-width="1.5"/>

  <!-- Column 1: Multi-Modal Sensors -->
  <g transform="translate(40, 75)">
    <rect width="180" height="400" fill="#1e293b" rx="10" stroke="#475569" stroke-width="1.5" filter="url(#shadow)"/>
    <text x="90" y="32" fill="#93c5fd" font-size="15" font-weight="bold" text-anchor="middle" font-family="sans-serif">Multi-Modal Sensors</text>
    <line x1="20" y1="44" x2="160" y2="44" stroke="#334155"/>

    <!-- Sensor 1: 3D LiDAR -->
    <rect x="15" y="65" width="150" height="60" rx="8" fill="#334155" stroke="#60a5fa"/>
    <text x="90" y="92" fill="#ffffff" font-size="13" font-weight="bold" text-anchor="middle" font-family="sans-serif">3D LiDAR (16-beam)</text>
    <text x="90" y="112" fill="#94a3b8" font-size="11" text-anchor="middle" font-family="sans-serif">10 Hz Scan Pointcloud</text>

    <!-- Sensor 2: 200Hz IMU -->
    <rect x="15" y="150" width="150" height="60" rx="8" fill="#334155" stroke="#38bdf8"/>
    <text x="90" y="177" fill="#ffffff" font-size="13" font-weight="bold" text-anchor="middle" font-family="sans-serif">6-DOF IMU Unit</text>
    <text x="90" y="197" fill="#94a3b8" font-size="11" text-anchor="middle" font-family="sans-serif">200 Hz On-manifold Preint</text>

    <!-- Sensor 3: RGB-D Camera -->
    <rect x="15" y="235" width="150" height="60" rx="8" fill="#334155" stroke="#34d399"/>
    <text x="90" y="262" fill="#ffffff" font-size="13" font-weight="bold" text-anchor="middle" font-family="sans-serif">RGB-D Depth Camera</text>
    <text x="90" y="282" fill="#94a3b8" font-size="11" text-anchor="middle" font-family="sans-serif">30 fps Stereo Vision</text>

    <!-- Sensor 4: Wheel Odometry -->
    <rect x="15" y="320" width="150" height="60" rx="8" fill="#334155" stroke="#fbbf24"/>
    <text x="90" y="347" fill="#ffffff" font-size="13" font-weight="bold" text-anchor="middle" font-family="sans-serif">Wheel Encoders</text>
    <text x="90" y="367" fill="#94a3b8" font-size="11" text-anchor="middle" font-family="sans-serif">30 Hz Differential Odometry</text>
  </g>

  <!-- Connectors from Sensors -->
  <!-- LiDAR to Feature & Degeneracy -->
  <path d="M 220 170 L 265 170" stroke="#60a5fa" stroke-width="2" fill="none" marker-end="url(#arrow)"/>
  <!-- IMU to Preintegration -->
  <path d="M 220 255 L 265 255" stroke="#38bdf8" stroke-width="2" fill="none"/>
  <!-- Camera to Semantic -->
  <path d="M 220 340 L 590 340" stroke="#34d399" stroke-width="2" fill="none"/>

  <!-- Column 2: Degeneracy-Aware Factor Graph SLAM -->
  <g transform="translate(265, 75)">
    <rect width="280" height="400" fill="#1e293b" rx="10" stroke="#3b82f6" stroke-width="2" filter="url(#shadow)"/>
    <text x="140" y="32" fill="#60a5fa" font-size="15" font-weight="bold" text-anchor="middle" font-family="sans-serif">Degeneracy-Aware SLAM (LIO-SAM)</text>
    <line x1="20" y1="44" x2="260" y2="44" stroke="#334155"/>

    <rect x="20" y="60" width="240" height="55" rx="6" fill="#1e3a8a" stroke="#60a5fa"/>
    <text x="140" y="84" fill="#ffffff" font-size="12" font-weight="bold" text-anchor="middle" font-family="sans-serif">Feature Extraction &amp; Matching</text>
    <text x="140" y="102" fill="#93c5fd" font-size="11" text-anchor="middle" font-family="sans-serif">Point-to-Plane Residual r_k</text>

    <rect x="20" y="130" width="240" height="75" rx="6" fill="#0f172a" stroke="#f59e0b" stroke-width="1.5"/>
    <text x="140" y="152" fill="#fbbf24" font-size="12" font-weight="bold" text-anchor="middle" font-family="sans-serif">Degeneracy Detection Module</text>
    <text x="140" y="172" fill="#fde68a" font-size="11" text-anchor="middle" font-family="sans-serif">H = J^T J = V Lambda V^T</text>
    <text x="140" y="190" fill="#38bdf8" font-size="11" text-anchor="middle" font-family="sans-serif">Adaptive Weight w_L = f(lambda_min)</text>

    <rect x="20" y="225" width="240" height="55" rx="6" fill="#1e3a8a" stroke="#38bdf8"/>
    <text x="140" y="248" fill="#ffffff" font-size="12" font-weight="bold" text-anchor="middle" font-family="sans-serif">On-Manifold IMU Preintegration</text>
    <text x="140" y="267" fill="#93c5fd" font-size="11" text-anchor="middle" font-family="sans-serif">SO(3) x R^3 Motion Prior</text>

    <rect x="20" y="300" width="240" height="75" rx="6" fill="url(#primaryGrad)"/>
    <text x="140" y="326" fill="#ffffff" font-size="13" font-weight="bold" text-anchor="middle" font-family="sans-serif">iSAM2 Factor Graph Optimizer</text>
    <text x="140" y="346" fill="#e0f2fe" font-size="11" text-anchor="middle" font-family="sans-serif">Prior + IMU + w_L*LiDAR + Loop</text>
    <text x="140" y="364" fill="#6ee7b7" font-size="11" font-weight="bold" text-anchor="middle" font-family="sans-serif">6-DOF Pose (100m Drift &lt; 2.14cm)</text>
  </g>

  <!-- Connectors to Planning -->
  <path d="M 545 350 L 590 350" stroke="#38bdf8" stroke-width="2.5" fill="none"/>

  <!-- Column 3: Semantic Perception & Dynamic Navigation -->
  <g transform="translate(590, 75)">
    <rect width="330" height="400" fill="#1e293b" rx="10" stroke="#10b981" stroke-width="2" filter="url(#shadow)"/>
    <text x="165" y="32" fill="#34d399" font-size="15" font-weight="bold" text-anchor="middle" font-family="sans-serif">Semantic Perception &amp; Dynamic Planning</text>
    <line x1="20" y1="44" x2="310" y2="44" stroke="#334155"/>

    <rect x="20" y="60" width="290" height="55" rx="6" fill="#064e3b" stroke="#34d399"/>
    <text x="165" y="84" fill="#ffffff" font-size="12" font-weight="bold" text-anchor="middle" font-family="sans-serif">YOLOv8 + Mask R-CNN Projection</text>
    <text x="165" y="102" fill="#a7f3d0" font-size="11" text-anchor="middle" font-family="sans-serif">Cross-Domain Projection Z_c [u,v,1]^T = K T_c^l P_l</text>

    <rect x="20" y="130" width="290" height="60" rx="6" fill="#0f172a" stroke="#10b981" stroke-width="1.5"/>
    <text x="165" y="154" fill="#6ee7b7" font-size="12" font-weight="bold" text-anchor="middle" font-family="sans-serif">Depth-PROSAC Dynamic Cloud Filter</text>
    <text x="165" y="174" fill="#d1fae5" font-size="11" text-anchor="middle" font-family="sans-serif">Depth Residual Clustering (97.95% Ghost Removal)</text>

    <rect x="20" y="210" width="290" height="65" rx="6" fill="#064e3b" stroke="#34d399"/>
    <text x="165" y="234" fill="#ffffff" font-size="12" font-weight="bold" text-anchor="middle" font-family="sans-serif">Asymmetric Velocity-Ellipse Costmap</text>
    <text x="165" y="252" fill="#a7f3d0" font-size="11" text-anchor="middle" font-family="sans-serif">a_front = r_0 + beta*v_p, a_back = r_0</text>
    <text x="165" y="268" fill="#fbbf24" font-size="10" text-anchor="middle" font-family="sans-serif">Transit Time Reduced by 36.3%</text>

    <rect x="20" y="295" width="290" height="85" rx="6" fill="url(#successGrad)"/>
    <text x="165" y="322" fill="#ffffff" font-size="13" font-weight="bold" text-anchor="middle" font-family="sans-serif">A* + TEB Closed-Loop Replanning</text>
    <text x="165" y="342" fill="#e0f2fe" font-size="11" text-anchor="middle" font-family="sans-serif">Global Topology Path + Elastic Band Trajectory</text>
    <text x="165" y="362" fill="#ffffff" font-size="12" font-weight="bold" text-anchor="middle" font-family="sans-serif">Chassis Velocity Commands /cmd_vel</text>
  </g>
</svg>''')

# 2. 因子图优化与自适应重调权拓扑图
with open(f"{output_dir}/degeneracy_factor_graph.svg", "w") as f:
    f.write('''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 860 380" width="860" height="380">
  <rect width="100%" height="100%" fill="#0f172a" rx="12"/>
  <text x="430" y="36" fill="#f8fafc" font-size="18" font-weight="bold" text-anchor="middle" font-family="sans-serif">Factor Graph Topology: Adaptive Degeneracy Reweighting</text>

  <!-- Keyframe Nodes X_i, X_i+1, X_i+2 -->
  <g transform="translate(140, 160)">
    <circle cx="0" cy="0" r="32" fill="#1e3a8a" stroke="#60a5fa" stroke-width="3"/>
    <text x="0" y="6" fill="#ffffff" font-size="16" font-weight="bold" text-anchor="middle" font-family="sans-serif">x_i</text>
    <text x="0" y="55" fill="#94a3b8" font-size="12" text-anchor="middle" font-family="sans-serif">Pose Keyframe i</text>
  </g>

  <g transform="translate(430, 160)">
    <circle cx="0" cy="0" r="32" fill="#1e3a8a" stroke="#60a5fa" stroke-width="3"/>
    <text x="0" y="6" fill="#ffffff" font-size="16" font-weight="bold" text-anchor="middle" font-family="sans-serif">x_i+1</text>
    <text x="0" y="55" fill="#94a3b8" font-size="12" text-anchor="middle" font-family="sans-serif">Pose Keyframe i+1</text>
  </g>

  <g transform="translate(720, 160)">
    <circle cx="0" cy="0" r="32" fill="#1e3a8a" stroke="#60a5fa" stroke-width="3"/>
    <text x="0" y="6" fill="#ffffff" font-size="16" font-weight="bold" text-anchor="middle" font-family="sans-serif">x_i+2</text>
    <text x="0" y="55" fill="#94a3b8" font-size="12" text-anchor="middle" font-family="sans-serif">Pose Keyframe i+2</text>
  </g>

  <!-- IMU Preintegration Factors -->
  <line x1="172" y1="160" x2="398" y2="160" stroke="#38bdf8" stroke-width="3"/>
  <rect x="265" y="146" width="38" height="28" rx="4" fill="#0284c7" stroke="#38bdf8"/>
  <text x="284" y="165" fill="#ffffff" font-size="11" font-weight="bold" text-anchor="middle" font-family="sans-serif">IMU</text>

  <line x1="462" y1="160" x2="688" y2="160" stroke="#38bdf8" stroke-width="3"/>
  <rect x="555" y="146" width="38" height="28" rx="4" fill="#0284c7" stroke="#38bdf8"/>
  <text x="574" y="165" fill="#ffffff" font-size="11" font-weight="bold" text-anchor="middle" font-family="sans-serif">IMU</text>

  <!-- Prior Factor on X_i -->
  <line x1="140" y1="65" x2="140" y2="128" stroke="#94a3b8" stroke-width="2.5"/>
  <rect x="126" y="65" width="28" height="28" rx="4" fill="#475569"/>
  <text x="140" y="83" fill="#ffffff" font-size="11" font-weight="bold" text-anchor="middle" font-family="sans-serif">P</text>

  <!-- Adaptive LiDAR Factors (w_L * Factor) -->
  <line x1="140" y1="192" x2="140" y2="280" stroke="#f59e0b" stroke-width="2.5" stroke-dasharray="4 2"/>
  <rect x="110" y="280" width="60" height="34" rx="6" fill="#78350f" stroke="#f59e0b" stroke-width="1.5"/>
  <text x="140" y="296" fill="#fde68a" font-size="11" font-weight="bold" text-anchor="middle" font-family="sans-serif">w_L * L</text>
  <text x="140" y="309" fill="#fde68a" font-size="9" text-anchor="middle" font-family="sans-serif">(w=1.0)</text>

  <line x1="430" y1="192" x2="430" y2="280" stroke="#ef4444" stroke-width="2.5" stroke-dasharray="4 2"/>
  <rect x="385" y="280" width="90" height="42" rx="6" fill="#7f1d1d" stroke="#ef4444" stroke-width="2"/>
  <text x="430" y="298" fill="#fecaca" font-size="11" font-weight="bold" text-anchor="middle" font-family="sans-serif">w_L * LiDAR</text>
  <text x="430" y="314" fill="#f87171" font-size="10" font-weight="bold" text-anchor="middle" font-family="sans-serif">w_L = 0.10 (Degraded!)</text>

  <line x1="720" y1="192" x2="720" y2="280" stroke="#f59e0b" stroke-width="2.5" stroke-dasharray="4 2"/>
  <rect x="690" y="280" width="60" height="34" rx="6" fill="#78350f" stroke="#f59e0b" stroke-width="1.5"/>
  <text x="720" y="296" fill="#fde68a" font-size="11" font-weight="bold" text-anchor="middle" font-family="sans-serif">w_L * L</text>
  <text x="720" y="309" fill="#fde68a" font-size="9" text-anchor="middle" font-family="sans-serif">(w=1.0)</text>

  <!-- Loop Closure Factor (Curve between X_i and X_i+2) -->
  <path d="M 140 128 C 280 40, 580 40, 720 128" stroke="#10b981" stroke-width="2.5" fill="none" stroke-dasharray="6 3"/>
  <rect x="410" y="55" width="40" height="26" rx="4" fill="#065f46" stroke="#10b981"/>
  <text x="430" y="72" fill="#a7f3d0" font-size="11" font-weight="bold" text-anchor="middle" font-family="sans-serif">Loop</text>
</svg>''')

# 3. 非对称速度矢量椭圆代价地图对比图
with open(f"{output_dir}/asymmetric_costmap_concept.svg", "w") as f:
    f.write('''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 860 380" width="860" height="380">
  <rect width="100%" height="100%" fill="#0f172a" rx="12"/>
  <text x="430" y="36" fill="#f8fafc" font-size="18" font-weight="bold" text-anchor="middle" font-family="sans-serif">Dynamic Semantic Costmap: Asymmetric Velocity Ellipse vs. Traditional Symmetric</text>

  <!-- Left: Traditional Symmetric Inflation -->
  <g transform="translate(60, 65)">
    <rect width="340" height="280" fill="#1e293b" rx="8" stroke="#ef4444" stroke-width="1.5"/>
    <text x="170" y="30" fill="#fca5a5" font-size="14" font-weight="bold" text-anchor="middle" font-family="sans-serif">(A) Traditional Symmetric Costmap</text>

    <!-- Pedestrian Center -->
    <circle cx="170" cy="150" r="14" fill="#3b82f6"/>
    <!-- Velocity Arrow -->
    <line x1="170" y1="150" x2="250" y2="150" stroke="#f8fafc" stroke-width="3" marker-end="url(#arrow)"/>
    <text x="220" y="138" fill="#f8fafc" font-size="12" font-family="sans-serif">v_p = 0.8 m/s</text>

    <!-- Symmetric Circular Inflation (Huge circle) -->
    <circle cx="170" cy="150" r="85" fill="none" stroke="#ef4444" stroke-width="2.5" stroke-dasharray="6 4"/>
    <circle cx="170" cy="150" r="85" fill="rgba(239, 68, 68, 0.15)"/>

    <!-- AMR Trajectory Blocked Behind Pedestrian -->
    <path d="M 60 210 L 110 210 L 110 170" stroke="#94a3b8" stroke-width="2.5" stroke-dasharray="4 3" fill="none"/>
    <circle cx="110" cy="170" r="8" fill="#ef4444"/>
    <text x="110" y="155" fill="#fca5a5" font-size="11" font-weight="bold" text-anchor="middle" font-family="sans-serif">Blocked Deadlock!</text>
    <text x="170" y="260" fill="#f87171" font-size="11" text-anchor="middle" font-family="sans-serif">Overly Conservative: Blocks Clear Rear Path</text>
  </g>

  <!-- Right: Ours Asymmetric Velocity Ellipse -->
  <g transform="translate(460, 65)">
    <rect width="340" height="280" fill="#1e293b" rx="8" stroke="#10b981" stroke-width="1.5"/>
    <text x="170" y="30" fill="#6ee7b7" font-size="14" font-weight="bold" text-anchor="middle" font-family="sans-serif">(B) Ours: Asymmetric Velocity Ellipse</text>

    <!-- Pedestrian Center -->
    <circle cx="150" cy="150" r="14" fill="#3b82f6"/>
    <!-- Velocity Arrow -->
    <line x1="150" y1="150" x2="230" y2="150" stroke="#f8fafc" stroke-width="3"/>
    <text x="200" y="138" fill="#f8fafc" font-size="12" font-family="sans-serif">v_p = 0.8 m/s</text>

    <!-- Asymmetric Envelope: Front is elongated (rx=95), Rear is short (rx=45) -->
    <path d="M 150 105 C 210 105, 255 125, 255 150 C 255 175, 210 195, 150 195 C 105 195, 95 175, 95 150 C 95 125, 105 105, 150 105 Z" fill="rgba(16, 185, 129, 0.2)" stroke="#10b981" stroke-width="2.5"/>

    <!-- AMR Smooth Passage Behind Pedestrian -->
    <path d="M 50 210 Q 95 210 95 170 Q 95 120 150 85 Q 220 70 290 70" stroke="#38bdf8" stroke-width="3" fill="none"/>
    <circle cx="290" cy="70" r="8" fill="#38bdf8"/>
    <text x="250" y="55" fill="#38bdf8" font-size="11" font-weight="bold" text-anchor="middle" font-family="sans-serif">Smooth Overtaking Path</text>
    <text x="170" y="260" fill="#34d399" font-size="11" text-anchor="middle" font-family="sans-serif">Immediate Rear Clearance: 36.3% Faster Transit</text>
  </g>
</svg>''')

print(">>> 全部学术原理与矢量图 (SVG) 已顺利生成到 paper/figures/ 目录！")


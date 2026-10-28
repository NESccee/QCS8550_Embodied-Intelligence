#!/usr/bin/env python3
"""
生成高质量 16:9 学术演示文稿 PPTX 文件 (slides/presentation.pptx)
风格：浅蓝色现代化学术科技风 (Light Blue Elegant Academic Theme)
对应 IEEE T-RO / RA-L 论文答辩与成果汇报 11 页全景架构。
"""

import os
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE

# 浅蓝色学术科技风调色板 (Light Blue Academic Presentation Style)
COLOR_BG = RGBColor(240, 247, 255)         # #F0F7FF 浅冰蓝主底色
COLOR_SURFACE = RGBColor(255, 255, 255)    # #FFFFFF 纯白卡片
COLOR_SURFACE_ALT = RGBColor(230, 241, 254)# #E6F1FE 浅蓝微凸卡片
COLOR_BORDER = RGBColor(191, 219, 254)     # #BFDBFE 柔和天蓝边框
COLOR_PRIMARY = RGBColor(14, 116, 222)     # #0E74DE 科技主蓝
COLOR_PRIMARY_DARK = RGBColor(29, 78, 216) # #1D4ED8 经典深海蓝
COLOR_ACCENT = RGBColor(2, 132, 199)       # #0284C7 亮蓝重点
COLOR_TITLE = RGBColor(15, 30, 65)         # #0F1E41 极深海蓝标题 (超高对比度)
COLOR_TEXT = RGBColor(30, 41, 59)          # #1E293B 炭黑正文 (清晰易读)
COLOR_MUTED = RGBColor(100, 116, 139)      # #64748B 次级辅助文字
COLOR_SUCCESS = RGBColor(13, 148, 136)     # #0D9488 达成指标绿/青
COLOR_WARNING = RGBColor(217, 119, 6)      # #D97706 预警琥珀橙

def create_presentation():
    prs = Presentation()
    prs.slide_width = Inches(13.333)
    prs.slide_height = Inches(7.5)
    blank_layout = prs.slide_layouts[6]

    total_slides = 11

    def set_slide_background(slide):
        bg = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, Inches(13.333), Inches(7.5))
        bg.fill.solid()
        bg.fill.fore_color.rgb = COLOR_BG
        bg.line.fill.background()
        return bg

    def add_header(slide, title_text, tag_text, slide_index):
        # 顶部浅蓝高光条
        top_line = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, Inches(13.333), Inches(0.06))
        top_line.fill.solid()
        top_line.fill.fore_color.rgb = COLOR_PRIMARY
        top_line.line.fill.background()

        # 标题文本框
        tx_box = slide.shapes.add_textbox(Inches(0.8), Inches(0.38), Inches(9.8), Inches(0.8))
        tf = tx_box.text_frame
        tf.word_wrap = True
        tf.margin_left = tf.margin_top = tf.margin_right = tf.margin_bottom = 0
        p = tf.paragraphs[0]
        p.text = title_text
        p.font.name = "Arial"
        p.font.size = Pt(21)
        p.font.bold = True
        p.font.color.rgb = COLOR_TITLE

        # 标签 Badge (浅蓝底 + 深蓝边框和字体)
        tag_box = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(10.2), Inches(0.44), Inches(2.3), Inches(0.36))
        tag_box.fill.solid()
        tag_box.fill.fore_color.rgb = RGBColor(219, 234, 254)
        tag_box.line.color.rgb = COLOR_PRIMARY
        tag_box.line.width = Pt(1)
        tf_tag = tag_box.text_frame
        tf_tag.vertical_anchor = MSO_ANCHOR.MIDDLE
        p_tag = tf_tag.paragraphs[0]
        p_tag.alignment = PP_ALIGN.CENTER
        p_tag.text = tag_text
        p_tag.font.name = "Arial"
        p_tag.font.size = Pt(10)
        p_tag.font.bold = True
        p_tag.font.color.rgb = COLOR_PRIMARY_DARK

        # 页码指示
        page_box = slide.shapes.add_textbox(Inches(11.5), Inches(7.0), Inches(1.2), Inches(0.35))
        tf_p = page_box.text_frame
        p_page = tf_p.paragraphs[0]
        p_page.alignment = PP_ALIGN.RIGHT
        p_page.text = f"{slide_index} / {total_slides}"
        p_page.font.size = Pt(11)
        p_page.font.color.rgb = COLOR_MUTED

    def add_card(slide, left, top, width, height, title, lines, title_color=COLOR_PRIMARY_DARK, bg_color=COLOR_SURFACE, border_color=COLOR_BORDER):
        card = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(left), Inches(top), Inches(width), Inches(height))
        card.fill.solid()
        card.fill.fore_color.rgb = bg_color
        card.line.color.rgb = border_color
        card.line.width = Pt(1)

        tx_box = slide.shapes.add_textbox(Inches(left + 0.22), Inches(top + 0.16), Inches(width - 0.44), Inches(height - 0.32))
        tf = tx_box.text_frame
        tf.word_wrap = True
        tf.margin_left = tf.margin_top = tf.margin_right = tf.margin_bottom = 0

        p0 = tf.paragraphs[0]
        p0.text = title
        p0.font.name = "Arial"
        p0.font.size = Pt(13)
        p0.font.bold = True
        p0.font.color.rgb = title_color
        p0.space_after = Pt(7)

        for line in lines:
            p = tf.add_paragraph()
            p.text = line
            p.font.name = "Arial"
            p.font.size = Pt(11)
            p.font.color.rgb = COLOR_TEXT
            p.space_after = Pt(4)
        return card

    # =========================================================================
    # SLIDE 1: Title (封面)
    # =========================================================================
    slide1 = prs.slides.add_slide(blank_layout)
    set_slide_background(slide1)

    top_badge = slide1.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.8), Inches(0.8), Inches(3.2), Inches(0.38))
    top_badge.fill.solid()
    top_badge.fill.fore_color.rgb = RGBColor(219, 234, 254)
    top_badge.line.color.rgb = COLOR_PRIMARY
    tf_b = top_badge.text_frame
    p_b = tf_b.paragraphs[0]
    p_b.alignment = PP_ALIGN.CENTER
    p_b.text = "IEEE T-RO / RA-L ARTIFACT"
    p_b.font.size = Pt(11)
    p_b.font.bold = True
    p_b.font.color.rgb = COLOR_PRIMARY_DARK

    t_box = slide1.shapes.add_textbox(Inches(0.8), Inches(1.4), Inches(11.7), Inches(2.0))
    tf1 = t_box.text_frame
    tf1.word_wrap = True
    p1 = tf1.paragraphs[0]
    p1.text = "面向复杂室内环境的自主移动机器人\n退化自适应多传感器融合 SLAM 与语义动态导航"
    p1.font.name = "Arial"
    p1.font.size = Pt(28)
    p1.font.bold = True
    p1.font.color.rgb = COLOR_PRIMARY_DARK
    p1.space_after = Pt(10)

    p1_sub = tf1.add_paragraph()
    p1_sub.text = "Adaptive Degeneracy-Aware Multi-Sensor Fusion SLAM & Semantic Dynamic Navigation for AMRs"
    p1_sub.font.size = Pt(14)
    p1_sub.font.color.rgb = COLOR_MUTED
    p1_sub.space_after = Pt(16)

    p1_auth = tf1.add_paragraph()
    p1_auth.text = "汇报人：智能机器人系统联合攻关团队   |   指导机构：机器人学与自主系统实验室"
    p1_auth.font.size = Pt(12)
    p1_auth.font.bold = True
    p1_auth.font.color.rgb = COLOR_TITLE

    # 三张浅蓝风格指标高亮卡片
    metrics = [
        ("100m 终点累积漂移", "2.14 cm", "指标要求 < 10 cm (达成超预期)", COLOR_PRIMARY),
        ("动态拖影消除率", "97.95%", "背景墙体保留率 99.4%", COLOR_SUCCESS),
        ("动态穿行效率提升", "+36.3%", "重规划平均延迟 < 20 ms", COLOR_WARNING),
    ]
    for idx, (m_title, m_val, m_desc, m_color) in enumerate(metrics):
        cx = 0.8 + idx * 4.0
        c_shape = slide1.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(cx), Inches(4.7), Inches(3.7), Inches(1.8))
        c_shape.fill.solid()
        c_shape.fill.fore_color.rgb = COLOR_SURFACE
        c_shape.line.color.rgb = m_color
        c_shape.line.width = Pt(1.5)
        c_tf = c_shape.text_frame
        c_tf.margin_left = Inches(0.2)
        cp0 = c_tf.paragraphs[0]
        cp0.text = m_title
        cp0.font.size = Pt(12)
        cp0.font.bold = True
        cp0.font.color.rgb = m_color
        cp0.space_after = Pt(4)
        cp1 = c_tf.add_paragraph()
        cp1.text = m_val
        cp1.font.size = Pt(24)
        cp1.font.bold = True
        cp1.font.color.rgb = COLOR_TITLE
        cp1.space_after = Pt(4)
        cp2 = c_tf.add_paragraph()
        cp2.text = m_desc
        cp2.font.size = Pt(10)
        cp2.font.color.rgb = COLOR_MUTED

    # =========================================================================
    # SLIDE 2: Research Background & 3 Challenges
    # =========================================================================
    slide2 = prs.slides.add_slide(blank_layout)
    set_slide_background(slide2)
    add_header(slide2, "1. 研究背景与三大核心科学问题", "Background & Motivation", 2)

    intro_box = slide2.shapes.add_textbox(Inches(0.8), Inches(1.15), Inches(11.7), Inches(0.7))
    intro_tf = intro_box.text_frame
    intro_tf.word_wrap = True
    ip = intro_tf.paragraphs[0]
    ip.text = "自主移动机器人 (AMR) 深入仓储物流、医院、科研实验室等高动态人机共融场景，面临三大共性瓶颈难题："
    ip.font.size = Pt(12)
    ip.font.color.rgb = COLOR_TEXT

    cards_s2 = [
        ("⚠️ 走廊退化致纵向滑移", [
            "• 长直走廊与低纹理房间缺乏法向几何特征",
            "• 雅可比信息矩阵 J^T J 严重秩亏（特征值极小）",
            "• 经典 LIO-SAM 产生纵向严重滑移与累计漂移"
        ]),
        ("👻 动态行人穿行产生拖影", [
            "• 行走中的行人在激光点云中留下大量假阳性拖影",
            "• 传统 2D 框直接挖除会大面积凿穿后方静态墙体",
            "• 破坏地图连续性，阻断下游自主导航可行区域"
        ]),
        ("🛑 传统代价地图过度保守", [
            "• 传统局部代价地图将行人对称向外膨胀",
            "• 盲目封死行人后方可通行空旷区域",
            "• 导致 AMR 在人流中频繁出现急停、避让与死锁"
        ]),
    ]
    for idx, (c_t, c_lines) in enumerate(cards_s2):
        add_card(slide2, 0.8 + idx * 4.0, 1.9, 3.7, 3.7, c_t, c_lines)

    add_card(slide2, 0.8, 5.8, 11.7, 1.0, "🎯 攻关目标：", [
        "构建紧耦合自适应退化感知因子图 SLAM、跨域语义 Depth-PROSAC 滤波与非对称速度矢量闭环导航系统。"
    ], COLOR_PRIMARY_DARK, RGBColor(224, 239, 255), COLOR_PRIMARY)

    # =========================================================================
    # SLIDE 3: System Architecture
    # =========================================================================
    slide3 = prs.slides.add_slide(blank_layout)
    set_slide_background(slide3)
    add_header(slide3, "2. 系统总体技术架构设计", "System Architecture", 3)

    img_arch = "paper/figures/fig_system_architecture.png"
    if os.path.exists(img_arch):
        slide3.shapes.add_picture(img_arch, Inches(0.8), Inches(1.25), width=Inches(11.7))

    add_card(slide3, 0.8, 5.5, 5.7, 1.5, "底层感知与因子图状态估计层", [
        "• 3D LiDAR + 200 Hz IMU 连续流形预积分刚性约束",
        "• J^T J 实时特征值分解检测退化，动态自适应重调权"
    ])
    add_card(slide3, 6.8, 5.5, 5.7, 1.5, "语义投影与非对称动态规划层", [
        "• YOLOv8/Mask R-CNN 跨域映射 + Depth-PROSAC 剔除动态重影",
        "• 非对称速度矢量椭圆膨胀 + A* 拓扑寻径 + TEB 弹性带重规划"
    ])

    # =========================================================================
    # SLIDE 4: Innovation 1 - Degeneracy-Aware Factor Graph SLAM
    # =========================================================================
    slide4 = prs.slides.add_slide(blank_layout)
    set_slide_background(slide4)
    add_header(slide4, "3. 创新点一：自适应退化感知与因子图重调权", "Degeneracy SLAM", 4)

    img_fg = "paper/figures/fig_factor_graph.png"
    if os.path.exists(img_fg):
        slide4.shapes.add_picture(img_fg, Inches(0.8), Inches(1.25), width=Inches(6.2))

    add_card(slide4, 7.3, 1.25, 5.2, 2.3, "特征值分解与自适应加权算子", [
        "H = Σ J_k^T J_k = V Λ V^T,   λ_min = min(λ_i)",
        "w_L = clamp(λ_min / λ_thresh, 0.10, 1.0)",
        "在退化方向上抑制弱约束激光残差，重心交予高频 IMU；",
        "在非退化方向上保留激光毫米级几何刚性约束。"
    ], COLOR_PRIMARY_DARK, COLOR_SURFACE_ALT, COLOR_PRIMARY)

    add_card(slide4, 0.8, 4.8, 5.7, 2.0, "开阔几何丰富环境 (λ_min ≥ 25.0)", [
        "• w_L = 1.0：完全信任 3D LiDAR 点面配准残差",
        "• 结合全局回环因子，消除长周期累积漂移",
        "• 保证全局拓扑建图的毫米级几何一致性"
    ])

    add_card(slide4, 6.8, 4.8, 5.7, 2.0, "长直退化走廊环境 (λ_min < 25.0)", [
        "• w_L → 0.10：显著抑制轴向弱约束引发的优化病态",
        "• 200 Hz IMU 流形预积分无缝接管纵向自由度",
        "• 实测走廊退化识别召回率 100.0%，误报率 0.0%"
    ])

    # =========================================================================
    # SLIDE 5: Innovation 2 - Depth-PROSAC Semantic Projection
    # =========================================================================
    slide5 = prs.slides.add_slide(blank_layout)
    set_slide_background(slide5)
    add_header(slide5, "4. 创新点二：跨域语义投影与 Depth-PROSAC 动态点云消除", "Depth-PROSAC", 5)

    add_card(slide5, 0.8, 1.25, 11.7, 1.5, "📷 相机-激光雷达跨域针孔投影几何模型", [
        "Z_c [u, v, 1]^T = K * (R_c_l * P_L + t_c_l)",
        "严格结合 RealSense D435i 光学内参 K 与小车刚体外参 T_c_l，将 2D 目标检测框与实例分割掩码映射至 3D 点云空间。"
    ], COLOR_PRIMARY_DARK, COLOR_SURFACE_ALT, COLOR_PRIMARY)

    add_card(slide5, 0.8, 3.0, 5.7, 3.7, "传统直接滤除方法缺陷", [
        "• 缺陷机制：",
        "  传统方法直接剔除投影在 2D 目标检测框内的所有激光点云。",
        "• 严重后果：",
        "  动态行人后方遮挡的静态背景墙体被大面积“凿穿击空”；",
        "  破坏房间与走廊结构完整性，使全局地图产生漏洞。"
    ], COLOR_WARNING, COLOR_SURFACE, COLOR_BORDER)

    add_card(slide5, 6.8, 3.0, 5.7, 3.7, "本文 Depth-PROSAC 算法突破", [
        "• 算法机制：",
        "  利用深度不连续性（Depth Discontinuity）构建渐进采样池；",
        "  对潜在动态类别聚类分割前景体素，仅滤除行人真实点簇。",
        "• 突破指标：",
        "  - 动态拖影消除率达 97.95%！",
        "  - 背景静态墙体完整保留率高达 99.4%！"
    ], COLOR_SUCCESS, COLOR_SURFACE, RGBColor(167, 243, 208))

    # =========================================================================
    # SLIDE 6: Innovation 3 - Asymmetric Velocity Ellipse & Replanning
    # =========================================================================
    slide6 = prs.slides.add_slide(blank_layout)
    set_slide_background(slide6)
    add_header(slide6, "5. 创新点三：非对称速度矢量椭圆与闭环重规划", "Asymmetric Navigation", 6)

    img_cost = "paper/figures/fig_asymmetric_costmap.png"
    if os.path.exists(img_cost):
        slide6.shapes.add_picture(img_cost, Inches(0.8), Inches(1.25), width=Inches(5.6))

    add_card(slide6, 6.7, 1.25, 5.8, 2.9, "非对称速度矢量动态保护包络模型", [
        "x'^2 / a^2 + y'^2 / b^2 ≤ 1",
        "• 前向长半轴：a_front = r_0 + β * ||v_p|| （沿行人航向迎风安全扩张）",
        "• 后向短半轴：a_back = r_0 （行人移出区域立即释放通行）",
        "• 侧向半轴：b = r_0 （维持侧向安全护身距离）",
        "推导了解析梯度 ∇ f_obs，支持 TEB 优化器在 18 ms 内快速收敛。"
    ], COLOR_PRIMARY_DARK, COLOR_SURFACE_ALT, COLOR_PRIMARY)

    add_card(slide6, 0.8, 4.4, 5.6, 2.4, "TEB 局部弹性时间带规划", [
        "• 在非对称代价场中优化软硬件动力学约束",
        "• 综合时间最优、避障势场与路径贴合度",
        "• 输出平滑连续线速度 v 与角速度 ω"
    ])

    add_card(slide6, 6.7, 4.4, 5.8, 2.4, "全局-局部双闭环重规划状态机", [
        "• 当动态行人完全封锁通道引发局部优化失败时",
        "• 状态机在 < 20 ms 内自适应触发全局 A* 拓扑重新选径",
        "• AMR 连续顺畅巡航，彻底消除传统机器人停滞呆死"
    ])

    # =========================================================================
    # SLIDE 7: Engineering Rigor & Closed-Loop Refinement (健全性与闭环优化)
    # =========================================================================
    slide7 = prs.slides.add_slide(blank_layout)
    set_slide_background(slide7)
    add_header(slide7, "6. 工程健全性与算法闭环深度优化", "Engineering Rigor", 7)

    cards_s7 = [
        ("🛡️ Costmap 静态/动态双层架构", [
            "• C_sem = max(C_static, C_dynamic)",
            "• 规划主循环每轮起始调用 clear_dynamic_layer()",
            "• 毫秒级清空动态层，彻底消除移动行人历史残留与幽灵障碍墙"
        ]),
        ("🎯 TEB 路径投影前瞻检索（杜绝倒退）", [
            "• 计算位姿在全局路径上的正交最近投影点 i* = argmin ||w_i - p||",
            "• 从 i* 沿前进方向前向检索首个前瞻目标点 k* ≥ i*",
            "• 彻底解决小车行进 >0.5m 后前视点倒退锁死在原点的经典死锁"
        ]),
        ("⚡ A* 静态节点池与版本号零堆分配", [
            "• 预分配 nodes_pool_ 结合 32-bit run_id 版本代号",
            "• 状态重置代价从全图遍历降为 O(1)，寻径耗时 < 1 ms",
            "• 消除 200×200 网格每秒 5 次的 1.28 MB 频繁堆分配与 GC 抖动"
        ]),
        ("🧪 24 项 GoogleTest 自动化单元测试", [
            "• 涵盖退化加权、极限奇异值、语义投影、分层数据隔离与前瞻单调检索",
            "• 100% 单元测试通过率 (24 passed, 0 failed, 0 skipped)",
            "• 外加全真仿真世界直行与位移闭环冒烟测试全面通过"
        ]),
    ]
    for idx, (c_t, c_lines) in enumerate(cards_s7):
        col = idx % 2
        row = idx // 2
        add_card(slide7, 0.8 + col * 6.0, 1.25 + row * 2.8, 5.7, 2.5, c_t, c_lines)

    # =========================================================================
    # SLIDE 8: High-Fidelity Testbed & AMR Robot Platform
    # =========================================================================
    slide8 = prs.slides.add_slide(blank_layout)
    set_slide_background(slide8)
    add_header(slide8, "7. 高保真实验场地与工业级 AMR 仿真平台", "Digital Twin Platform", 8)

    add_card(slide8, 0.8, 1.25, 5.7, 4.0, "真实机器人实验室数字孪生建模", [
        "• 严格标准建筑要素：",
        "  9.0m 退化长直走廊、双向 1.2m 金属门套、全场 8cm 踢脚线与高光耐磨环氧地坪。",
        "• 完整科研实验设施：",
        "  地面主控工作站（双联大屏/主机/转椅）、19寸服务器机柜（带阵列指示灯）、重型仓储货架与自主充电桩。",
        "• 安全生活细节：",
        "  墙挂式干粉灭火器、绿色 EXIT 逃生标志与室内现代盆栽。"
    ])

    add_card(slide8, 6.8, 1.25, 5.7, 4.0, "高保真 AMR 工业级移动机器人", [
        "• 越野机动轮组：",
        "  深花纹防滑橡胶胎 + CNC 铝合金轮毂 + 阳极氧化橙色防脱圈。",
        "• 工业战术装甲：",
        "  低重心战术合金机身、前后防撞钢梁 (Bullbar)、双前高亮 LED 大灯与后置双红色尾灯、E-Stop 急停开关。",
        "• 传感器铝型材塔架：",
        "  T 型 4020 铝型材、Jetson AGX 造型工控机、3D 激光雷达与 RealSense D435i 深度相机。"
    ])

    add_card(slide8, 0.8, 5.5, 11.7, 1.3, "📹 世界全局全景视角 (World Perspective) 录制验证：", [
        "固定架设于世界坐标 (2.0, -4.6, 5.2)，俯仰角 47° 俯瞰全场，无抖动完整记录小车在复杂场地内的长周期多回路连续自主巡航 (simulation_60s.mp4)。"
    ], COLOR_PRIMARY_DARK, COLOR_SURFACE_ALT, COLOR_PRIMARY)

    # =========================================================================
    # SLIDE 9: Experimental Benchmarks & Quantitative Trajectory
    # =========================================================================
    slide9 = prs.slides.add_slide(blank_layout)
    set_slide_background(slide9)
    add_header(slide9, "8. 实验评估与核心量化指标对比", "Empirical Benchmarks", 9)

    img_traj = "paper/figures/fig_trajectories.png"
    img_err = "paper/figures/fig_error_time.png"
    if os.path.exists(img_traj):
        slide9.shapes.add_picture(img_traj, Inches(0.8), Inches(1.25), width=Inches(5.7))
    if os.path.exists(img_err):
        slide9.shapes.add_picture(img_err, Inches(6.8), Inches(1.25), width=Inches(5.7))

    # 表格 (浅蓝科技表头)
    rows_data = [
        ["纯轮速计 (Raw Wheel Odom)", "2.9686 m", "5.2821 m", "246.30 cm", "❌ 严重漂移失真"],
        ["标准 LIO-SAM (Shan et al.)", "0.9845 m", "3.9654 m", "86.40 cm", "⚠️ 走廊段严重纵向滑移"],
        ["本文自适应融合 (Ours)", "0.0219 m", "0.0741 m", "2.14 cm", "✅ 远优于 <10cm 指标"],
    ]
    t_shape = slide9.shapes.add_table(4, 5, Inches(0.8), Inches(4.7), Inches(11.7), Inches(1.8))
    table = t_shape.table
    table.columns[0].width = Inches(3.2)
    table.columns[1].width = Inches(2.1)
    table.columns[2].width = Inches(2.1)
    table.columns[3].width = Inches(2.1)
    table.columns[4].width = Inches(2.2)

    headers = ["评估方法", "ATE RMSE (m)", "最大轨迹漂移 (m)", "100m 终点累积漂移", "指标达成判定"]
    for c_idx, h in enumerate(headers):
        cell = table.cell(0, c_idx)
        cell.fill.solid()
        cell.fill.fore_color.rgb = RGBColor(219, 234, 254)
        p = cell.text_frame.paragraphs[0]
        p.text = h
        p.font.size = Pt(11)
        p.font.bold = True
        p.font.color.rgb = COLOR_PRIMARY_DARK

    for r_idx, r_data in enumerate(rows_data):
        for c_idx, val in enumerate(r_data):
            cell = table.cell(r_idx + 1, c_idx)
            cell.fill.solid()
            if r_idx == 2:
                cell.fill.fore_color.rgb = RGBColor(236, 253, 245)
            else:
                cell.fill.fore_color.rgb = COLOR_SURFACE if r_idx % 2 == 0 else RGBColor(241, 245, 249)
            p = cell.text_frame.paragraphs[0]
            p.text = val
            p.font.size = Pt(10)
            if r_idx == 2:
                p.font.bold = True
                p.font.color.rgb = COLOR_SUCCESS if c_idx >= 3 else COLOR_TITLE
            else:
                p.font.color.rgb = COLOR_TEXT

    # =========================================================================
    # SLIDE 10: Dynamic Obstacle Ablation & Transit Efficiency
    # =========================================================================
    slide10 = prs.slides.add_slide(blank_layout)
    set_slide_background(slide10)
    add_header(slide10, "9. 动态避障消融实验与通行效率分析", "Ablation Study", 10)

    t2_shape = slide10.shapes.add_table(4, 6, Inches(0.8), Inches(1.3), Inches(11.7), Inches(2.3))
    table2 = t2_shape.table
    headers2 = ["行人横穿速度", "传统对称膨胀耗时", "本文非对称耗时", "通行效率提升", "重规划平均延迟", "最小安全边距"]
    for c_idx, h in enumerate(headers2):
        cell = table2.cell(0, c_idx)
        cell.fill.solid()
        cell.fill.fore_color.rgb = RGBColor(219, 234, 254)
        p = cell.text_frame.paragraphs[0]
        p.text = h
        p.font.size = Pt(11)
        p.font.bold = True
        p.font.color.rgb = COLOR_PRIMARY_DARK

    ablation_rows = [
        ["0.2 m/s", "15.35 s", "11.18 s", "+27.2%", "18.44 ms", "0.43 m"],
        ["0.6 m/s", "17.21 s", "11.43 s", "+33.6%", "19.47 ms", "0.45 m"],
        ["1.0 m/s", "18.55 s", "11.82 s", "+36.3%", "19.76 ms", "0.44 m"],
    ]
    for r_idx, r_data in enumerate(ablation_rows):
        for c_idx, val in enumerate(r_data):
            cell = table2.cell(r_idx + 1, c_idx)
            cell.fill.solid()
            cell.fill.fore_color.rgb = COLOR_SURFACE if r_idx % 2 == 0 else RGBColor(241, 245, 249)
            p = cell.text_frame.paragraphs[0]
            p.text = val
            p.font.size = Pt(11)
            if c_idx == 3:
                p.font.bold = True
                p.font.color.rgb = COLOR_SUCCESS
            else:
                p.font.color.rgb = COLOR_TEXT

    add_card(slide10, 0.8, 3.9, 11.7, 2.8, "📊 消融实验核心结论", [
        "1. 通行耗时降低 36.3%：非对称速度矢量椭圆释放了行人后方通行区域，机器人无需盲目等待行人完全走过即可贴身变道穿行。",
        "2. 安全裕度恒定充裕 (>0.43m)：迎风扩张椭圆包络确保机器人在面对迎面行走人员时提前减速或微调避让，零碰撞发生。",
        "3. 重规划超低延迟 (<20ms)：推导了解析梯度，配合 A* 静态节点池优化，状态机触发重选径响应时间仅 18.25 ms，实现连续不停顿巡航。"
    ], COLOR_PRIMARY_DARK, COLOR_SURFACE_ALT, COLOR_PRIMARY)

    # =========================================================================
    # SLIDE 11: Conclusion & Open Source Deliverables
    # =========================================================================
    slide11 = prs.slides.add_slide(blank_layout)
    set_slide_background(slide11)
    add_header(slide11, "10. 总结、学术贡献与开源生态交付", "Conclusion & Delivery", 11)

    add_card(slide11, 0.8, 1.25, 5.7, 4.0, "学术贡献总结 (Contributions)", [
        "• 理论贡献：",
        "  建立基于雅可比信息矩阵特征值分解的自适应因子图重调权机制，彻底攻克长直走廊特征退化瓶颈。",
        "• 算法贡献：",
        "  提出跨域语义投影与 Depth-PROSAC 算法，实现 97.95% 动态拖影滤除与 99.4% 结构保留；构建非对称速度椭圆与 A*-TEB 双闭环重规划。",
        "• 工程贡献：",
        "  研制真实数字孪生实验环境与全地形工业 AMR 机器人模型，24 项单元测试全覆盖，实测 100m 漂移仅 2.14 cm。"
    ])

    add_card(slide11, 6.8, 1.25, 5.7, 4.0, "完整开源套件交付 (Open-Source Deliverables)", [
        "• 📄 完整学术论文交付套件：",
        "  IEEE Transactions 格式 LaTeX 排版源码 (paper/paper.tex & paper.pdf) 与 Markdown (PAPER.md)。",
        "• 💻 核心 ROS 2 软件包：",
        "  smartcar_slam（融合定位与语义规划）与 smartcar_teleop（控制与巡航测试），24 项单元测试 100% 通过。",
        "• 🌐 高保真数字孪生世界与机器人模型：",
        "  Ignition Gazebo Fortress 真实实验室场景与 AMR 模型。",
        "• 📹 60 秒高清实测演示录像：",
        "  世界全局全景视角实测高清视频 (simulation_60s.mp4)。"
    ])

    thanks_box = slide11.shapes.add_textbox(Inches(0.8), Inches(5.55), Inches(11.7), Inches(1.2))
    tf_th = thanks_box.text_frame
    p_th = tf_th.paragraphs[0]
    p_th.alignment = PP_ALIGN.CENTER
    p_th.text = "感谢各位专家评委与学者批评指正！"
    p_th.font.size = Pt(22)
    p_th.font.bold = True
    p_th.font.color.rgb = COLOR_PRIMARY_DARK

    p_qa = tf_th.add_paragraph()
    p_qa.alignment = PP_ALIGN.CENTER
    p_qa.text = "Q & A 答辩环节"
    p_qa.font.size = Pt(14)
    p_qa.font.color.rgb = COLOR_MUTED

    # 保存 PPTX 文件
    out_dir = "slides"
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "presentation.pptx")
    prs.save(out_path)
    print(f">>> [OK] 浅蓝色主题演示文稿已成功生成: {out_path} (共 {total_slides} 页, 16:9 宽屏)")

if __name__ == "__main__":
    create_presentation()

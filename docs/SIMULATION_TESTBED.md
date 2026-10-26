# 真实机器人实验场地数字孪生与仿真录制开发手册

## 1. 实验室环境建模规范 ([`smart_car_world.sdf`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/src/smartcar_sim/worlds/smart_car_world.sdf))

场地根据真实室内自主移动机器人测试标准（DARPA SubT / RoboCup Industrial）设计：

### 1.1 建筑与空间分区
- **测试走廊 (Corridor)**：$X \in [-2.5, 6.5]\text{ m}, Y \in [-1.6, 0.85]\text{ m}$，净宽约 $2.45\text{ m}$，用于验证长直道几何退化；
- **北侧综合测试区 (North Arena)**：$X \in [-2.5, 6.5]\text{ m}, Y \in [0.85, 3.2]\text{ m}$，宽敞多功能测试区；
- **金属门套门洞 (Portals)**：
  - 东门洞：$X \in [2.1, 3.3]\text{ m}$（净宽 $1.2\text{ m}$）；
  - 西门洞：$X \in [-0.6, 0.6]\text{ m}$（净宽 $1.2\text{ m}$）；
  门洞两侧加装深灰铝合金门框立柱，顶部设有横向门楣，逼真模拟室内建筑穿行。
- **踢脚线细节**：全场墙体底部嵌装 $8\text{ cm}$ 深色踢脚线（`#181a1d`）。

### 1.2 实验家具与基础设施
1. **地面主控工作站 (`ground_control_station`)**：
   - 双联 27 寸宽屏 LCD 显示器（左倾 $15^\circ$，右倾 $-15^\circ$）；
   - 工控塔式机箱，配有常亮蓝色工作指示灯；
   - 黑色办公网布五星轮人体工学转椅。
2. **19寸标准服务器机柜 (`server_telemetry_rack`)**：
   - 黑色防尘机柜配深色透明玻璃门，内嵌 2U 服务器与绿/黄双色指示灯阵列。
3. **重型仓储货架 (`industrial_storage_shelving`)**：
   - 冷轧钢货架，堆叠放置蓝色与黄色分类零件收纳盒及铝制仪器箱。
4. **自主充电对准桩 (`charging_dock_station`)**：
   - 地面充电触点板与立柱正面的 AprilTag / ArUco 高对比度视觉对准靶标。
5. **安全与生活点缀**：
   - 挂墙式干粉灭火器 (`fire_extinguisher`)；
   - 门楣发光安全出口指示牌 (`exit_sign`)；
   - 白色方形花盆室内观叶绿植 (`indoor_potted_plant`)。

---

## 2. 工业级自主移动机器人 (AMR) 模型 ([`model.sdf`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/src/smartcar_sim/models/smart_car/model.sdf))

### 2.1 底盘机身
- 低重心战术深灰合金机身，前部带倾角斜切装甲护板；
- 前后配备重型防撞钢梁 (Bullbar)；
- 前置高亮度白光双 LED 矩阵车灯，后置双红光状态尾灯；
- 顶层碳纤维板配置鲜红色急停蘑菇头按钮 (E-Stop) 与双全向天线。

### 2.2 传感器与计算塔架
- T 型 4020 黑色阳极氧化铝型材塔架；
- 车载工控机（NVIDIA Jetson AGX Orin 风格造型，带散热鳍片与状态绿灯）；
- 3D 激光雷达（轴向散热座 + 深蓝增透膜光学视窗 + 同心顶盖）；
- RGB-D 深度相机（Intel RealSense D435i 造型，带双红外镜头、RGB 镜头与散斑投射器）；
- 四组深花纹越野橡胶外胎 + CNC 铝合金轮毂 + 橙色阳极防脱点缀圈。

---

## 3. 世界全局视角 (World Perspective) 录制与回放

### 3.1 全景相机配置
- 相机服务调用脚本：[`gui_world_view.sh`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/gui_world_view.sh)
- 世界坐标位置：$X = 2.0\text{ m}, Y = -4.6\text{ m}, Z = 5.2\text{ m}$；
- 姿态四元数：$x = -0.2819, y = 0.2819, z = 0.6485, w = 0.6485$（俯角约 $47^\circ$ 正对北方）；
- 视角效果：无镜头跟随晃动，全局俯瞰小车在完整实验场地内的多回路闭环导航。

### 3.2 一键录制与校验命令
```bash
# 启动 60 秒高清视频录制并同步记录双传感器遥测数据
./record_simulation_60s.sh

# 提取并验证录制视频元信息
ffprobe -v error -show_entries format=duration,size -select_streams v:0 -show_entries stream=width,height simulation_60s.mp4
```
生成的视频保存在 [`simulation_60s.mp4`](file:///home/aidlux/ros2_ws/SmartCar_Gazebo_ws/simulation_60s.mp4)，遥测数据保存在 `/tmp/sim_telemetry.log`。


#!/bin/bash
# 将 Gazebo GUI 相机切换为全局/世界俯瞰视角 (World Perspective)
# 视角位置：X=2.0, Y=-4.6, Z=5.2，俯仰角 ~47° 俯视整个室内测试场地
set -e

# 1. 取消任何模型跟踪 (确保世界视角固定)
ign service -s /gui/follow --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean \
  --timeout 3000 --req 'data: ""' >/dev/null 2>&1 || true

# 2. 调用 /gui/move_to/pose 服务将相机定位至世界全局视角
ign service -s /gui/move_to/pose \
  --reqtype ignition.msgs.GUICamera \
  --reptype ignition.msgs.Boolean \
  --timeout 5000 \
  --req 'pose: {position: {x: 2.0, y: -4.6, z: 5.2}, orientation: {x: -0.2819, y: 0.2819, z: 0.6485, w: 0.6485}}' >/dev/null 2>&1 || true

echo ">>> 相机已切换至世界全局俯视视角 (X=2.0, Y=-4.6, Z=5.2)"


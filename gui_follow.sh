#!/bin/bash
# 让 Gazebo GUI 的相机跟随小车（车只有 20 cm，默认视角很远，容易看不见）。
# 用法：./gui_follow.sh [跟随距离(米)，默认 1.6]
set -e
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIST="${1:-1.6}"
export IGN_GAZEBO_RESOURCE_PATH="$WS_DIR/install/smartcar_sim/share/smartcar_sim/models"

if ! ign service -s /gui/follow --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean \
     --timeout 5000 --req 'data: "smart_car"' >/dev/null 2>&1; then
  echo "连不上 Gazebo GUI：先启动带界面的仿真（./gui.sh 或 ./sim_teleop.sh）" >&2
  exit 1
fi

ign service -s /gui/follow/offset --reqtype ignition.msgs.Vector3d --reptype ignition.msgs.Boolean \
  --timeout 5000 --req "x: $DIST, y: -$DIST, z: 1.0" >/dev/null 2>&1 || true

echo ">>> 相机已跟随 smart_car（距离约 ${DIST} m）；想拉远/拉近：./gui_follow.sh 3"

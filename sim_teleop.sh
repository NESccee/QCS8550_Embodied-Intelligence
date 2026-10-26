#!/bin/bash
# 仿真 + 手柄遥控。用法：./sim_teleop.sh [launch 参数]
# 手柄：按住 LT 使能，RT 高速档。
# 有可用 X 显示时默认连 Gazebo 界面一起起；强制无界面加 gui:=false。
set -e
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "$WS_DIR/install/setup.bash"

ARGS=("$@")
has_gui=false
for a in "$@"; do case "$a" in gui:=*) has_gui=true ;; esac; done
if [ "$has_gui" = false ] && [ -n "$DISPLAY" ] && xdpyinfo >/dev/null 2>&1; then
  echo ">>> 检测到显示 $DISPLAY，带界面启动（强制无界面：./sim_teleop.sh gui:=false）"
  ARGS+=(gui:=true)
fi

exec ros2 launch smartcar_sim sim_teleop.launch.py "${ARGS[@]}"

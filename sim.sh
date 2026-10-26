#!/bin/bash
# 启动 Ignition Fortress 仿真 + ROS 桥接。用法：./sim.sh [launch 参数]
# 有可用 X 显示时默认连 Gazebo 界面一起起；强制无界面加 gui:=false。
set -e
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "$WS_DIR/install/setup.bash"

ARGS=("$@")
has_gui=false
for a in "$@"; do case "$a" in gui:=*) has_gui=true ;; esac; done
if [ "$has_gui" = false ] && [ -n "$DISPLAY" ] && xdpyinfo >/dev/null 2>&1; then
  echo ">>> 检测到显示 $DISPLAY，带界面启动（强制无界面：./sim.sh gui:=false）"
  ARGS+=(gui:=true)
fi

exec ros2 launch smartcar_sim sim.launch.py "${ARGS[@]}"

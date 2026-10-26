#!/bin/bash
# 构建工作空间。用法：./build.sh [包名 ...]
set -e
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
[ -f "$ROS_SETUP" ] || { echo "找不到 $ROS_SETUP，请确认 ROS 2 已安装" >&2; exit 1; }
source "$ROS_SETUP"

cd "$WS_DIR"
if [ $# -gt 0 ]; then
  echo ">>> 只构建：$*"
  colcon build --symlink-install --packages-select "$@"
else
  echo ">>> 构建全部包"
  colcon build --symlink-install
fi

echo
echo ">>> 完成。当前终端生效：source $WS_DIR/install/setup.bash（新终端自动生效）"

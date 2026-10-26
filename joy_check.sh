#!/bin/bash
# 手柄自检：先列设备，再启动 joy_node + joy_monitor；Ctrl-C 后打印轴范围与按过的键。
set -e
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "$WS_DIR/install/setup.bash"

echo "=== /dev/input/by-id 下的输入设备 ==="
ls -1 /dev/input/by-id/ 2>/dev/null || echo "（读不到 /dev/input/by-id）"
if [ -e /dev/input/js0 ]; then
  echo "=== /dev/input/js0 存在 ==="
else
  echo "=== 注意：没有 /dev/input/js0（本机内核没编 joydev），但 joy 3.3.0 走 SDL，不影响 ==="
fi
echo "=== 启动 joy_node + joy_monitor（Ctrl-C 结束）==="
exec ros2 launch smartcar_teleop joy_monitor.launch.py "$@"

#!/bin/bash
# 只启动手柄遥控（发布 /cmd_vel，接实车底盘或已有仿真）。用法：./teleop.sh [launch 参数]
# 例：./teleop.sh cmd_vel_topic:=/diff_drive_controller/cmd_vel_unstamped
set -e
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "$WS_DIR/install/setup.bash"
exec ros2 launch smartcar_teleop teleop.launch.py button_deadman:=8 button_turbo:=9 "$@"

#!/bin/bash
# 单元测试；加 --sim 再跑一遍仿真冒烟测试（起仿真 → drive_test 直行 → 校验里程计位移）。
set -e
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "$WS_DIR/install/setup.bash"

echo ">>> 单元测试"
(cd "$WS_DIR" && colcon test --packages-select smartcar_teleop smartcar_slam)
(cd "$WS_DIR" && colcon test-result --verbose)

if [ "${1:-}" != "--sim" ]; then
  echo
  echo ">>> 跳过仿真冒烟测试（加 --sim 执行）"
  exit 0
fi

SIM_LOG="${TMPDIR:-/tmp}/smartcar_sim_smoke.log"
SIM_PID=""
cleanup() {
  # ros2 launch 被 kill 时不会带走 ign / 桥接这些子进程，所以按进程组一起收
  if [ -n "$SIM_PID" ] && kill -0 "$SIM_PID" 2>/dev/null; then
    kill -INT  -"$SIM_PID" 2>/dev/null || true
    sleep 1
    kill -TERM -"$SIM_PID" 2>/dev/null || true
  fi
  wait "$SIM_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo
echo ">>> 启动仿真（日志：$SIM_LOG）"
# setsid 让它独占一个进程组，方便一次性收掉全部子进程
setsid ros2 launch smartcar_sim sim.launch.py >"$SIM_LOG" 2>&1 &
SIM_PID=$!

odom_field() {
  local field=$1 value
  for _ in $(seq 1 25); do
    value=$(timeout 10 ros2 topic echo /odom --once --field "$field" 2>/dev/null |
      grep -E '^[-+]?[0-9]' | head -1)
    if [ -n "$value" ]; then printf '%s' "$value"; return 0; fi
    sleep 1
  done
  return 1
}

echo -n "    等待 /odom …"
if ! x0=$(odom_field pose.pose.position.x); then
  echo " 失败：仿真没起来？看 $SIM_LOG"
  exit 1
fi
echo " 就绪（x0 = $x0）"

echo ">>> 直行 0.3 m/s × 3 s（按仿真时钟计时）"
# 必须用 use_sim_time：本机实时率 RTF 会随负载在 0.1~1.0 之间抖，墙钟计时会走出完全不同的距离。
# 也不能用固定超时——RTF 低时 3 仿真秒要很久，所以等节点自己打印“轨迹执行完毕”再量。
DT_LOG="${TMPDIR:-/tmp}/smartcar_drive_test.log"
: > "$DT_LOG"
setsid ros2 run smartcar_teleop drive_test --ros-args \
  -p pattern:=straight -p linear:=0.3 -p leg_time:=3.0 -p use_sim_time:=true \
  >"$DT_LOG" 2>&1 &
DRIVE_PID=$!
START=$(date +%s)
for _ in $(seq 1 180); do
  if grep -q '轨迹执行完毕' "$DT_LOG"; then break; fi
  if ! kill -0 "$DRIVE_PID" 2>/dev/null; then break; fi
  sleep 1
done
echo "    drive_test 用时 $(( $(date +%s) - START )) s（墙钟，等价于 3 仿真秒）"
# drive_test 跑完会一直发零速，kill 的是整个进程组，别留孤儿（孤儿会持续发零速干扰后续测试）
kill -INT -"$DRIVE_PID" 2>/dev/null || true
sleep 1
kill -TERM -"$DRIVE_PID" 2>/dev/null || true
wait "$DRIVE_PID" 2>/dev/null || true
timeout 10 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true
sleep 1

if ! x1=$(odom_field pose.pose.position.x); then
  echo "    读取 /odom 失败"
  exit 1
fi

python3 - "$x0" "$x1" <<'PY'
import sys
x0, x1 = float(sys.argv[1]), float(sys.argv[2])
moved = x1 - x0
print(f"    里程计位移 = {moved:+.4f} m（期望 0.7 ~ 1.0 m）")
if not 0.7 <= moved <= 1.0:
    print("    判定：失败")
    sys.exit(1)
print("    判定：通过")
PY

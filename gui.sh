#!/bin/bash
# 带界面的仿真：先探测显示和窗口，能开就开，开不了直接说清原因（而不是让 ign 崩一堆栈）。
# 用法：./gui.sh [launch 参数]
set -e
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "$WS_DIR/install/setup.bash"

WORLD="$WS_DIR/install/smartcar_sim/share/smartcar_sim/worlds/smart_car_world.sdf"
MODELS="$WS_DIR/install/smartcar_sim/share/smartcar_sim/models"

# 1) 找显示：环境变量优先，其次本机 X socket（AidLux 桌面通常是 :1，但常常没导出 DISPLAY）
if [ -z "$DISPLAY" ]; then
  for sock in /tmp/.X11-unix/X*; do
    [ -e "$sock" ] || continue
    cand=":${sock##*X}"
    if DISPLAY="$cand" xdpyinfo >/dev/null 2>&1; then export DISPLAY="$cand"; break; fi
  done
fi
if [ -z "$DISPLAY" ] || ! xdpyinfo >/dev/null 2>&1; then
  echo ">>> 没有可用的 X 显示（DISPLAY 未设置，也没有能连上的 /tmp/.X11-unix/X*）"
  echo "    无界面跑仿真：./sim.sh 或 ./sim_teleop.sh"
  exit 1
fi
echo ">>> 显示：$DISPLAY"

# 找 Gazebo 主窗口。注意：窗口管理器会把客户端窗口 reparent 进框架窗口，
# 所以它不在 xwininfo -root -children 里（那里只有 1x1 的 leader 窗口 "Gazebo GUI"），
# 必须查 _NET_CLIENT_LIST；拿不到时退回整棵树 xwininfo -root -tree。
gui_window() {
  local ids id
  ids=$(xprop -root _NET_CLIENT_LIST 2>/dev/null | tr ',' '\n' | grep -oE '0x[0-9a-f]+' || true)
  [ -n "$ids" ] || ids=$(xwininfo -root -tree 2>/dev/null | grep -i 'ign-gazebo-gui' | grep -oE '0x[0-9a-f]+' || true)
  for id in $ids; do
    if xprop -id "$id" WM_CLASS 2>/dev/null | grep -q 'ign-gazebo-gui'; then printf '%s' "$id"; return 0; fi
  done
  return 1
}

# 2) 探界面：起一次仿真，看有没有真正映射出来、尺寸正常的 Gazebo 窗口
echo ">>> 正在探测图形界面（最多 30 秒）..."
setsid env IGN_GAZEBO_RESOURCE_PATH="$MODELS" ign gazebo -r "$WORLD" > /tmp/gui_probe.log 2>&1 < /dev/null &
PROBE_PID=$!
ok=""
for _ in $(seq 1 30); do
  sleep 1
  if win=$(gui_window); then
    state=$(xwininfo -id "$win" 2>/dev/null | awk '/Map State/{print $3}')
    size=$(xwininfo -id "$win" 2>/dev/null | awk '/Width/{w=$2} /Height/{h=$2} END{print w"x"h}')
    width=${size%%x*}
    if [ "$state" = IsViewable ] && [ -n "$width" ] && [ "$width" -ge 200 ] 2>/dev/null; then
      ok=1
      echo "    窗口已就绪：$win  尺寸 $size  状态 $state"
      break
    fi
  fi
done
kill -INT -"$PROBE_PID" 2>/dev/null || true
sleep 1
kill -TERM -"$PROBE_PID" 2>/dev/null || true
wait "$PROBE_PID" 2>/dev/null || true
sleep 1

if [ -z "$ok" ]; then
  echo ">>> 探测失败：30 秒内没出现可见（宽度 ≥ 200）的 Gazebo 窗口"
  if [ -s /tmp/gui_probe.log ]; then
    echo "    探测日志尾部（完整见 /tmp/gui_probe.log）："
    tail -5 /tmp/gui_probe.log | sed 's/^/      /'
  fi
  cat <<'EOM'
    怎么办：
      1) 先看桌面任务栏有没有 Gazebo 窗口（可能只是被别的窗口挡住）；
      2) 手动跑一次看报错：ign gazebo -r <world.sdf>（别忘了设 IGN_GAZEBO_RESOURCE_PATH，
         直接跑 ./sim.sh gui:=true 则会自动设好）；
      3) 本机先无界面跑：./sim.sh 或 ./sim_teleop.sh。
EOM
  exit 2
fi

echo ">>> 界面可用，启动带界面的仿真"
exec ros2 launch smartcar_sim sim.launch.py gui:=true "$@"

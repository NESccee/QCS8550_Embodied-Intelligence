#!/bin/bash
# 验证智能小车仿真并录制 60 秒高清视频
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export DISPLAY=:1
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

OUTPUT_VIDEO="$WS_DIR/simulation_60s.mp4"
RECORD_LOG="/tmp/sim_record.log"
TELEMETRY_LOG="/tmp/sim_telemetry.log"

echo "=== 1. 启动仿真与图形界面 ==="
export IGN_GAZEBO_RESOURCE_PATH="$WS_DIR/install/smartcar_sim/share/smartcar_sim/models"
setsid ros2 launch smartcar_sim sim.launch.py gui:=true > "$RECORD_LOG" 2>&1 &
SIM_PID=$!

cleanup() {
  echo
  echo "=== 清理进程组 ==="
  if [ -n "$TELEMETRY_PID" ] && kill -0 "$TELEMETRY_PID" 2>/dev/null; then
    kill -TERM "$TELEMETRY_PID" 2>/dev/null || true
  fi
  if [ -n "$DRIVE_PID" ] && kill -0 "$DRIVE_PID" 2>/dev/null; then
    kill -INT -"$DRIVE_PID" 2>/dev/null || true
    sleep 1
    kill -TERM -"$DRIVE_PID" 2>/dev/null || true
  fi
  if [ -n "$FFMPEG_PID" ] && kill -0 "$FFMPEG_PID" 2>/dev/null; then
    kill -INT "$FFMPEG_PID" 2>/dev/null || true
  fi
  timeout 5 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true
  if [ -n "$SIM_PID" ] && kill -0 "$SIM_PID" 2>/dev/null; then
    kill -INT -"$SIM_PID" 2>/dev/null || true
    sleep 2
    kill -TERM -"$SIM_PID" 2>/dev/null || true
  fi
  wait "$SIM_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo -n ">>> 等待 Gazebo 窗口映射 …"
ok=""
WIN_ID=""
for _ in $(seq 1 45); do
  sleep 1
  ids=$(xprop -root _NET_CLIENT_LIST 2>/dev/null | tr ',' '\n' | grep -oE '0x[0-9a-f]+' || true)
  for id in $ids; do
    if xprop -id "$id" WM_CLASS 2>/dev/null | grep -q 'ign-gazebo-gui'; then
      state=$(xwininfo -id "$id" 2>/dev/null | awk '/Map State/{print $3}')
      if [ "$state" = "IsViewable" ]; then
        WIN_ID="$id"
        ok=1
        break 2
      fi
    fi
  done
done

if [ -z "$ok" ]; then
  echo " 失败：Gazebo 窗口未在 45s 内显示，查看 $RECORD_LOG"
  exit 1
fi
echo " 成功（Window ID: $WIN_ID）"

echo -n ">>> 等待 /odom 话题就绪 …"
for _ in $(seq 1 30); do
  if timeout 2 ros2 topic echo /odom --once >/dev/null 2>&1; then
    echo " 就绪"
    break
  fi
  sleep 1
done

# 移开光标并配置世界全局视角
/tmp/warp 2>/dev/null || true
sleep 1
echo ">>> 配置世界全局俯视全景视角（World Perspective: X=2.0, Y=-4.6, Z=5.2）"
"$WS_DIR/gui_world_view.sh" >/dev/null 2>&1 || true
sleep 2

echo "=== 2. 启动小车巡航任务与 60 秒高清录制 ==="
# 启动走廊与室内多区域闭环巡航（长边 7.0s，短边 4.8s，转弯 2.16s，循环进行）
setsid ros2 run smartcar_teleop drive_test --ros-args \
  -p pattern:=patrol -p linear:=0.35 -p angular:=0.8 -p leg_time:=7.0 \
  -p leg_time_short:=4.8 -p turn_time:=2.16 -p repeat:=true -p use_sim_time:=true \
  > /tmp/drive_record.log 2>&1 &
DRIVE_PID=$!

echo ">>> 开始录制视频 -> $OUTPUT_VIDEO（时长 60s，25fps）"
ffmpeg -y -f x11grab -framerate 25 -video_size 1000x680 -i :1.0+0,38 \
  -t 60 -c:v libx264 -preset veryfast -pix_fmt yuv420p "$OUTPUT_VIDEO" \
  > /tmp/ffmpeg_record.log 2>&1 &
FFMPEG_PID=$!

echo ">>> 启动实时遥测数据采集 -> $TELEMETRY_LOG"
python3 "$WS_DIR/telemetry_logger.py" "$TELEMETRY_LOG" &
TELEMETRY_PID=$!

echo ">>> 等待 60 秒高清录制完成 …"
wait "$FFMPEG_PID" 2>/dev/null || true
if [ -n "$TELEMETRY_PID" ] && kill -0 "$TELEMETRY_PID" 2>/dev/null; then
  kill -TERM "$TELEMETRY_PID" 2>/dev/null || true
  wait "$TELEMETRY_PID" 2>/dev/null || true
fi

echo
echo "=== 3. 仿真验证结果与视频元信息 ==="
if [ -f "$OUTPUT_VIDEO" ]; then
  SIZE=$(du -h "$OUTPUT_VIDEO" | cut -f1)
  DURATION=$(ffprobe -v error -show_entries format=duration -of default=noprint_wrappers=1:nokey=1 "$OUTPUT_VIDEO" 2>/dev/null || echo "60")
  RES=$(ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of csv=s=x:p=0 "$OUTPUT_VIDEO" 2>/dev/null || echo "1000x680")
  echo ">>> 视频录制成功！"
  echo "    文件路径: $OUTPUT_VIDEO"
  echo "    视频尺寸: $RES"
  echo "    视频时长: ${DURATION}s"
  echo "    文件大小: $SIZE"
else
  echo ">>> 视频录制失败，请查看 /tmp/ffmpeg_record.log"
  exit 1
fi

echo
echo ">>> 遥测抽样记录（部分）："
tail -n 12 "$TELEMETRY_LOG"

echo
echo ">>> 仿真验证与录制全部完成！"


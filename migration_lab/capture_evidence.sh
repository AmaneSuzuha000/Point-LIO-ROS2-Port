#!/bin/bash
# Point-LIO(ROS2) 证据生成脚本 —— 可重复运行
#
# 产物（写入 ../evidence/）：
#   pl_build.log          colcon 编译输出（0 error / 0 warning）
#   pl_node_runtime.log   节点启动日志（IMU 初始化 -> 100%）
#   pl_topics_params.log  node/topic list、topic info、param list/get、topic hz
#   pl_feeder_output.log  离线数据喂入的收发统计与 VERDICT
#   E_rviz2_pointlio.png  rviz2 实际渲染截图（点云 + 路径 + 坐标）
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(dirname "$HERE")/point_lio_ros2"          # 仓库里的迁移后源码（唯一真源）
EV="$(dirname "$HERE")/evidence"
WS=/root/pointlio_ros2_ws
LW=/root/livox2_ws/install/livox_ros_driver2      # 修好 typesupport 命名的 livox 驱动
FEED_SECONDS="${FEED_SECONDS:-60}"

mkdir -p "$EV"

# ---------- 0. 环境 ----------
set +u; source /opt/ros/humble/setup.bash; set -u
export AMENT_PREFIX_PATH="$LW:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="$LW:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$LW/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$LW/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"

# ---------- 1. 同步源码（关键：先删产物再 source ws，避免 stale COLCON_PREFIX_PATH）----------
rm -rf "$WS/build" "$WS/install" "$WS/log"
mkdir -p "$WS/src"
rsync -a --delete "$REPO/" "$WS/src/point_lio/"
echo "[sync] repo -> $WS/src/point_lio"

# ---------- 2. 编译 ----------
cd "$WS"
colcon build --packages-select point_lio --event-handlers console_direct- > "$EV/pl_build.log" 2>&1
BE=$?
{
  echo "BUILD_EXIT=$BE"
  echo "errors:   $(grep -c 'error:' "$EV/pl_build.log")"
  echo "warnings: $(grep -ci 'warning:' "$EV/pl_build.log")"
} >> "$EV/pl_build.log"
tail -3 "$EV/pl_build.log"
[ $BE -eq 0 ] || { echo "BUILD FAILED"; exit 1; }
set +u; source "$WS/install/setup.bash"; set -u

# ---------- 3. 启动节点 ----------
# 先清掉上一轮可能残留的节点：同名旧进程若还在发，ros2 topic info 会报出
# 一个话题多种类型，ros2 hz 直接失败，证据就被污染了（实测踩过，见 §3.9）。
pkill -f pointlio_mapping 2>/dev/null; pkill -f pointlio_feeder 2>/dev/null; pkill -f rviz2 2>/dev/null
sleep 2
mkdir -p /tmp/plrun && cd /tmp/plrun
timeout 130 ros2 launch point_lio mapping_avia.launch.py rviz:=false > "$EV/pl_node_runtime.log" 2>&1 &
NP=$!
sleep 6

# ---------- 4. 话题 / 参数 / 速率 ----------
{
  echo "===== ros2 node list ====="; timeout 10 ros2 node list 2>&1
  echo;  echo "===== ros2 topic list ====="; timeout 10 ros2 topic list 2>&1
  echo;  echo "===== ros2 topic info /livox/lidar -v ====="; timeout 10 ros2 topic info /livox/lidar -v 2>&1
  echo;  echo "===== ros2 topic info /aft_mapped_to_init -v ====="; timeout 10 ros2 topic info /aft_mapped_to_init -v 2>&1
} > "$EV/pl_topics_params.log" 2>&1
{
  echo; echo "===== ros2 param list /point_lio ====="
  timeout 15 ros2 param list /point_lio 2>&1 | tee /tmp/plrun/p.txt | head -14
  echo "..."; echo "TOTAL_PARAMS=$(grep -c . /tmp/plrun/p.txt)"
  echo; echo "===== 抽样 ros2 param get ====="
  # 只查本工程真实存在的键。上一版这里写了 mapping.gyr_bias / mapping.mapping_size，
  # 那是 FAST-LIO 的键名，Point-LIO 根本没有 -> 节点侧回一行 [WARN] Failed to get parameters，
  # 被一起录进 pl_node_runtime.log，看起来像迁移引入的运行时错误。教训见 migration_guide §3。
  for k in mapping.acc_norm preprocess.lidar_type mapping.gravity mapping.imu_time_inte; do
    timeout 8 ros2 param get /point_lio "$k" 2>&1
  done
} >> "$EV/pl_topics_params.log" 2>&1

# ---------- 5. 起 rviz2（在喂数据之前，让点云在屏幕上累积）----------
export DISPLAY="${DISPLAY:-:0}"
RV="$WS/install/point_lio/share/point_lio/rviz_cfg/loam_livox.rviz"
if [ "${SKIP_RVIZ:-0}" != "1" ]; then
  ( timeout 120 rviz2 -d "$RV" > /tmp/plrun/rviz.log 2>&1 ) &
  sleep 12
fi

# ---------- 6. 离线数据喂入（后台跑，速率测量要在它运行期间做）----------
# 上一版把喂数据放在前台、速率测量放在它**结束之后**，于是测到的是一个没有输入的
# 空闲节点：topic hz 报 "does not appear to be published yet"，echo 数到 0 条，
# 证据里留下两行 0 Hz —— 数字看着像节点不发布，其实纯属测量时机错了。
FEED_SECONDS="$FEED_SECONDS" timeout $((FEED_SECONDS + 30)) \
  python3 "$HERE/pointlio_feeder.py" > "$EV/pl_feeder_output.log" 2>&1 &
FP=$!
sleep 18            # 等 IMU 初始化完成 + 开始稳定输出

# ---------- 7. 话题速率（喂数据进行中）----------
{
  # Humble 的 `ros2 topic hz` 没有 QoS 选项，而 /cloud_registered 是 SensorDataQoS
  # (best_effort) 发布 -> hz 用默认 reliable 订阅，收不到任何消息。
  # 改用 `ros2 topic echo --qos-profile sensor_data` 数行数来测速率。
  echo; echo "===== 速率测量：/aft_mapped_to_init（reliable）用 topic hz ====="
  timeout 9 ros2 topic hz /aft_mapped_to_init 2>&1 | head -3
  echo; echo "===== 速率测量：/cloud_registered（best_effort）用 echo --qos-profile sensor_data ====="
  N=$(timeout 9 ros2 topic echo /cloud_registered --qos-profile sensor_data --field width 2>/dev/null | grep -c "^[0-9]")
  HZ=$(awk -v n="$N" "BEGIN{printf \"%.1f\", n/9.0}")
  echo "cloud_registered: $N msgs in 9s -> $HZ Hz"
  echo; echo "===== /aft_mapped_to_init 类型数（必须只有 1 种，>1 说明有残留节点）====="
  timeout 8 ros2 topic info /aft_mapped_to_init 2>&1 | grep -i "type"
} >> "$EV/pl_topics_params.log" 2>&1

wait $FP 2>/dev/null
echo "FEEDER_EXIT=$?" >> "$EV/pl_feeder_output.log"
grep -E "SCANS_SENT|ODOM_MSGS|CLOUD_MSGS|PATH_MSGS|TF_MSGS|VERDICT" "$EV/pl_feeder_output.log"

# ---------- 8. rviz2 截图 ----------
if [ "${SKIP_RVIZ:-0}" != "1" ]; then
  WID=""
  for id in $(xdotool search --maxdepth 2 "" 2>/dev/null); do
    nm=$(xdotool getwindowname "$id" 2>/dev/null)
    case "$nm" in *" - RViz") WID=$id;; esac
  done
  # 注意：xdotool search --name rviz2 会命中 3x3 的 "Qt Selection Owner"，
  # 真正的渲染窗口标题以 " - RViz" 结尾，必须按这个筛。
  if [ -n "$WID" ]; then
    xdotool windowactivate --sync "$WID" 2>/dev/null; sleep 3
    # 把 Displays 与 3D 视图之间的分隔条往左拖：.rviz 里序列化的 QMainWindow 状态
    # 会把左侧面板撑到 ~82%，3D 视图只剩一条，截图没有可读性。
    AX=$(xwininfo -id "$WID" | awk '/Absolute upper-left X/{print $4}')
    AY=$(xwininfo -id "$WID" | awk '/Absolute upper-left Y/{print $4}')
    WW=$(xwininfo -id "$WID" | awk '/^  Width/{print $2}')
    WH=$(xwininfo -id "$WID" | awk '/^  Height/{print $2}')
    xdotool mousemove --sync $((AX + WW * 82 / 100)) $((AY + WH / 2)) \
      mousedown 1 mousemove --sync $((AX + WW * 20 / 100)) $((AY + WH / 2)) mouseup 1
    sleep 2
    import -window "$WID" "$EV/E_rviz2_pointlio.png" 2>/dev/null \
      || scrot -u "$EV/E_rviz2_pointlio.png"
    identify "$EV/E_rviz2_pointlio.png" 2>&1 | head -1
    # 自检：截图不能是全黑（WSLg 抓 GL 窗口有时返回纯黑，那种图没有证据价值）
    python3 - "$EV/E_rviz2_pointlio.png" <<'PY'
import sys
from PIL import Image
import numpy as np
im = np.array(Image.open(sys.argv[1]).convert("RGB")).astype(int)
nb = int((im.sum(2) > 60).sum())
frac = 100.0 * nb / (im.shape[0] * im.shape[1])
print("screenshot non-black %.2f%%" % frac)
if frac < 5.0:
    sys.exit("SCREENSHOT_IS_BLACK - 无证据价值，检查 WSLg/GLX")
PY
  else
    echo "NO RVIZ WINDOW"
  fi
fi

kill $NP 2>/dev/null; pkill -f rviz2 2>/dev/null
echo "CAPTURE_DONE"
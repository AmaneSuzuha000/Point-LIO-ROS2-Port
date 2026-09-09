#!/bin/bash
# Point-LIO 迁移中三个编译器抓不到、只在 ROS2 运行时暴露的参数系统坑。
#
# 用法：bash repro_param_landmines.sh
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
EV="$(dirname "$HERE")/evidence"
WS=/root/pointlio_ros2_ws
LW=/root/livox2_ws/install/livox_ros_driver2
set +u; source /opt/ros/humble/setup.bash; set -u
export AMENT_PREFIX_PATH="$LW:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$LW/lib:${LD_LIBRARY_PATH:-}"
set +u; source "$WS/install/setup.bash"; set -u
BIN="$WS/install/point_lio/lib/point_lio/pointlio_mapping"
CFG="$WS/install/point_lio/share/point_lio/config/avia.yaml"
mkdir -p /tmp/plrun && cd /tmp/plrun

# 造一份上游原样的 YAML：extrinsic_R 写成整数数组
CFG="$CFG" python3 - <<'PY'
import re
import os
src = os.environ["CFG"]
s = open(src, encoding="utf-8").read()
s = re.sub(r"(extrinsic_R:\s*\[)[^\]]*(\])",
           lambda m: m.group(1) + "1, 0, 0, 0, 1, 0, 0, 0, 1" + m.group(2), s)
open("/tmp/plrun/int_array.yaml", "w", encoding="utf-8").write(s)
print("  生成的整数数组行:", [l.strip() for l in s.split("\n") if "extrinsic_R" in l])
PY

{
echo "# Point-LIO ROS2 参数系统三个运行时坑（可复现）"
echo "# 生成脚本：migration_lab/repro_param_landmines.sh"
echo
echo "================================================================"
echo "坑 1  YAML 的顶层节点名选择器不匹配 -> 参数**一条都没加载**，且不报错"
echo "================================================================"
echo "# avia.yaml 第一行是 'point_lio:'，这是 ROS2 的节点名选择器。"
echo "# 但可执行文件叫 pointlio_mapping，直接 ros2 run 时节点名 = pointlio_mapping，"
echo "# 与选择器不相交 -> 整个 params-file 被忽略 -> gravity 等数组参数为空。"
echo "# ROS1 里 \'<rosparam command=\"load\" ns=\"private\"/>\' 无条件作用于本节点私有命名空间，"
echo "# 没有这个坑。"
echo
echo "$ $BIN --ros-args --params-file avia.yaml          # 节点名不匹配"
timeout 8 "$BIN" --ros-args --params-file "$CFG" > a.out 2>&1
echo "  EXIT=$?  (139=SIGSEGV：空 gravity 向量被 Eigen 逗号初始化越界写)"
tail -2 a.out | sed 's/^/  | /'
echo
echo "$ $BIN --ros-args -r __node:=point_lio --params-file avia.yaml   # 修复：对齐节点名"
timeout 8 "$BIN" --ros-args -r __node:=point_lio --params-file "$CFG" > b.out 2>&1
echo "  EXIT=$?  (124=timeout，说明进程正常活着直到被杀)"
head -2 b.out | sed 's/^/  | /'
echo
echo "# 本仓库的 launch 文件里 Node(name='point_lio') 正是为此而写，"
echo "# 见 launch/mapping_avia.launch.py。"
echo
echo "================================================================"
echo "坑 2  YAML 整数数组 vs std::vector<double>：ROS2 强类型，ROS1 会隐式转"
echo "================================================================"
echo "# 上游 config 里 extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]（无小数点）"
echo "# ROS1 nh.param<vector<double>> 会把 int 逐个转 double；ROS2 参数是强类型的。"
echo
echo "$ $BIN --ros-args -r __node:=point_lio --params-file int_array.yaml"
timeout 8 "$BIN" --ros-args -r __node:=point_lio --params-file /tmp/plrun/int_array.yaml > c.out 2>&1
echo "  EXIT=$?  (134=abort，未捕获异常)"
grep -A2 "InvalidParameterType" c.out | head -3 | sed 's/^/  | /'
echo
echo "# 修复：config 里所有浮点数组元素写成带小数点的形式（1.0 而非 1）。"
echo "# 本仓库 4 份 config 已由 migration_lab 的 patch_yaml.py 统一转换，"
echo "# 转换后 56 个参数全部可被 declare_parameter 接受。"
echo
echo "================================================================"
echo "坑 3  重复 declare_parameter：上游把 mapping.lidar_meas_cov 读了两次"
echo "================================================================"
echo "# ROS1 nh.param 重复读同一键合法；ROS2 重复 declare 抛 ParameterAlreadyDeclared。"
grep -n "lidar_meas_cov" "$CFG" | head -2 | sed 's/^/  config: /'
grep -n "lidar_meas_cov" "$WS/src/point_lio/src/parameters.cpp" | head -3 | sed 's/^/  code:   /'
echo
echo "# 修复：get_param() 里先 has_parameter() 再 declare（parameters.cpp:55-57）"
sed -n '48,60p' "$WS/src/point_lio/src/parameters.cpp" | sed 's/^/  | /'
echo
echo "$ ros2 launch point_lio mapping_avia.launch.py rviz:=false   # 对照：修复后可正常启动"
timeout 8 ros2 launch point_lio mapping_avia.launch.py rviz:=false 2>&1 \
  | grep -E "process started|process has died|already declared" | head -3 | sed 's/^/  | /'
} > "$EV/pl_param_landmines.log" 2>&1
echo "wrote $EV/pl_param_landmines.log"
wc -l "$EV/pl_param_landmines.log"
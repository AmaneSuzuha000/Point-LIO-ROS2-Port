#!/usr/bin/env bash
# =============================================================================
# fix_livox_driver.sh —— 修复 livox_ros_driver2 (ROS2) 的 typesupport 库命名 bug
#
# 对应 migration_guide.md §3.B4。真实前后差异见 ../evidence/pl_diff_livox_driver.diff
#
# 【问题】上游 CMakeLists.txt 的 ROS2 分支里写的是
#     set(LIVOX_INTERFACES livox_interfaces2)
#     rosidl_generate_interfaces(${LIVOX_INTERFACES} ...)
# 于是生成的 typesupport 库全部叫 livox_interfaces2__* 。
# 但消息的 C++ 命名空间是 livox_ros_driver2::msg（由包名决定），rclcpp 运行时是按
# **命名空间**拼 dlopen 名字：liblivox_ros_driver2__rosidl_typesupport_fastrtps_cpp.so
# 头文件在 -> 编译能过；.so 名字对不上 -> 一 ros2 run 就 dlopen 失败。
# 即编译绿、运行死，且报错信息里不会提到 CMakeLists，很难定位。
#
# 【修法】把接口目标名改成与包名一致：LIVOX_INTERFACES = livox_ros_driver2。
# 但第一次改立刻撞名：
#     add_library cannot create target "livox_ros_driver2" because another target
#     with the same name already exists
# 因为驱动自己的组件库也叫 ${PROJECT_NAME} = livox_ros_driver2。
# 所以组件库必须一起改名成 livox_driver2_component（含它的全部引用与
# rclcpp_components_register_node 的 EXECUTABLE 名）。
#
# 注意：本脚本只改 ROS2 分支（第 240 行之后）。文件前半段 103-172 行的
# ${PROJECT_NAME}_node 属于 ROS1(catkin) 分支，不能动，所以这里用的是逐条精确
# pattern，而不是对 ${PROJECT_NAME} 做全局替换。
#
# 【用法】
#   bash fix_livox_driver.sh /path/to/src/livox_ros_driver2
#   bash fix_livox_driver.sh                # 默认 /root/livox2_ws/src/livox_ros_driver2
# 幂等：已经是修复态则直接报 ALREADY-FIXED 退出 0。
# =============================================================================
set -euo pipefail

PKG_DIR="${1:-/root/livox2_ws/src/livox_ros_driver2}"
CML="$PKG_DIR/CMakeLists.txt"
BAK="$PKG_DIR/CMakeLists.txt.orig"

[ -f "$CML" ] || { echo "FATAL: not found: $CML" >&2; exit 2; }

# ---------------------------------------------------------------- 幂等检查
if grep -q 'set(LIVOX_INTERFACES livox_ros_driver2)' "$CML"; then
  echo "ALREADY-FIXED: $CML"
  exit 0
fi
grep -q 'set(LIVOX_INTERFACES livox_interfaces2)' "$CML" || {
  echo "FATAL: 未找到上游特征行 set(LIVOX_INTERFACES livox_interfaces2)，" >&2
  echo "       说明这份 CMakeLists 不是预期的上游原版，拒绝修改。" >&2
  exit 3
}

cp -n "$CML" "$BAK" 2>/dev/null || true

# ------------------------------------------------- 9 处精确替换（= 真实 diff）
# 1) 接口目标名 -> 与包名/消息命名空间一致（本修复的核心）
sed -i 's/set(LIVOX_INTERFACES livox_interfaces2)/set(LIVOX_INTERFACES livox_ros_driver2)/' "$CML"

# 2) 组件库改名，给接口目标让出 livox_ros_driver2 这个名字
sed -i 's/ament_auto_add_library(\${PROJECT_NAME} SHARED/ament_auto_add_library(livox_driver2_component SHARED/' "$CML"

# 3-8) 组件库的 6 处引用
sed -i 's/target_include_directories(\${PROJECT_NAME} PRIVATE \${livox_sdk_INCLUDE_DIRS})/target_include_directories(livox_driver2_component PRIVATE \${livox_sdk_INCLUDE_DIRS})/' "$CML"
sed -i 's/target_link_libraries(\${PROJECT_NAME} "\${cpp_typesupport_target}")/target_link_libraries(livox_driver2_component "\${cpp_typesupport_target}")/' "$CML"
sed -i 's/add_dependencies(\${PROJECT_NAME} \${LIVOX_INTERFACES})/add_dependencies(livox_driver2_component \${LIVOX_INTERFACES})/' "$CML"
sed -i 's/target_include_directories(\${PROJECT_NAME} PUBLIC/target_include_directories(livox_driver2_component PUBLIC/' "$CML"
# （这一行原文件里就是跨行的 target_link_libraries(\${PROJECT_NAME} 开头，
#   参数在后续行，所以替换后不能补右括号）
sed -i 's/target_link_libraries(\${PROJECT_NAME}[[:space:]]*$/target_link_libraries(livox_driver2_component/' "$CML"
sed -i 's/rclcpp_components_register_node(\${PROJECT_NAME}/rclcpp_components_register_node(livox_driver2_component/' "$CML"

# 9) 注册出来的可执行文件名（只改 ROS2 分支这一行；
#    103 行 add_executable(${PROJECT_NAME}_node 属 ROS1 分支，保持原样）
sed -i 's/EXECUTABLE \${PROJECT_NAME}_node/EXECUTABLE livox_driver2_component_node/' "$CML"

# ---------------------------------------------------------------- 自校验
# 期望：ROS2 分支内已无裸 ${PROJECT_NAME} 目标引用；ROS1 分支的 add_executable 未被动。
n_ifaces=$(grep -c 'set(LIVOX_INTERFACES livox_ros_driver2)' "$CML" || true)
n_comp=$(grep -c 'livox_driver2_component' "$CML" || true)
n_ros1=$(grep -c 'add_executable(\${PROJECT_NAME}_node' "$CML" || true)

[ "$n_ifaces" = "1" ] || { echo "FATAL: LIVOX_INTERFACES 替换数=$n_ifaces (期望 1)" >&2; exit 4; }
[ "$n_comp" = "8" ]   || { echo "FATAL: livox_driver2_component 出现数=$n_comp (期望 8)" >&2; exit 4; }
[ "$n_ros1" = "1" ]   || { echo "FATAL: ROS1 分支 add_executable 被动了 (=$n_ros1)" >&2; exit 4; }

echo "FIXED: $CML"
echo "  LIVOX_INTERFACES = livox_ros_driver2   (与消息命名空间一致 -> dlopen 能命中)"
echo "  component lib    = livox_driver2_component (8 处引用已同步)"
echo "  ROS1 分支        = 未改动"
echo "  备份             = $BAK"

echo "构建验证（应生成 liblivox_ros_driver2__rosidl_typesupport_*.so）："
echo "  cd <ws> && source /opt/ros/humble/setup.bash && colcon build --packages-select livox_ros_driver2"
echo "  ls install/livox_ros_driver2/lib | grep typesupport"

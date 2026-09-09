#!/bin/bash
# Reproduce the whole task-4 verification from a clean Ubuntu 22.04 (native or WSL2).
#
#   sudo bash build_and_verify.sh
#
# 1. installs ROS2 Humble + the deps this package actually needs
# 2. clones the PINNED upstream ROS1 commit and the official ROS2 branch
# 3. runs migrate_rules.py + extract_core.py to GENERATE the migrated package
# 4. colcon build  -> proves the migration compiles
# 5. starts the node, feeds synthetic IMU+cloud, subscribes to /Odometry
#    -> proves the migrated ESKF / ikd-Tree core actually runs
#
# Every number in ../fastlio_verify.md comes from this script's log.
set -euo pipefail

WS=${WS:-/root/fastlio_verify_ws}
LAB=$(cd "$(dirname "$0")" && pwd)
PKG=$(dirname "$LAB")/fast_lio_ros2          # the delivered package
ROS1_SHA=7cc4175de6f8ba2edf34bab02a42195b141027e9   # "Support MARSIM simulator."
ROS2_SHA=a4743b095409588842a5b30ddfa27e29d2f99164   # official ROS2 branch, for diffing

log() { printf '\n=== %s ===\n' "$*"; }

# ---------------------------------------------------------------- 1. deps ----
if [ ! -d /opt/ros/humble ]; then
  log "installing ROS2 Humble"
  apt-get update
  apt-get install -y software-properties-common curl gnupg lsb-release
  add-apt-repository -y universe
  curl -fsSL https://repo.ros2.org/repos.key | gpg --dearmor -o /usr/share/keyrings/ros2.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2.gpg] https://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list
  apt-get update
  apt-get install -y ros-humble-desktop python3-colcon-common-extensions git build-essential
fi
set +u; source /opt/ros/humble/setup.bash; set -u   # setup.bash refs unset vars
apt-get install -y ros-humble-pcl-conversions ros-humble-pcl-ros ros-humble-tf2-eigen

# livox_ros_driver2 is NOT an apt package. Three facts, each learned by watching
# it fail: (a) it needs the closed-source Livox SDK installed into /usr/local;
# (b) the repo has no package.xml until its own build.sh copies package_ROS2.xml
#     into place, so plain `colcon build` on it always fails;
# (c) its CMakeLists passes LIBRARY_NAME to rosidl_generate_interfaces, which
#     Humble rejects -- drop that one line.
if [ ! -f /usr/local/lib/liblivox_lidar_sdk_static.a ]; then
  log "building the Livox SDK"
  apt-get install -y libpcap-dev cmake
  rm -rf /tmp/Livox-SDK
  git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK.git /tmp/Livox-SDK
  cmake -S /tmp/Livox-SDK -B /tmp/Livox-SDK/build > /dev/null
  cmake --build /tmp/Livox-SDK/build -j"$(nproc)" > /dev/null
  cmake --install /tmp/Livox-SDK/build > /dev/null
fi
# build.sh does `cd ../../` and runs colcon there, and it `rm -rf`s ../../build,
# ../../devel and ../../install. So it MUST be cloned into <ws>/src/<pkg> of a
# throwaway workspace: run it from the clone root and ../../ is `/`, colcon then
# walks the entire filesystem (it found my Windows Android NDK under /mnt/c and
# died there); run it inside $WS/src and it would delete the fast_lio build.
LW=/tmp/livox_ws
if [ ! -d "$LW/install/livox_ros_driver2" ]; then
  log "building livox_ros_driver2 (this takes a few minutes)"
  rm -rf "$LW"   # a run killed mid-clone leaves a partial dir
  mkdir -p "$LW/src"
  git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git "$LW/src/livox_ros_driver2"
  sed -i "/LIBRARY_NAME/d" "$LW/src/livox_ros_driver2/CMakeLists.txt"
  # `build.sh ROS2` leaves DISTRO_ROS empty, so the driver takes its
  # get_target_property() branch and CMake dies on a NOTFOUND include dir.
  # `build.sh humble` sets DISTRO_ROS=humble, which selects the
  # rosidl_get_typesupport_target() branch that Humble needs. Verified:
  # ROS2 -> "1 package failed", humble -> "1 package finished".
  ( cd "$LW/src/livox_ros_driver2" && ./build.sh humble ) 2>&1 | tail -5
  [ -d "$LW/install/livox_ros_driver2" ] || { echo "livox_ros_driver2 did not build"; exit 1; }
fi
export AMENT_PREFIX_PATH=$LW/install/livox_ros_driver2:${AMENT_PREFIX_PATH:-}
export CMAKE_PREFIX_PATH=$LW/install/livox_ros_driver2:${CMAKE_PREFIX_PATH:-}

# ------------------------------------------------------------- 2. upstream ---
log "cloning pinned upstream revisions"
if [ ! -d /tmp/fastlio_ros1/.git ]; then rm -rf /tmp/fastlio_ros1; git clone https://github.com/hku-mars/FAST_LIO.git /tmp/fastlio_ros1; fi
git -C /tmp/fastlio_ros1 fetch --all -q
git -C /tmp/fastlio_ros1 checkout -q "$ROS1_SHA"
test -f /tmp/fastlio_ros1/src/laserMapping.cpp
if [ ! -d /tmp/fastlio_official/.git ]; then rm -rf /tmp/fastlio_official; git clone -b ROS2 https://github.com/hku-mars/FAST_LIO.git /tmp/fastlio_official; fi
git -C /tmp/fastlio_official fetch --all -q
git -C /tmp/fastlio_official checkout -q "$ROS2_SHA"
# ikd-Tree and IKFoM_toolkit are submodules: empty in a plain clone. The build
# needs them, and they are upstream code, not part of the migration deliverable.
git -C /tmp/fastlio_official submodule update --init --recursive -q || true
test -f /tmp/fastlio_official/include/ikd-Tree/ikd_Tree.cpp

# --------------------------------------------------------- 3. generate ------
log "generating the migrated package"
rm -rf "$WS/src/fast_lio"
mkdir -p "$WS/src"
cp -r "$PKG" "$WS/src/fast_lio"
mkdir -p "$WS/src/fast_lio/include/ikd-Tree"
cp /tmp/fastlio_official/include/ikd-Tree/ikd_Tree.h /tmp/fastlio_official/include/ikd-Tree/ikd_Tree.cpp \
   "$WS/src/fast_lio/include/ikd-Tree/"
cp -r /tmp/fastlio_official/include/IKFoM_toolkit "$WS/src/fast_lio/include/"
cp /tmp/fastlio_ros1/include/so3_math.h /tmp/fastlio_ros1/include/Exp_mat.h "$WS/src/fast_lio/include/"
cp /tmp/fastlio_ros1/src/preprocess.h /tmp/fastlio_ros1/src/preprocess.cpp \
   /tmp/fastlio_ros1/src/IMU_Processing.hpp "$WS/src/fast_lio/src/"
cp /tmp/fastlio_ros1/include/common_lib.h /tmp/fastlio_ros1/include/use-ikfom.hpp "$WS/src/fast_lio/include/"
# the mechanical rules, applied to the real ROS1 files, then the core extraction
python3 "$LAB/migrate_rules.py" "$WS/src/fast_lio"
python3 "$LAB/extract_core.py" --ros1 /tmp/fastlio_ros1/src/laserMapping.cpp --out "$WS/src/fast_lio"

# ------------------------------------------------------------- 4. build -----
log "colcon build"
cd "$WS"
rm -rf build install log
colcon build --packages-select fast_lio 2>&1 | tee /tmp/fastlio_colcon.log
test -x "$WS/install/fast_lio/lib/fast_lio/fastlio_mapping"
echo "BUILD_OK"

# ----------------------------------------------------------- 5. run + check -
log "runtime check: node registers, then produces odometry from synthetic data"
set +u; source "$WS/install/setup.bash"; set -u
# --ros-args is mandatory. `ros2 run pkg exe -p a:=1` silently DROPS the -p
# because ros2run consumes it as its own argument -- found by running
# `ros2 param get` and seeing the default come back instead of the value.
# lidar_type=4 is MARSIM: the only PointCloud2 handler in this ROS1 revision
# whose expected layout is a plain pcl::PointXYZI (see feeder_node.py).
timeout 40 ros2 run fast_lio fastlio_mapping --ros-args \
    -p preprocess.lidar_type:=4 \
    -p point_filter_num:=1 \
    -p preprocess.blind:=0.05 \
    -p mapping.filter_size_surf:=0.3 \
    -p mapping.filter_size_map_min:=0.3 \
    -p publish.frequency:=20.0 > /tmp/fastlio_node.log 2>&1 &
NODE=$!
disown 2>/dev/null || true
sleep 6

timeout 70 python3 "$LAB/feeder_node.py" 2>&1 | tee /tmp/fastlio_feeder.log
sleep 2
kill "$NODE" 2>/dev/null || true

log "assertions"
fail=0
grep -q "IMU Initial Done" /tmp/fastlio_node.log && echo "PASS  ESKF initialised" \
  || { echo "FAIL  ESKF never initialised"; fail=1; }
grep -q "VERDICT PASS" /tmp/fastlio_feeder.log && echo "PASS  odometry published and finite" \
  || { echo "FAIL  no usable odometry"; fail=1; }
[ "$(grep -c "Error LiDAR Type" /tmp/fastlio_node.log)" = "0" ] && echo "PASS  lidar handler reached" \
  || { echo "FAIL  lidar handler rejected the cloud"; fail=1; }
[ "$(grep -c "Too few input point cloud" /tmp/fastlio_node.log)" = "0" ] && echo "PASS  cloud non-empty after sync" \
  || { echo "FAIL  empty cloud"; fail=1; }
# The registered-scan topic must carry points, not just exist. It used to be
# created and never published -- `ros2 topic list` showed it, so a topic-list
# check passes while the viewer stays black (fastlio_verify.md 7.3).
awk '/^CLOUD_MAX_PTS/{ok=($2>100)} END{exit !ok}' /tmp/fastlio_feeder.log \
  && echo "PASS  /cloud_registered carries points" \
  || { echo "FAIL  /cloud_registered empty"; fail=1; }
echo "FAILURES $fail"
exit "$fail"
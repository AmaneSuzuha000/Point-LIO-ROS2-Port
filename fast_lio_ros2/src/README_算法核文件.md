# 算法核文件：获取方式与迁移核查（已用真实上游核实）

本目录含 `fastlio_core.cpp`——它是**生成物**：由 `migration_lab/extract_core.py` 从上游
`laserMapping.cpp` 逐字提取算法核 + 机械改写 ROS API 得到（md5
`54b22e39f81eee0dbd2682e69439f0ab`，与真编译那份逐字节相同；重跑生成器得到同一 md5）。
其余算法核文件（下表）是第三方代码，**不入库**，由 `migration_lab/build_and_verify.sh`
从上游 clone 后拷入。上游仓库：https://github.com/hku-mars/FAST_LIO
（本包基线 `7cc4175`；官方另有 `ROS2` 分支 `a4743b0`，架构未拆分，见 `migration_verify.md` §9）

## 真实文件清单（`git clone` 后 `ls` 核实，不是凭印象）

| 文件 | 作用 | 含 ROS1 API？ |
|---|---|---|
| `src/laserMapping.cpp` | 主循环 + 全部 ROS 收发 + `main()`（1055 行） | **是，35 处** |
| `src/IMU_Processing.hpp` | IMU 预积分与运动预测（入口 `ImuProcess::Process`） | **是，8 处** |
| `src/preprocess.cpp` / `.h` | 点云预处理/去畸变 | **是，8 处** |
| `include/common_lib.h` | 公共头（含 `deque<sensor_msgs::Imu::ConstPtr>`） | **是，1 处** |
| `include/ikd-Tree/ikd_Tree.{h,cpp}` | 增量 kd 树 | 否 |
| `include/IKFoM_toolkit/**`（11 个头） | 流形 ESKF 模板库 | 否 |
| `include/{Exp_mat,so3_math,use-ikfom,matplotlibcpp}.h` | 数学工具 | 否 |

> **重要更正**：本文件早期版本列了 `src/ESKF.cpp`、`src/IMU.cpp`、`src/fuse_smoother.cpp`、
> `src/parameters.cpp`、`src/useful_libs.cpp` 五个文件——**上游根本不存在这些文件**。
> 那是凭社区教程印象写的。真实情况：ESKF 在 `IMU_Processing.hpp` + `IKFoM_toolkit` 里，
> 全局参数定义在 `laserMapping.cpp` 顶部（第 91 行起），没有独立 `.cpp`。
> 教训：**文件清单必须 `ls` 出来。**

## 两个必踩的坑

1. **`include/ikd-Tree` 是 git 子模块**，`git clone` 后是空目录。必须：
   ```bash
   git submodule update --init --recursive
   ```
   否则 `CMakeLists.txt` 里的 `include/ikd-Tree/ikd_Tree.cpp` 直接找不到。

2. **`preprocess.h` 依赖 `livox_ros_driver::CustomMsg`**（Livox 雷达自定义消息，非 ROS 标准包）：
   ```cpp
   void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, ...);
   ```
   迁移时要么一并移植该消息包（官方 ROS2 分支用的是 `livox_ros_driver2`），
   要么删掉 Livox 分支只保留 `PointCloud2` 路径。不做这一步，编译直接失败。

## 为什么不能写算法核零改动

社区教程普遍这么说。实测打脸：上表四个文件都有 ROS1 API 泄漏（35/8/8/1 处）。
真实迁移后的签名（取自官方 `ROS2` 分支）：

```cpp
// ROS1
void Reset(double t, const sensor_msgs::ImuConstPtr &lastimu);
// ROS2 官方分支
void Reset(double t, const sensor_msgs::msg::Imu::ConstSharedPtr &lastimu);
```

## 拷贝后必做：grep 粗筛 + 编译终判

```bash
cd fast_lio
PAT='\brosl?os::|\bros::|#include <ros/|ROS_INFO|ROS_WARN|ROS_ERROR|ROS_ASSERT|ROS_DEBUG|->header\.stamp\.toSec|\bNodeHandle\b|\bnh\.|\b_nh\.|dynamic_reconfigure'
grep -cE "$PAT" src/fastlio_core.cpp src/preprocess.{cpp,h} src/IMU_Processing.hpp \
             include/common_lib.h include/use-ikfom.hpp include/so3_math.h include/Exp_mat.h
# 每个文件都应为 0（实测：evidence/api_residue_census.log，CORE_TOTAL 0）
```

这条命令的两个细节，都是实测教出来的：

1. **必须加词边界**。旧版裸写 `ros::` 会把 `ouster_ros::Point` / `velodyne_ros::Point`
   （`preprocess.h` 里自定义点类型的命名空间，与 ROS API 无关）算成泄漏——2 处假阳性。
2. **grep 拦不住 include 形式的泄漏**。`#include <fast_lio/Pose6D.h>` 这类泄漏，
   旧版新版都抓不到，而它恰恰是整个迁移的**第一个编译错误**。所以 grep 只是粗筛，
   **终判是 `colcon build`**——14 条实测错误清单见 `migration_guide.md` §7.2。

## 与迁移主文档的关系

完整的七类改动模板、参数/tf/launch/QoS 细节见上级 `migration_guide.md` §8。
本文件只回答：算法核文件从哪来、真实清单是什么、怎么判断要不要改。
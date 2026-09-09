# ROS1 → ROS2 工程迁移说明

> **迁移对象：Point-LIO**（hku-mars/Point-LIO，ROS1 版）→ **ROS2 Humble**
> 迁移后代码：`point_lio_ros2/`（包名 `point_lio`，可执行 `pointlio_mapping`）

---

## 0. 交付清单

| 交付内容 | 本仓库对应 | 状态 |
|---|---|---|
| 迁移后的代码目录 | `point_lio_ros2/`（51 个文件 / 10766 行；其中**我改过的** `src/` + 自有 `include/` = 3988 行，第三方头 IKFoM + ivox 5704 行原样保留，其余为构建/配置/launch/rviz；故可离线 `colcon build`） | ✅ |
| 迁移说明文档 | 本文件 `migration_guide.md` | ✅ |
| 编译成功截图（终端输出） | `evidence/pl_build.log`（0 error / 0 warning） | ✅ |
| 运行成功截图（终端输出） | `evidence/pl_node_runtime.log` + `evidence/pl_feeder_output.log` | ✅ |
| 运行成功截图（rviz 显示） | `evidence/E_rviz2_pointlio.png` | ✅ |
| 话题/参数实测记录 | `evidence/pl_topics_params.log` | ✅ |
| 真实代码差异 | `evidence/pl_diff_*.diff`（5 个文件，1068 行：point_lio 的 CMakeLists / package.xml / laserMapping / 上游↔迁移整体，外加依赖库修复 `pl_diff_livox_driver.diff`） | ✅ |
| 依赖库问题的修复（可复现） | `migration_lab/fix_livox_driver.sh` + 验证日志 `evidence/pl_fix_livox_verify.log`（脚本从上游原版跑出的结果与实修文件 **sha256 相同**，且幂等） | ✅ |
| 离线测试数据 | `migration_lab/pointlio_feeder.py` 合成 Livox AVIA 数据（无需硬件） | ✅ |
| 一键复跑全部证据 | `migration_lab/capture_evidence.sh` | ✅ |

**另有一份 FAST-LIO 迁移** ( `fast_lio_ros2`)
---

## 1. 原始工程简介及选择的理由

### 1.1 工程简介

**Point-LIO**（港大 Mechatronics Lab / hku-mars）是一个 **LiDAR-IMU 紧耦合里程计**。
它与 FAST-LIO 系列最大的差别在**采样时刻**：

| | FAST-LIO 类 | Point-LIO |
|---|---|---|
| 状态传播时刻 | 每一**帧**点云到达时 | 每一个**点**的采样时刻 |
| IMU 角色 | 帧间预积分 | 高频（4k–8kHz）逐点传播 |
| 运动畸变 | 需去畸变补偿 | **原理上无畸变** |
| 对 IMU 饱和/剧烈振动的鲁棒性 | 一般 | 强（官方测试到 75 rad/s） |

上游 README 自述的五条特性：
> 1. high odometry output frequency, 4k-8kHz.
> 2. robust to IMU saturation and severe vibration, and other aggressive motions (75 rad/s in our test).
> 3. no motion distortion.
> 4. computationally efficient, robust, versatile on public datasets with general motions.
> 5. As a LiDAR odometry, Point-LIO could be used in various autonomous tasks…

论文：*Point-LIO: Robust High-Bandwidth Lidar-Inertial Odometry*, **Advanced Intelligent
Systems**（Wiley），DOI `10.1002/aisy.202200459`。

**代码规模**（`wc -l` 实测上游 `src/`）：算法本体 **3552 行 / 11 个文件**，
其中 `laserMapping.cpp` 1073 行、`preprocess.cpp` 1035 行。上游 `include/` 另有
`common_lib.h` 242 + `so3_math.h` 128 + `matplotlibcpp.h` 2500。
第三方头两棵子树（IKFoM_toolkit 流形 ESKF、ivox 增量体素）不参与 ROS 编译期依赖。


**ROS1 API 分布**（`grep -c` 实测，见 `evidence/api_residue_census.log`）：

| API | 出现次数 | 说明 |
|---|---|---|
| `nh.param<>` | **56** | 全部集中在 `parameters.cpp`，迁移工作量最大的一处 |
| `ros::Publisher` | 13 | 5 个真发布 + 若干注释掉的 |
| `ros::Time` | 11 | 时间戳换算 |
| `tf::` | 6 | 只在 `publish_odometry()` 一个函数里 |
| `ros::NodeHandle` | 3 | `main()` + `readParameters()` 签名 |
| `ros::Subscriber` | 2 | 一个三目表达式 + IMU |
| `ros::Rate` / `ROS_INFO` | 1 / 2 | |
| `dynamic_reconfigure` | **0** | 本工程没有动态重配置，少一类迁移面 |

### 1.2 为什么选它

1. **没有官方 ROS2 支持。
   只有两个分支：`master` 和 `point-lio-with-grid-map`，**没有 ROS2 分支**。
   这条是选择它最硬的理由，证据是上面那两行分支哈希。
2. **迁移面覆盖度合适且能离线验证。** 56 个参数 + 双类型订阅 + tf 广播 + launch +
   构建系统五类都碰到，但没有 VINS-Mono 那种消息自定义 + 多节点 + octomap 的规模；
   而且它的输入是 `CustomMsg`/`PointCloud2` + `Imu`，**可以用脚本合成喂入**

### 1.3 上游依赖与迁移后的对应

| 项 | ROS1 上游 | ROS2 本仓库 | 证据 |
|---|---|---|---|
| 构建 | catkin，`CMAKE_CXX_STANDARD 14` | ament_cmake，**17**（Humble 的 rclcpp 头要求） | `evidence/pl_diff_CMakeLists.diff` |
| 包描述 | format 1，`<buildtool_depend>catkin`，`message_generation`/`message_runtime` | format 3，`<buildtool_depend>ament_cmake`，无消息生成 | `evidence/pl_diff_package.diff` |
| PCL | `find_package(PCL 1.8 REQUIRED)` | `find_package(PCL 1.10 REQUIRED)`（Humble 自带 1.12） | 同上 |
| Livox 消息 | `livox_ros_driver`（ROS1） | `livox_ros_driver2`（ROS2） | `CMakeLists.txt:49` |
| tf | `tf` + `tf/transform_broadcaster.h` | `tf2_ros` + `tf2_geometry_msgs` | `evidence/pl_diff_ros1_vs_ros2.diff` |
| Python 绘图 | `find_package(PythonLibs REQUIRED)` + `matplotlibcpp.h` | **删除** | §3.4 |
| `eigen_conversions` | `common_lib.h` 里 include | **删除**（ROS2 无此包，且全仓库只 include 未调用） | 同上 |
| `msg/LocalSensorExternalTrigger.msg` | 有文件 + `generate_messages()` | **不迁移**：`grep` 全仓库对它的引用 **0 次**，是死依赖 | §3.5 |

### 1.4 包结构 / 重要文件 / 消息定义

**包结构**（包名 `point_lio`，一个可执行 `pointlio_mapping`）：

```
point_lio_ros2/
├── CMakeLists.txt          # ament_cmake；project(point_lio) / add_executable(pointlio_mapping)
├── package.xml             # format 3
├── src/                    # 算法本体 3591 行 / 11 文件
│   ├── laserMapping.cpp    1089  # main + 节点创建 + 订阅/发布 + 融合主循环 + 发布
│   ├── preprocess.cpp      1035  # 点云预处理：去畸变/降采样/多机型 lidar_type 分支
│   ├── Estimator.cpp        401  # ESKF 量测更新与迭代（调用 IKFoM）
│   ├── li_initialization.cpp 354 # 静止初始化（重力对齐、零偏初值）
│   ├── parameters.cpp       183  # 56 个参数的声明与读取（迁移工作量最大处）
│   ├── preprocess.h         156  IMU_Processing.cpp 135  Estimator.h 60
│   ├── IMU_Processing.h      59  parameters.h 84  li_initialization.h 35
├── include/
│   ├── common_lib.h        242  # 全局类型/宏（VEC_FROM_ARRAY 等）+ 消息别名
│   ├── so3_math.h          128  # SO(3) 反对数等数学工具
│   ├── ros2_time_compat.h   27  # ★迁移新增：to_sec/from_sec（见 §3.A2）
│   ├── IKFoM/                     # 第三方：流形上的 ESKF 模板库（原样保留）
│   └── ivox/                      # 第三方：增量体素近邻索引（原样保留）
├── config/    avia.yaml horizon.yaml ouster64.yaml velody16.yaml   # 4 种雷达
├── launch/    mapping_{avia,horizon,ouster64,velody16}.launch.py + gdb_debug_example.launch.py
├── rviz_cfg/  .rviz（ROS1 YAML 方言）→ rviz2 配置（见 §2.8）
└── Log/       # 上游的调试落盘目录。`root_dir` 来自编译期宏 ROOT_DIR
│              #   （CMakeLists.txt:15 = 构建时的源码目录），所以运行时被打开/截断的是
```

第三方两棵子树（IKFoM + ivox）**原样保留、不参与 ROS 编译期依赖**，
所以本目录可以离线 `colcon build`，不需要联网拉依赖。

**重要文件与迁移关系**：

| 文件 | 迁移中的角色 | 对应小节 |
|---|---|---|
| `src/laserMapping.cpp` | 改动最集中：节点句柄、订阅/发布、tf2、时间、消息命名空间全在这 | §2.3 / §2.5 / §2.6，diff 见 `evidence/pl_diff_laserMapping.diff`（512 行） |
| `src/parameters.cpp` | 56 处 `nh.param<>` → `declare_parameter` + `get_parameter` | §2.4 |
| `src/preprocess.{h,cpp}` | 消息命名空间 + include 修复（A3/A9 两个坑的现场） | §3.A |
| `include/common_lib.h` | 消息别名、`Ptr`/`SharedPtr` 之分（A1 坑）、时间函数搬出（A7 坑） | §3.A |
| `include/ros2_time_compat.h` | **本次迁移新增的唯一文件**，带 `#ifndef` 头卫，两个函数都 `inline` | §3.A2 |
| `CMakeLists.txt` / `package.xml` | catkin→ament_cmake、format 1→3、删 PythonLibs | §2.1 / §2.2 |
| `launch/*.launch.py` | XML → Python DSL，并显式 `Node(name="point_lio")` | §2.7 / §3.B1 |
| `config/*.yaml` | 加节点名选择器 + 浮点数组写成 `1.0` | §2.4 / §3.B2 |
| `rviz_cfg/*.rviz` | ROS1 方言 → rviz2 类名与结构 | §2.8 / §3.C11 |

**消息定义说明**（这里说清楚本工程自己定义了什么、用了什么）：

- **本工程不定义任何自定义消息**：`point_lio_ros2/` 下没有 `msg/`、`srv/`、`action/`
  目录，`CMakeLists.txt` 里也没有 `rosidl_generate_interfaces()`。上游那个
  `msg/LocalSensorExternalTrigger.msg` 是死依赖（引用 0 次），迁移时删掉，
  连带删掉 `generate_messages()`（§1.3 末行）。
- **输入消息**（订阅）：
  - `livox_ros_driver2/msg/CustomMsg` — Livox 私有非重复扫描格式，字段
    `header / timebase(uint64) / point_num(uint32) / lidar_id(uint8) / rsvd[3] / points[]`；
    其中 `CustomPoint` = `offset_time(uint32) + x/y/z(float32) + reflectivity/tag/line(uint8)`。
    **`offset_time` 是逐点相对时刻**，正是 Point-LIO 能做逐点传播的输入前提，
    也是它和一帧一个时间戳的 PointCloud2 路线的根本区别。
  - `sensor_msgs/msg/PointCloud2` — 非 Livox 雷达（ouster64/velody16）走这条，
    由 `lidar_type` 参数二选一（§3.A6 那个三目表达式拆不开就是这里）。
  - `sensor_msgs/msg/Imu` — `/livox/imu`，**加速度单位是 g**（feeder 按此合成，
    `acc_norm:1.0`；单位搞错会让重力对齐直接发散）。
- **输出消息**（发布，实测话题见 `evidence/pl_topics_params.log`）：

  | 话题 | 类型 | 说明 |
  |---|---|---|
  | `/aft_mapped_to_init` | `nav_msgs/msg/Odometry` | 里程计主输出（reliable QoS） |
  | `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | 去畸变后的配准点云（best_effort） |
  | `/cloud_registered_body` | `sensor_msgs/msg/PointCloud2` | 机体坐标系版本 |
  | `/Laser_map` | `sensor_msgs/msg/PointCloud2` | 累积地图 |
  | `/path` | `nav_msgs/msg/Path` | 轨迹 |
  | `/tf` | `tf2_msgs/msg/TFMessage` | `camera_init → body`（§2.5 由 `tf` 换 `tf2_ros::TransformBroadcaster`） |

  ROS1→ROS2 在这类消息上的唯一实质差异是**命名空间多了一层 `msg::`**
  （`nav_msgs::Odometry` → `nav_msgs::msg::Odometry`），字段与语义完全兼容。

---

## 2. 每一步修改的内容

迁移不是一次写完的，是**规则表 + 逐处结构改写 + 编译器反馈**三轮迭代。
全部机械改写规则写在 `migration_lab/migrate_pointlio_rules.py`（29 条规则），
结构性改写（规则表达不了的）写在 `patch_structural.py` / `patch_api.py` /
`patch_time.py` / `patch_yaml.py` / `patch_launch.py` / `patch_pcl_ptr.py`。
下面按依赖顺序讲七步。

### 2.1 第一步：工作空间与构建系统

```bash
# ROS1（上游）：catkin
cd catkin_ws && catkin_make

# ROS2（本仓库）：colcon，src/ 结构相同，命令不同
cd ~/pointlio_ros2_ws && colcon build --packages-select point_lio
```

`CMakeLists.txt` 的实质差异（完整 diff 见 `evidence/pl_diff_CMakeLists.diff`，142 行）：

```diff
- find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs
-                                              geometry_msgs nav_msgs tf pcl_ros
-                                              livox_ros_driver message_generation)
- catkin_package(CATKIN_DEPENDS ... message_runtime)
- find_package(PythonLibs REQUIRED)                    # 只服务 matplotlibcpp
- find_package(PCL 1.8 REQUIRED)
- set(CMAKE_CXX_STANDARD 14)
+ find_package(ament_cmake REQUIRED)
+ find_package(rclcpp REQUIRED)
+ find_package(sensor_msgs REQUIRED)   # + nav/geometry/visualization_msgs
+ find_package(pcl_conversions REQUIRED)
+ find_package(tf2_ros REQUIRED)
+ find_package(tf2_geometry_msgs REQUIRED)
+ find_package(livox_ros_driver2 REQUIRED)
+ find_package(PCL 1.10 REQUIRED)
+ set(CMAKE_CXX_STANDARD 17)           # Humble 的 rclcpp 头要求 C++17
+ ament_target_dependencies(pointlio_mapping rclcpp sensor_msgs ... livox_ros_driver2)
+ install(TARGETS pointlio_mapping DESTINATION lib/${PROJECT_NAME})
+ install(DIRECTORY launch config rviz_cfg DESTINATION share/${PROJECT_NAME})
+ ament_package()
```

三个容易漏的点我都踩过：

- **`catkin_package()` 的 INCLUDE_DIRS 语义**在 ament 里没有对应物。上游靠
  `include_directories(include)` 全局生效，我保留了这条（`CMakeLists.txt:54`），
  否则 `common_lib.h` / `IKFoM` / `ivox` 全找不到。
- **`add_definitions(-DROOT_DIR=...)` 必须保留**（`CMakeLists.txt:15`）。
  `Log/` 目录下的调试记录路径靠它拼出来，删了不影响编译但运行时写文件位置不对。
- **`-DMP_EN -DMP_PROC_NUM`** 那套多进程编译定义是上游原样保留的，不是 ROS 相关，
  别在迁移时顺手删掉。

### 2.2 第二步：package.xml format 1 → format 3

```diff
- <package>
-   <buildtool_depend>catkin</buildtool_depend>
-   <build_depend>roscpp</build_depend>
-   <build_depend>tf</build_depend>
-   <build_depend>pcl_ros</build_depend>
-   <build_depend>livox_ros_driver</build_depend>
-   <build_depend>message_generation</build_depend>
-   <run_depend>message_runtime</run_depend>
-   <export></export>
+ <package format="3">
+   <buildtool_depend>ament_cmake</buildtool_depend>
+   <depend>rclcpp</depend>
+   <depend>tf2_ros</depend>
+   <depend>tf2_geometry_msgs</depend>
+   <depend>pcl_conversions</depend>
+   <depend>livox_ros_driver2</depend>
+   <exec_depend>rviz2</exec_depend>
+   <export><build_type>ament_cmake</build_type></export>
```

format 3 的 `<depend>` 一个标签同时覆盖 build/exec，不再需要 `build_depend` +
`run_depend` 写两遍。**`pcl_ros` → `pcl_conversions`** 是必改项：本工程只用
`pcl_conversions::pclFromRosMessage` / `toROSMsg`，不用 pcl_ros 的节点/滤镜封装，
而 ROS2 里 `pcl_ros` 是另一个包（且 Humble 下 `pcl_conversions` 才是正解）。

### 2.3 第三步：节点句柄与收发

**入口签名**：ROS1 传引用，ROS2 一切皆 `SharedPtr`。

```diff
- void readParameters(ros::NodeHandle &nh)
+ void readParameters(rclcpp::Node::SharedPtr &nh)
```

`main()` 里从造一个 NodeHandle变成造一个 Node 对象 + executor：

```diff
- ros::init(argc, argv, "laserMapping");
- ros::NodeHandle nh("~");
- readParameters(nh);
- ...
- ros::Rate loop_rate(1000);
- while (ros::ok()) { ...; ros::spinOnce(); loop_rate.sleep(); }
+ rclcpp::init(argc, argv);
+ auto nh = rclcpp::Node::make_shared("point_lio");   // 名字必须与 YAML 顶层键一致，见 §3.1
+ readParameters(nh);
+ ...
+ rclcpp::executors::SingleThreadedExecutor exec;
+ exec.add_node(nh);
+ while (rclcpp::ok()) { exec.spin_some(); ...; }
```

**订阅：ROS1 的三目表达式在 ROS2 里编译不过。**
diff 见 `evidence/pl_diff_laserMapping.diff:215-230`：

```diff
- ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ?
-     nh.subscribe(lid_topic, 200000, livox_pcl_cbk) :
-     nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
- ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
+ // ROS1 里两种订阅都是同一个类 ros::Subscriber，靠回调签名区分消息类型；
+ // ROS2 里 Subscription<CustomMsg> 与 Subscription<PointCloud2> 是不同类型，
+ // 三目运算符两侧无法统一（error: operands to ?: have different types），
+ // 必须拆成两个具名指针 + if/else。
+ rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_lvx;
+ rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl;
+ if (p_pre->lidar_type == AVIA)
+     sub_lvx = nh->create_subscription<livox_ros_driver2::msg::CustomMsg>(
+                   lid_topic, rclcpp::SensorDataQoS(), livox_pcl_cbk);
+ else
+     sub_pcl = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
+                   lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk);
+ auto sub_imu = nh->create_subscription<sensor_msgs::msg::Imu>(
+                    imu_topic, rclcpp::SensorDataQoS(), imu_cbk);
```

**回调签名**：ROS1 用 `const boost::shared_ptr<const T>&`（写作 `TConstPtr`），
ROS2 用 `const std::shared_ptr<const T>&`（`ConstSharedPtr`）。29 条规则里的
`ConstPtr → ConstSharedPtr`（26 处）、`sensor_msgs::X → sensor_msgs::msg::X` 就是干这个。

**QoS**：ROS1 只有一个队列长度数字 `200000`；ROS2 拆成多维策略。这里选
`rclcpp::SensorDataQoS()`（= KeepLast(5) + best_effort + volatile + 传感器可靠性）

### 2.4 第四步：参数系统（56 处，全在 parameters.cpp）

ROS1 `nh.param<T>(key, var, def)` 一步完成；ROS2 必须**先声明再取**，否则
`get_parameter` 抛 `ParameterNotDeclaredException`。为了保持 56 行 diff 最小，
我包了一个模板把两步合回一行（`src/parameters.cpp:47-60`）：

```cpp
template <typename T>
T get_param(rclcpp::Node::SharedPtr &nh, const std::string &name, const T &default_value)
{
  if (!nh->has_parameter(name))
    nh->declare_parameter(name, default_value);
  return nh->get_parameter(name).get_value<T>();
}
```

```diff
- nh.param<double>("mapping/satu_acc", satu_acc, 3.0);
+ satu_acc = get_param<double>(nh, "mapping.satu_acc", 3.0);
```

**键名里的 `/` 换成 `.`**：ROS1 用斜杠做命名空间分层，ROS2 参数名是扁平字符串，
`mapping/satu_acc` 会被当成一个字面键名（不报错，只是永远取不到值）。

YAML 侧还有两道关卡，错了**不报错**——这是我认为本题最值得写进文档的东西，
完整复现见 `evidence/pl_param_landmines.log`，脚本 `migration_lab/repro_param_landmines.sh`：

```yaml
# ROS2 的 params-file 顶层必须是节点名选择器，ROS1 的 <rosparam> 没有这个概念
point_lio:                    # ← 必须等于节点名，否则整个文件被静默忽略
  ros__parameters:
    mapping:
      satu_acc: 3.0
      extrinsic_R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]   # ← 必须带小数点
```

### 2.5 第五步：tf → tf2

上游 `tf::` 只有 6 处，全部在 `publish_odometry()` 一个函数里
（`evidence/pl_diff_laserMapping.diff:147-169`）：

```diff
- static tf::TransformBroadcaster br;
- tf::Transform   transform;
- tf::Quaternion  q;
- transform.setOrigin(tf::Vector3(pos.x, pos.y, pos.z));
- q.setW(odomAftMapped.pose.pose.orientation.w);
- q.setX(...); q.setY(...); q.setZ(...);
- transform.setRotation(q);
- br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp,
-                                       "camera_init", "body"));
+ // ROS1 tf 广播 -> ROS2 tf2_ros：StampedTransform 拆成 TransformStamped，
+ // 旋转直接拷贝四元数（tf::Quaternion 的 setW/X/Y/Z 在 tf2 里不存在）
+ geometry_msgs::msg::TransformStamped tfMsg;
+ tfMsg.header.stamp     = odomAftMapped.header.stamp;
+ tfMsg.header.frame_id  = "camera_init";
+ tfMsg.child_frame_id   = "body";
+ tfMsg.transform.translation.x = odomAftMapped.pose.pose.position.x;
+ tfMsg.transform.translation.y = odomAftMapped.pose.pose.position.y;
+ tfMsg.transform.translation.z = odomAftMapped.pose.pose.position.z;
+ tfMsg.transform.rotation      = odomAftMapped.pose.pose.orientation;
+ tfBr.sendTransform(tfMsg);
```

两个 ROS2 特有的坑：

1. **`tf2_ros::TransformBroadcaster` 需要节点时钟**，不能像 ROS1 那样 `static` 造在函数里。
   我把它提到 `main()` 创建、按引用传给 `publish_odometry()`（`laserMapping.cpp:327-328`），
   函数签名因此多了一个参数，3 处调用点同步改成
   `publish_odometry(pubOdomAftMapped, tfBr)`（改动记录见 `MIGRATION_LOG_structural.txt`）。
2. **四元数赋值顺序**：ROS1 的 `q.setW/x/y/z` 是四个独立调用，tf2 里
   `geometry_msgs::msg::Quaternion` 是普通 POD 结构，直接整体赋值即可。
   这里**没有**需要 `tf2::Quaternion` → `tf2::fromMsg` 的转换，因为源和目标都是消息类型。

### 2.6 第六步：时间

`ros::Time` 11 处。ROS2 的 `builtin_interfaces::msg::Time` **没有** `toSec()`/`fromSec()`，
必须经 `rclcpp::Time` 中转。我加了一个小兼容头 `include/ros2_time_compat.h`：

```cpp
namespace ros2_time_compat {
inline double to_sec(const builtin_interfaces::msg::Time &t) { return rclcpp::Time(t).seconds(); }
inline builtin_interfaces::msg::Time from_sec(double s)      { return rclcpp::Time(static_cast<int64_t>(s * 1e9)).get_rcl_timestamp(); }
}
```

规则表把两处机械替换掉，数量是 `grep -c` 数出来的，不是估的：

| ROS1 | ROS2 | 处数 | 分布 |
|---|---|---|---|
| `X->header.stamp.toSec()` | `to_sec(X->header.stamp)` | **45** | laserMapping 25 / li_initialization 14 / preprocess 6 |
| `ros::Time().fromSec(s)` | `from_sec(s)` | **8** | laserMapping 7 / li_initialization 1 |

上游 `ros::Time` 一共 11 处，其中 `ros::Time::now()` 只有 1 处且**在注释里**
（`laserMapping.cpp:304`），所以迁移后不需要 `get_clock()->now()`；
剩下 10 处是 `ros::Time().fromSec()`（8 处实际调用）和 2 处函数签名里的
`const ros::Time &ct`（改成 `const rclcpp::Time &ct`，见 `preprocess.h:142`）。
该函数 `Preprocess::pub_func` 在上游就**没有任何活调用点**（唯一的两处调用在
`preprocess.cpp:399-400` 且都是注释），所以改签名不影响行为，只是为了让编译通过。
主循环保持原结构：`ros::Rate loop_rate(500)` → `rclcpp::Rate loop_rate(500)`，
`ros::spinOnce()` → `rclcpp::spin_some(nh)`，`ros::ok()` → `rclcpp::ok()`。
**没有**换成 `SingleThreadedExecutor`——`spin_some` 就是它在 ROS2 里的等价物，
换执行器会改变回调调度语义，属于超出必要范围的改动。

> 这里我**自己制造过一次 bug**：第一版规则产出了 `sec_to_ros_time` 这个函数名，
> 但从未定义它——编译器立刻报了 7 个未声明错误。教训写在 §3.6。

### 2.7 第七步：launch 文件 XML → Python

5 个 launch 全部重写（`patch_launch.py` 保留了原 XML 片段作为注释，便于对照）。
以 `mapping_avia.launch.py` 为例：

```diff
- <launch>
-   <arg name="rviz" default="true"/>
-   <rosparam command="load" file="$(find point_lio)/config/avia.yaml"/>
-   <node pkg="point_lio" type="pointlio_mapping" name="laserMapping"
-         output="screen" ns="/point_lio"/>
-   <node pkg="rviz" type="rviz" name="rviz"
-         args="-d $(find point_lio)/rviz_cfg/loam_livox.rviz" if="$(arg rviz)"/>
- </launch>
+ def generate_launch_description():
+     pkg = get_package_share_directory('point_lio')
+     return LaunchDescription([
+         DeclareLaunchArgument('rviz', default_value='true'),
+         Node(package='point_lio', executable='pointlio_mapping',
+              name='point_lio',                                  # ← 必须与 YAML 顶层键一致
+              parameters=[os.path.join(pkg, 'config', 'avia.yaml')],
+              output='screen'),
+         Node(package='rviz2', executable='rviz2', name='rviz2',
+              arguments=['-d', os.path.join(pkg, 'rviz_cfg', 'loam_livox.rviz')],
+              condition=IfCondition(LaunchConfiguration('rviz'))),
+     ])
```

`<rosparam command="load">` 在 ROS2 没有对应标签——参数改为通过 `parameters=[...]`
传给具体节点。**`name='point_lio'` 这个字段不是可选的装饰**，它决定 YAML 是否生效，
见 §3.1。

### 2.8 rviz 配置迁移

上游 `loam_livox.rviz` 是 ROS1 格式，rviz2 读不了。逐类改名：

```diff
-   - Class: rviz/Displays
+   - Class: rviz_common/Displays
-   - Class: rviz/Views
+   - Class: rviz_common/Views
-   - Class: rviz/Time
+   - Class: rviz_common/Time
-       Class: rviz/PointCloud2
+       Class: rviz_default_plugins/PointCloud2
-       Topic: /cloud_registered
+       Topic:
+         Depth: 5
+         Durability Policy: Volatile
+         History Policy: Keep Last
+         Reliability Policy: Best Effort
+         Value: /cloud_registered
```

`Fixed Frame` 保持 `camera_init`（与 §2.5 广播的父帧一致）。
`Best Effort` 必须显式写：节点的发布端是 `SensorDataQoS()`（best_effort），
rviz 若用默认 reliable 就**收不到任何点云**，界面一片黑，且不报错。

> 这一节我第一版写错过：把面板类名写成 `displays/Displays`（小写包名），
> rviz2 报 `PluginlibFactory: The plugin for class 'displays/Displays' failed to load`，
> 三个面板全空。正确类名要从 `/opt/ros/humble/share/rviz_common/default.rviz` 里抄，
> 不能凭印象。第一版截图因此是3D 视图被挤成一条的废图，已重做。

---

## 3. 遇到的错误和解决方法

分三类：**A 类**写错迁移规则（编译器会告诉你），**B 类**是 ROS1/ROS2 语义差异
（编译全绿、一跑就死），**C 类**是证据链本身是错的（连报错都没有，
只有去核对才会发现）。

每条都标了**出处**：`编译#N` = 第 N 次 `colcon build` 的原始报错；`复现脚本` = 有
可重跑的脚本和退出码；`自查` = 编译器没报，是读头文件/读日志时发现的。

### 3.A 迁移规则自身引入的错误

| # | 错误（原文摘录） | 根因 | 解决 | 出处 |
|---|---|---|---|---|
| A1 | `'SharedPtr' in 'PointCloudXYZI' {aka 'class pcl::PointCloud<pcl::PointXYZINormal>'} does not name a type`（`common_lib.h:144/161`）+ 连带 `base operand of '->' is not a pointer` | 我写了条规则 `::Ptr → ::SharedPtr`。`SharedPtr` 是 **ROS2 消息**的命名习惯，**PCL 1.12 只有 `Ptr`** | 规则拆开：只有 ROS/boost 智能指针转 `SharedPtr`，`pcl::` 的 `Ptr` 原样保留 | 编译#4 |
| A2 | `'to_sec' was not declared in this scope`（`preprocess.cpp:374`） | 规则产出了 `to_sec(...)` 这个调用，但**从没定义过它**。第一版规则用的名字是 `sec_to_ros_time`，我在 grep 普查时就发现它有 7 处调用、0 处定义，改名后**忘了补定义**，于是同一个错误换了个函数名又炸了一次 | 新建 `include/ros2_time_compat.h`，`to_sec` / `from_sec` 各一个 inline | 编译#4（`sec_to_ros_time` 那半是自查） |
| A3 | `expected ',' or '...' before '<' token`（`preprocess.cpp:96`）+ 连带 12 个变量未声明 | `pcl_conversions.h` 只**前向声明** `pcl::PointCloud`，在 `preprocess.h` 那个位置模板还不完整；`deque<...>` 也依赖被我的 DEAD_INC 规则删掉的传递 include | 补 `#include <pcl/point_cloud.h>` 与 `<deque>`；那 12 个未声明全是**同一个根因的级联**——函数签名解析失败后参数全丢 | 编译#5 |
| A4 | `fatal error: glog/logging.h: No such file or directory`（`ivox3d.h:8`） | 上游遗留依赖。实测 glog 在本工程**唯一**用法是 `ivox3d.h:240` 一句被注释掉的 `// LOG(ERROR)`；机器上也没装（`GLOG_MISSING`） | 删掉这个 include，并在原位置留中文注释说明为什么删（不是悄悄删） | 编译#6/#7 |
| A5 | `'PoseStamped' in namespace 'geometry_msgs' does not name a type`（`laserMapping.cpp:52`） | 消息类型名没迁移：ROS2 里在 `geometry_msgs::msg::` 子命名空间 | 改成 `geometry_msgs::msg::PoseStamped` | 编译#8 |
| A6 | `operands to '?:' have different types 'std::shared_ptr<rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg...>>'`（`laserMapping.cpp:381`） | **这条不是我不小心，是 ROS2 真的和 ROS1 不一样**：ROS1 里两种 `ros::Subscriber` 是同一个类型擦除的类，可以三目；ROS2 里 `Subscription<CustomMsg>` 与 `Subscription<PointCloud2>` 是完全无关的两个类型 | 拆成两个具名 `SharedPtr` + `if/else` 各自 `create_subscription`（源码 `laserMapping.cpp:382-391` 的注释就是这个） | 编译#8 |
| A7 | `redefinition of 'builtin_interfaces::msg::Time from_sec(double)'`（`common_lib.h:18`） | 为了把这两个函数搬进独立兼容头，写了段删除旧定义的逻辑，用 `s.find("\n\n", i)` 定位结尾——**但两个函数之间就有一个空行**，于是只删掉了 `to_sec`，`from_sec` 留了下来，和兼容头里的定义撞名 | 删干净残留；兼容头带 `#ifndef POINT_LIO_ROS2_TIME_COMPAT_H` 头卫，两个函数都 `inline` | 编译#5 |
| A8 | `find_package(PythonLibs REQUIRED)` 与 `#include <Python.h>` | 上游 `CMakeLists.txt:43` 有 `PythonLibs REQUIRED`，`parameters.h` 有 `<Python.h>`，都只为 `matplotlibcpp.h` 服务——而那个 include 在 `laserMapping.cpp:15` **本身就是注释**，绘图函数 0 次调用 | 删依赖，`parameters.h:23` 留注释说明 | 编译#3（catkin 阶段）+ 自查 |
| A9 | `pcl_conversions/pcl_conversions.hpp: No such file` | 我按 ROS2`.h`→`.hpp`的直觉改了 include，但这个包在 Humble 里**只有 `.h`** | 回退成 `<pcl_conversions/pcl_conversions.h>`（`preprocess.h:5`） | **自查**：读 `/opt/ros/humble/include` 目录时发现的，编译器没报（我在编译前就改了） |

### 3.B ROS1/ROS2 语义差异造成的运行时错误

共同点：**`colcon build` 全绿，一跑就死**。三条都有可复现脚本与退出码，
见 `evidence/pl_param_landmines.log`（由 `migration_lab/repro_param_landmines.sh` 生成）。

**B1 节点名选择器不匹配 → 参数一条都没加载 → SIGSEGV（exit 139）**

```
$ pointlio_mapping --ros-args --params-file avia.yaml
  EXIT=139   Segmentation fault (core dumped)
$ pointlio_mapping --ros-args -r __node:=point_lio --params-file avia.yaml
  EXIT=124   (timeout：进程正常活着直到被杀)  | lidar_type: 1
```

ROS2 的 params-file 顶层必须是**节点名选择器**（`point_lio:` → `ros__parameters:`）。
可执行文件叫 `pointlio_mapping`，`ros2 run` 默认节点名 = 可执行名 → 选择器不相交 →
**整个 YAML 被静默忽略，一行警告都没有**。接着 `mapping.gravity` 取到默认值 = 空 vector，
`parameters.cpp:135` 的 `p_imu->gravity_ << VEC_FROM_ARRAY(gravity);` 展开成
`gravity[0], gravity[1], gravity[2]`（宏在 `common_lib.h:89`），`operator[]` 不做边界检查，
空 vector 的 `data()` 是 `nullptr` → 解引用空指针。gdb 的崩溃点正是这一行。

ROS1 没有这个概念：`<rosparam command="load" ns="private"/>` 无条件灌进本节点私有命名空间。
修法：launch 里 `Node(name='point_lio')` 显式对齐（`mapping_avia.launch.py`，注释写明原因）。

**B2 YAML 整数数组 vs `vector<double>` → InvalidParameterTypeException（exit 134）**

```
terminate called after throwing an instance of 'rclcpp::exceptions::InvalidParameterTypeException'
  what():  parameter 'mapping.extrinsic_R' has invalid type: ... is of type {double_array},
           setting it to {integer_array} is not allowed.
```

上游 `avia.yaml` 写的是 `extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]`（无小数点）。
ROS1 的 `nh.param<vector<double>>` 逐个隐式转；**ROS2 参数是强类型的，`integer_array ≠ double_array`**。
修法：`patch_yaml.py` 把 4 份 config 里所有浮点数组元素统一写成 `1.0`。
阴险之处：只改一个数字就触发，且崩在启动瞬间，很容易被误判成代码写坏了。

**B3 重复 `declare_parameter` → ParameterAlreadyDeclaredException**

上游 `parameters.cpp` 把 `mapping.lidar_meas_cov` **读了两次**（第 90 行和第 119 行，
上游自己的笔误，两处都是 `laser_point_cov = get_param<double>(...)`）。
ROS1 的 `nh.param` 重复读同一键合法；ROS2 重复 `declare` 直接抛。
修法：`get_param()` 模板里先 `has_parameter()` 再 `declare_parameter()`
（`parameters.cpp:56-57`），这样 56 个参数逐行 diff 保持最小，同时天然免疫上游这个重复读。

**B4 livox 驱动 typesupport 库名对不上 → 运行时 dlopen 失败**

编译能过（头文件在），一 `run` 就报找不到
`liblivox_ros_driver2__rosidl_typesupport_fastrtps_cpp.so`。根因是**上游驱动的 bug**：
包名叫 `livox_ros_driver2`，但 `rosidl_generate_interfaces()` 用的目标是 `livox_interfaces2`，
于是生成的库全叫 `livox_interfaces2__*`；而 rclcpp 是按**消息命名空间**
（`livox_ros_driver2`）拼 dlopen 名字的。
修法：把接口目标名改成与包名一致；第一次改直接撞 target 名
（`add_library cannot create target "livox_ros_driver2" ... already exists`，
因为驱动自己的组件库就叫这个），所以组件库也要一起改名。
这是解决依赖库问题里最真实的一条——**不是 apt 装不到包，是包自己内部名字不一致**。

### 3.C 证据链的错误

这一类**没有任何报错**，全靠核对发现。

| # | 现象 | 根因 | 后果 | 修法 |
|---|---|---|---|---|
| C1 | 逐格校验 0 命中，路径仍穿过障碍夹缝 | 只判目标格自由，允许**切角** | 有尺寸的机器人过不去那条缝 | `corner_free()`：对角移动要求两个直角格均自由 |
| C2 | 报告里的数字和脚本打印的对不上 | 数字是我手算的，不是 `analyze()` 打印的 | 复核无法复现 | 所有引用数字改为由脚本打印，报告标注出处 |
| C3 | 运行日志里两行 `[WARN] Failed to get parameters: mapping.gyr_bias / mapping.mapping_size` | 我的**取证脚本**去查这两个参数，而它们是 FAST-LIO 的键名，Point-LIO 根本没有（全仓库 grep = 0 命中） | 日志看着像迁移引入了运行时错误 | 换成真实存在的键（`mapping.gravity` / `mapping.imu_time_inte`），并在脚本里写明原因 |
| C4 | `ros2 topic hz` 报 `topic [/aft_mapped_to_init] does not appear to be published yet` / `contains more than one type: [PoseStamped, Odometry]` | 我转换 rviz 配置时**加了个上游没有的 Pose 显示**（`rviz_default_plugins/Pose` 订阅 `geometry_msgs/PoseStamped`），却指向节点发 `nav_msgs/Odometry` 的话题 | rviz 一订阅，该话题就被登记成两种类型，`topic hz`/`info` 全部不可用 | 改用 `rviz_default_plugins/Odometry`；实测 rviz2 加载该配置 **0 报错**，`topic info` 只剩 `Type: nav_msgs/msg/Odometry` |
| C5 | 速率证据是 `0 Hz` 和 `does not appear to be published yet` | 我把速率测量放在**喂数据结束之后**，测到的是一个没有输入的空闲节点 | 数字看着像节点不发布，纯属测量时机错 | 喂数据改后台、探测在喂数据进行中做、测完再 `wait`；重测得 odom 9.81 Hz / cloud 9.0 Hz |
| C6 | 截图全黑 / 3D 视图被挤成一条 | `xdotool search --name rviz2` 命中的是 3×3 的 "Qt Selection Owner"；且 `.rviz` 里没有 `QMainWindow State`，左侧面板回退到占 82% | 截图没有证据价值 | 按窗口标题结尾 `" - RViz"` 筛窗口；注入 rviz2 自带默认布局；脚本加非黑像素 < 5% 即失败自检（当前 57.17%） |
| C7 | 仓库里已修好，证据却由旧代码生成 | WSL 的 `src/point_lio` 是**副本**不是软链 | 证据与源码漂移 | `capture_evidence.sh` 每次先 `rsync -a --delete` 再重编译 |
| C8 | `colcon build` 静默找到 0 个包 | 我先 `source install/setup.bash` 再 `rm -rf install/`，留下失效的 `COLCON_PREFIX_PATH` | 构建什么都没做却返回成功 | 调整顺序：先删产物 → 同步 → 编译 → 最后 source |
| C9 | `set -u` 下 `source /opt/ros/humble/setup.bash` 直接失败 | ament 的 setup 脚本会读未定义变量（`AMENT_TRACE_SETUP_FILES: unbound variable`） | 脚本一启动就死 | source 前后用 `set +u` / `set -u` 包住 |


## 4. 最终编译命令和运行命令

### 4.1 先修依赖库

```bash
# 上游 livox_ros_driver2 的 typesupport 库名与消息命名空间不一致，必须先修
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ~/livox2_ws/src/livox_ros_driver2
# 注：该仓库默认分支就是 master（`git branch -r` 实测只有 origin/master，
#     没有 ros2 分支；package.xml version 1.0.0），不要写 `-b ros2`。
bash migration_lab/fix_livox_driver.sh ~/livox2_ws/src/livox_ros_driver2
cd ~/livox2_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select livox_ros_driver2 --event-handlers console_direct-
# 验证：必须看到 liblivox_ros_driver2__rosidl_typesupport_*.so（6 个）
ls install/livox_ros_driver2/lib | grep typesupport
```

实测产物（`evidence/pl_fix_livox_verify.log`）：
```
liblivox_ros_driver2__rosidl_typesupport_cpp.so
liblivox_ros_driver2__rosidl_typesupport_fastrtps_cpp.so      ← 修复前缺的就是这个
liblivox_ros_driver2__rosidl_typesupport_introspection_cpp.so  （等 6 个）
```

### 4.2 编译迁移后的 point_lio

```bash
mkdir -p ~/pointlio_ros2_ws/src
rsync -a --delete point_lio_ros2/ ~/pointlio_ros2_ws/src/point_lio/
cd ~/pointlio_ros2_ws
set +u; source /opt/ros/humble/setup.bash; set -u
export AMENT_PREFIX_PATH="$HOME/livox2_ws/install/livox_ros_driver2:${AMENT_PREFIX_PATH}"
export CMAKE_PREFIX_PATH="$HOME/livox2_ws/install/livox_ros_driver2:${CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$HOME/livox2_ws/install/livox_ros_driver2/lib:${LD_LIBRARY_PATH}"
colcon build --packages-select point_lio --event-handlers console_direct-
source install/setup.bash
```

> `set +u` / `export` 那两处不是装饰：前者是 §3.C9（ament 的 setup 脚本在 `set -u`
> 下会因未定义变量直接失败），后者是 §3.C8（必须先删产物再 source，且驱动要手工挂进
> 搜索路径，否则 `find_package(livox_ros_driver2)` 找不到）。

**实测编译结果**（`evidence/pl_build.log` 末尾）：
```
Summary: 1 package finished [1min 20s]
BUILD_EXIT=0
errors:   0
warnings: 0
```

`grep -c` 复核：`error:` 0 处、`warning:` 0 处、`note:` 5 处。那 5 条 note 全是
同一个来源——boost 的 `BOOST_PRAGMA_MESSAGE` 提醒在全局命名空间声明 bind 占位符
已废弃，由上游 `IKFoM/esekfom.hpp:42` include `<boost/bind.hpp>` 带进来，
**不是我的迁移引入的**，也不属于编译告警（`-Wall` 下 `warning:` 计数为 0）。
我没有为了让日志好看去 `-w` 屏蔽它，而是如实留在日志里并在此说明。

### 4.3 运行（有硬件）

```bash
ros2 launch point_lio mapping_avia.launch.py          # 节点名已在 launch 里设为 point_lio
ros2 launch point_lio mapping_avia.launch.py rviz:=false   # 不带 rviz
```

### 4.4 运行（无硬件，离线合成数据）

```bash
python3 migration_lab/pointlio_feeder.py     # 默认 25s；FEED_SECONDS=60 可加长
```

feeder 直接构造 `livox_ros_driver2::msg::CustomMsg`（10 Hz）与
`sensor_msgs::msg::Imu`（200 Hz），按 §1.4 的字段契约填：逐点 `offset_time`、
加速度单位 g（`acc_norm:1.0`）。所以这条路径同时验证了消息定义迁移是对的。

### 4.5 一键重跑全部证据

```bash
bash migration_lab/capture_evidence.sh
```

它按顺序做：清残留进程 → 删产物 → rsync 同步仓库源码 → 编译 → 起节点 → 后台喂数据 →
**在喂数据进行中**测速率（§3.C6 的教训）→ 抓 rviz2 截图并做非黑像素自检 →
把 5 份日志写进 `evidence/`。

### 4.6 实测运行结果

| 指标 | 实测值 | 证据文件 |
|---|---|---|
| 节点名 | `/point_lio` | `pl_topics_params.log` |
| IMU 初始化 | 1.0% → 19.0% → 56.0% → **100.0%**，`Reset ImuProcess` 正常触发 | `pl_node_runtime.log` |
| 喂入 LiDAR 帧 | **594**（10 Hz × 60 s） | `pl_feeder_output.log` |
| 喂入 IMU 条 | **11996**（200 Hz × 60 s，末条落在窗口外） | 同上 |
| 输出里程计 | **589** 条 | 同上 |
| 输出配准点云 | **589** 条，单帧最大 `max_pts=2722` | 同上 |
| 输出地图 | 1 条，累积 `max_pts=3999` | 同上 |
| 输出轨迹 / TF | 各 **589** 条，末帧 `camera_init -> body` | 同上 |
| `/aft_mapped_to_init` 频率 | **9.558 Hz**（二次采样 9.678 Hz） | `pl_topics_params.log` |
| `/cloud_registered` 频率 | **8.7 Hz**（78 msgs / 9 s，best_effort 用 `echo --qos-profile sensor_data` 测） | 同上 |
| 声明参数总数 | **63**（`TOTAL_PARAMS=63`） | 同上 |
| 参数抽样回读 | `1.0` / `1` / `[0.0, 0.0, -9.81]` / `0.005` | 同上 |
| 话题类型数 | 1（`nav_msgs/msg/Odometry`，>1 即说明有残留节点污染） | 同上 |
| 收敛性（全程极差） | x **0.0054 m** / y **0.0084 m** / z **0.0047 m**，即 60 s 内三轴漂移均 < 9 mm | `pl_feeder_output.log`（`odom x/y/z: … span=`） |
| 判定 | **`VERDICT PASS`**，`FEEDER_EXIT=0` | 同上 |
| rviz2 渲染 | 点云 + 路径 + 坐标可见，非黑像素 **57.17%**（阈值 5%） | `E_rviz2_pointlio.png` |

## 5. 对 ROS1 / ROS2 差异的理解深度

| 层面 | ROS1 | ROS2 | 为什么 ROS2 这样设计 | 我踩到的对应坑 |
|---|---|---|---|---|
| 构建 | catkin（CMake 宏封装） | colcon + ament_cmake | ROS1 的 catkin 把构建逻辑写死在 CMake 宏里，非 CMake 构建系统无法复用；colcon 是**语言/构建系统无关**的调度器，`--packages-select` 支持增量与并行 | §2.1；§3.C8 的 stale `COLCON_PREFIX_PATH` |
| 句柄 | 全局 `ros::init` + 隐式 `NodeHandle` | 显式 `rclcpp::Node`，一切挂在节点对象上 | ROS1 的全局节点使一进程多节点、命名空间隔离都很难做；ROS2 把上下文对象化 | §2.3 |
| 参数 | `nh.param<T>` 弱类型，读时转换 | `declare_parameter<T>` 先声明、**强类型**、后 `get_parameter` | 声明即契约：参数是节点对外接口的一部分，必须能在启动时被静态校验、被 `ros2 param` 工具枚举 | §3.B2（`integer_array ≠ double_array`）、§3.B3（重复 declare 抛异常） |
| 参数文件 | `<rosparam command="load">` 无条件灌进私有 ns | 顶层必须是**节点名选择器**，不匹配则**静默忽略** | 一个 params-file 可同时给多节点配参，代价是选择器写错没有任何提示 | §3.B1（SIGSEGV，exit 139）——**这条是 ROS2 最危险的差异** |
| 时间 | `ros::Time` 内置、与 wall time 混用 | `builtin_interfaces::msg::Time` + `Time::nanoseconds()`；`ros::Time::now()` 受 `/use_sim_time` 语义约束 | 时间要能仿真回放，就必须是消息里可序列化的量，而不是进程内单例 | §2.6；§3.A2/A7 的 `to_sec`/`from_sec` |
| 坐标变换 | `tf`（`TransformBroadcaster`，自带 10s 环形缓冲） | `tf2_ros`（`Buffer` + `TransformListener`/`TransformBroadcaster` 分离） | tf1 的缓冲与查询耦合在一个类里，跨语言（tf2 有 C++/Python 同构实现）时行为不一致；tf2 把存储与查询拆开 | §2.5 |
| 消息类型名 | `nav_msgs::Odometry` | `nav_msgs::msg::Odometry` | 多发行版共存时，`msg::` 子命名空间让同一进程能同时链接不同 ROS 版本生成的消息 | §3.A5 |
| 订阅句柄 | `ros::Subscriber` 是**类型擦除**的，可互相赋值/三目 | `Subscription<T>` 按消息类型模板特化，**不同类型互不可转换** | ROS2 的 QoS 与类型都是编译期一等公民，代价是类型擦除带来的灵活性没了 | §3.A6（三目表达式直接编译不过）——**这条最能体现不是改名字** |
| 智能指针 | `boost::shared_ptr`，习惯写 `Ptr` | `std::shared_ptr`，消息习惯写 `SharedPtr` | 摆脱 boost 依赖 | §3.A1：**但 PCL 1.12 只有 `Ptr` 没有 `SharedPtr`**，一刀切必炸——命名习惯不能跨库套用 |
| 组件化 | nodelet | `rclcpp_components` + 插件 `.so` | 组件要能在进程内动态装载，就必须走插件机制；而插件装载依赖 typesupport 的 **dlopen 命名约定** | §3.B4：驱动内部目标名与命名空间不一致 → 编译绿、运行 dlopen 失败 |
| QoS | 只有 `queue_size` | `SensorDataQoS` 等策略对象（可靠性/历史/期限） | 无线/rosbag 回放/异构网络下，丢帧与阻塞的选择必须由应用按语义声明 | `create_subscription<...>(topic, rclcpp::SensorDataQoS(), cb)`；§3.C6 里 best_effort 话题用 `topic hz` 测不到，得换 `echo --qos-profile sensor_data` |

---

```
evidence/
├── pl_build.log                 # 编译：0 error / 0 warning / BUILD_EXIT=0
├── pl_node_runtime.log          # 启动：IMU 初始化到 100%
├── pl_topics_params.log         # 话题/参数/速率/类型数（§4.6 多数数字来源）
├── pl_feeder_output.log         # 594/11996 → 589×4 + 三轴 span + VERDICT PASS
├── pl_param_landmines.log       # §3.B 三条崩溃的可复现退出码 139/134
├── pl_fix_livox_verify.log      # §3.B4 修复脚本：结果与实修文件 sha256 相同 + 幂等
├── pl_diff_livox_driver.diff    # 依赖库修复的真实 diff（9 处）
├── pl_diff_CMakeLists.diff      # 构建系统 diff（142 行）
├── pl_diff_package.diff         # package.xml diff（68 行）
├── pl_diff_laserMapping.diff    # 主节点 diff（512 行）
├── pl_diff_ros1_vs_ros2.diff    # 上游↔迁移整体 diff（279 行）
├── api_residue_census.log       # §1.1 的 ROS1 API 出现次数普查
├── livox_driver_build.log       # 驱动构建输出
└── E_rviz2_pointlio.png         # rviz2 实际渲染截图
```

复现顺序：`fix_livox_driver.sh`（一次性）→ `capture_evidence.sh`。

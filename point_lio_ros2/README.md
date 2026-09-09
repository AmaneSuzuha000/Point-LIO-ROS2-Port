# Point-LIO（ROS2 Humble 迁移版）

> **本目录是 ROS1 → ROS2 的迁移产物，不是上游原版。**
> 下面从 §1 起是**上游 README 原文**；
> ROS2 的实际构建与运行步骤只看下面这一节。
> 迁移了哪些内容、踩了哪些坑：见 `../migration_guide.md`。

## 0. ROS2 构建与运行（本目录实际用法）

```bash
# 0) 依赖：ROS2 Humble + PCL + Eigen + 修好 typesupport 命名的 livox_ros_driver2
#    驱动必须先构建并 source，否则 point_lio 找不到 livox_ros_driver2 的包配置
source /opt/ros/humble/setup.bash

# 1) 放进工作空间并编译
mkdir -p ~/pointlio_ros2_ws/src && cp -r point_lio_ros2 ~/pointlio_ros2_ws/src/point_lio
cd ~/pointlio_ros2_ws
colcon build --packages-select point_lio --event-handlers console_direct-
source install/setup.bash

# 2) 跑（launch 里已把节点名设为 point_lio，与 YAML 顶层选择器一致——见指南 §3.B1）
ros2 launch point_lio mapping_avia.launch.py

# 3) 没有硬件时，用合成数据喂（capture_evidence.sh 传 60s：594 帧 LiDAR@10Hz + 11996 条 IMU@200Hz，
#    时长由环境变量 FEED_SECONDS 控制：feeder 自身默认 25s，capture_evidence.sh 传 60s）
python3 ../migration_lab/pointlio_feeder.py

# 4) 一键重跑全部证据（rsync 同步源码 -> 重编译 -> 起节点 -> 喂数据 -> 测速率 -> 截图）
bash ../migration_lab/capture_evidence.sh
```

实测输出（`evidence/pl_topics_params.log`，喂数据进行中测得）：
`/aft_mapped_to_init` 9.558 Hz、`/cloud_registered` 8.7 Hz，
`evidence/pl_feeder_output.log` 里 `VERDICT PASS`。

---

# 以下为上游 README 原文
## Point-LIO: Robust High-Bandwidth Lidar-Inertial Odometry

## 1. Introduction

<div align="center">
    <div align="center">
        <img src="https://github.com/hku-mars/Point-LIO/raw/master/image/toc4.png" width = 75% >
    </div>
    <font color=#a0a0a0 size=2>The framework and key points of the Point-LIO.</font>
</div>

**New features:**
1. high odometry output frequency, 4k-8kHz.
2. robust to IMU saturation and severe vibration, and other aggressive motions (75 rad/s in our test).
3. no motion distortion.
4. computationally efficient, robust, versatile on public datasets with general motions. 
5. As a LiDAR odometry, Point-LIO could be used in various autonomous tasks, such as trajectory planning, control, and perception, especially in cases involving very fast ego-motions (e.g., in the presence of severe vibration and high angular or linear velocity) or requiring high-rate odometry output and mapping (e.g., for high-rate feedback control and perception).

**Important notes:**

A. Please make sure the IMU and LiDAR are **Synchronized**, that's important.

B. Please obtain the saturation values of your used IMU (i.e., accelerator and gyroscope), and the units of the accelerator of your used IMU, then modify the .yaml file according to those settings, including values of 'satu_acc', 'satu_gyro', 'acc_norm'. That's improtant.

C. The warning message "Failed to find match for field 'time'." means the timestamps of each LiDAR points are missed in the rosbag file. That is important because Point-LIO processes at the sampling time of each LiDAR point.

D. We recommend to set the **extrinsic_est_en** to false if the extrinsic is given. As for the extrinsic initiallization, please refer to our recent work: [**Robust and Online LiDAR-inertial Initialization**](https://github.com/hku-mars/LiDAR_IMU_Init).

E. If a high odometry output frequency without downsample is required, set ``` publish_odometry_without_downsample ``` as true. Then the warning message of tf "TF_REPEATED_DATA" will pop up in the terminal window, because the time interval between two publish odometery is too small. The following command could be used to suppress this warning to a smaller frequency:

in your catkin_ws/src,

git clone --branch throttle-tf-repeated-data-error git@github.com:BadgerTechnologies/geometry2.git

Then rebuild, source setup.bash, run and then it should be reduced down to once every 10 seconds. If 10 seconds is still too much log output then change the ros::Duration(10.0) to 10000 seconds or whatever you like.

F. If you want to use Point-LIO without imu, set the "imu_en" as false, and provide a predefined value of gavity in "gravity_init" as true as possible in the yaml file, and keep the "use_imu_as_input" as 0.

## **1.1. Developers:**
The codes of this repo are contributed by:
[Dongjiao He (贺东娇)](https://github.com/Joanna-HE) and [Wei Xu (徐威)](https://github.com/XW-HKU)


## **1.2. Related paper**
Our paper is published on Advanced Intelligent Systems(AIS). [Point-LIO](https://onlinelibrary.wiley.com/doi/epdf/10.1002/aisy.202200459), DOI: 10.1002/aisy.202200459


## **1.3. Related video**
Our accompany video is available on **YouTube**.
<div align="center">
    <a href="https://youtu.be/oS83xUs42Uw" target="_blank"><img src="https://github.com/hku-mars/Point-LIO/raw/master/image/final.png" width=60% /></a>
</div>

## 2. What can Point-LIO do?
### 2.1 Simultaneous LiDAR localization and mapping (SLAM) without motion distortion

### 2.2 Produce high odometry output frequence and high bandwidth

### 2.3 SLAM with aggressive motions even the IMU is saturated

# **3. Prerequisites**

## **3.1 Ubuntu and [ROS](https://www.ros.org/)**
We tested our code on Ubuntu20.04 with noetic. Ubuntu18.04 and lower versions have problems of environments to support the Point-LIO, try to avoid using Point-LIO in those systems. Additional ROS package is required:
```
sudo apt-get install ros-xxx-pcl-conversions
```

## **3.2 Eigen**
Following the official [Eigen installation](eigen.tuxfamily.org/index.php?title=Main_Page), or directly install Eigen by:
```
sudo apt-get install libeigen3-dev
```

## **3.3 livox_ros_driver**
Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).

*Remarks:*
- Since the Point-LIO supports Livox serials LiDAR, so the **livox_ros_driver** must be installed and **sourced** before run any Point-LIO luanch file.
- How to source? The easiest way is add the line ``` source $Licox_ros_driver_dir$/devel/setup.bash ``` to the end of file ``` ~/.bashrc ```, where ``` $Licox_ros_driver_dir$ ``` is the directory of the livox ros driver workspace (should be the ``` ws_livox ``` directory if you completely followed the livox official document).

## 4. Build
Clone the repository and catkin_make:

```
    cd ~/$A_ROS_DIR$/src
    git clone https://github.com/hku-mars/Point-LIO.git
    cd Point-LIO
    git submodule update --init
    cd ../..
    catkin_make
    source devel/setup.bash
```
- Remember to source the livox_ros_driver before build (follow 3.3 **livox_ros_driver**)
- If you want to use a custom build of PCL, add the following line to ~/.bashrc
`export PCL_ROOT={CUSTOM_PCL_PATH}`
  <!-- 上游原文这里写成 ```...``` 三个反引号包一行行内代码，会让 Markdown 渲染器
       把后续内容当成未闭合代码块（本文件后半段整篇被吞）。改为单反引号行内代码，
       文字内容未动。 -->

## 5. Directly run

### 5.1 For Avia
Connect to your PC to Livox Avia LiDAR by following  [Livox-ros-driver installation](https://github.com/Livox-SDK/livox_ros_driver), then
```
    cd ~/$Point_LIO_ROS_DIR$
    source devel/setup.bash
    roslaunch point_lio mapping_avia.launch
    roslaunch livox_ros_driver livox_lidar_msg.launch
```
- For livox serials, Point-LIO only support the data collected by the ``` livox_lidar_msg.launch ``` since only its ``` livox_ros_driver/CustomMsg ``` data structure produces the timestamp of each LiDAR point which is very important for Point-LIO. ``` livox_lidar.launch ``` can not produce it right now.
- If you want to change the frame rate, please modify the **publish_freq** parameter in the [livox_lidar_msg.launch](https://github.com/Livox-SDK/livox_ros_driver/blob/master/livox_ros_driver/launch/livox_lidar_msg.launch) of [Livox-ros-driver](https://github.com/Livox-SDK/livox_ros_driver) before make the livox_ros_driver pakage.

### 5.2 For Livox serials with external IMU

mapping_avia.launch theratically supports mid-70, mid-40 or other livox serial LiDAR, but need to setup some parameters befor run:

Edit ``` config/avia.yaml ``` to set the below parameters:

1. LiDAR point cloud topic name: ``` lid_topic ```
2. IMU topic name: ``` imu_topic ```
3. Translational extrinsic: ``` extrinsic_T ```
4. Rotational extrinsic: ``` extrinsic_R ``` (only support rotation matrix)
- The extrinsic parameters in Point-LIO is defined as the LiDAR's pose (position and rotation matrix) in IMU body frame (i.e. the IMU is the base frame). They can be found in the official manual.
5. Saturation value of IMU's accelerator and gyroscope: ```satu_acc```, ```satu_gyro```
6. The norm of IMU's acceleration according to unit of acceleration messages: ``` acc_norm ```

### 5.3 For Velodyne or Ouster (Velodyne as an example)

Step A: Setup before run

Edit ``` config/velodyne.yaml ``` to set the below parameters:

1. LiDAR point cloud topic name: ``` lid_topic ```
2. IMU topic name: ``` imu_topic ``` (both internal and external, 6-aixes or 9-axies are fine)
3. Set the parameter ```timestamp_unit``` based on the unit of **time** (Velodyne) or **t** (Ouster) field in PoindCloud2 rostopic
4. Line number (we tested 16, 32 and 64 line, but not tested 128 or above): ``` scan_line ```
5. Translational extrinsic: ``` extrinsic_T ```
6. Rotational extrinsic: ``` extrinsic_R ``` (only support rotation matrix)
- The extrinsic parameters in Point-LIO is defined as the LiDAR's pose (position and rotation matrix) in IMU body frame (i.e. the IMU is the base frame).
7. Saturation value of IMU's accelerator and gyroscope: ```satu_acc```, ```satu_gyro```
8. The norm of IMU's acceleration according to unit of acceleration messages: ``` acc_norm ```

Step B: Run below
```
    cd ~/$Point_LIO_ROS_DIR$
    source devel/setup.bash
    roslaunch point_lio mapping_velody16.launch
```

Step C: Run LiDAR's ros driver or play rosbag.

### 5.4 PCD file save

Set ``` pcd_save_enable ``` in launchfile to ``` 1 ```. All the scans (in global frame) will be accumulated and saved to the file ``` Point-LIO/PCD/scans.pcd ``` after the Point-LIO is terminated. ```pcl_viewer scans.pcd``` can visualize the point clouds.

*Tips for pcl_viewer:*
- change what to visualize/color by pressing keyboard 1,2,3,4,5 when pcl_viewer is running. 
```
    1 is all random
    2 is X values
    3 is Y values
    4 is Z values
    5 is intensity
```

# **6. Examples**

The example datasets could be downloaded through [onedrive](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/hdj65822_connect_hku_hk/EmRJYy4ZfAlMiIJ786ogCPoBcGQ2BAchuXjE5oJQjrQu0Q?e=igu44W). Pay attention that if you want to test on racing_drone.bag, [0.0, 9.810, 0.0] should be input in 'mapping/gravity_init' in avia.yaml, and set the 'start_in_aggressive_motion' as true in the yaml. Because this bag start from a high speed motion. And for PULSAR.bag, we change the measuring range of the gyroscope of the built-in IMU to 17.5 rad/s. Therefore, when you test on this bag, please change 'satu_gyro' to 17.5 in avia.yaml.

## **6.1. Example-1: SLAM on datasets with aggressive motions where IMU is saturated**
<div align="center">
<img src="https://github.com/hku-mars/Point-LIO/raw/master/image/example1.gif"  width="40%" />
<img src="https://github.com/hku-mars/Point-LIO/raw/master/image/example2.gif"  width="54%" />
</div>

## **6.2. Example-2: Application on FPV and PULSAR**
<div align="center">
<img src="https://github.com/hku-mars/Point-LIO/raw/master/image/example3.gif"  width="58%" />
<img src="https://github.com/hku-mars/Point-LIO/raw/master/image/example4.gif"  width="35%" />
</div>

PULSAR is a self-rotating UAV actuated by only one motor, [PULSAR](https://github.com/hku-mars/PULSAR)

## 7. Contact us
If you have any questions about this work, please feel free to contact me <hdj65822ATconnect.hku.hk> and Dr. Fu Zhang <fuzhangAThku.hk> via email.

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Point-LIO(ROS2) 离线数据验证：合成一个房间场景，按 Livox AVIA 的
CustomMsg 约定喂给迁移后的节点，并检查它是否真的输出了里程计/点云/路径/TF。

为什么必须用 CustomMsg 而不是 PointCloud2：
  config/avia.yaml 里 lidar_type: 1 == AVIA，迁移后的 main() 走
  livox_pcl_cbk 分支；喂 PointCloud2 的话节点根本收不到数据。

三个从源码读出来的硬性约定（写错任何一条，节点会静默地一帧都不处理）：
  1) avia_handler 只接受 points[i].line < N_SCANS(=6) 且
     (tag & 0x30) in {0x10, 0x00} 的点；
  2) offset_time 单位是 ns（源码 curvature = offset_time / 1e6 -> ms）；
  3) IMU 加速度单位是 g：mapping.acc_norm=1.0，而 Estimator 里做
     acc * G_m_s2 / acc_norm，所以静止时应发 z=1.0 而不是 9.81。
"""
import math, os, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav_msgs.msg import Odometry, Path
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

N_SCANS = 6
PTS_PER_SCAN = 4000
LIDAR_HZ = 10.0
IMU_HZ = 200.0


def stamp(sec):
    s = int(sec)
    return RosTime(sec=s, nanosec=int(round((sec - s) * 1e9)))


def make_scan(rng):
    """一帧非重复的房间点云：地面 + 4 面墙 + 顶部，line 字段轮转 0..N_SCANS-1。"""
    m = CustomMsg()
    m.header.frame_id = "lidar"
    m.timebase = 0
    pts = []
    n = PTS_PER_SCAN
    for i in range(n):
        f = rng.integers(0, 5)
        x, y, z = rng.uniform(-6, 6, 3)
        if f == 0:
            z = -0.8
        elif f == 1:
            x = 6.0
        elif f == 2:
            x = -6.0
        elif f == 3:
            y = 6.0
        else:
            z = 2.5
        # 轻微抖动，保证相邻点坐标不同：avia_handler 会丢弃与上一点完全相同的点
        x += float(rng.normal(0, 1e-3)); y += float(rng.normal(0, 1e-3)); z += float(rng.normal(0, 1e-3))
        p = CustomPoint()
        p.offset_time = int(i * (1e9 / (LIDAR_HZ * n)))   # ns，单调递增
        p.x, p.y, p.z = float(x), float(y), float(z)
        p.reflectivity = int(rng.integers(10, 250))
        p.tag = 0            # (tag & 0x30) == 0x00 -> 有效
        p.line = int(i % N_SCANS)
        pts.append(p)
    m.points = pts
    m.point_num = len(pts)
    return m


class Feeder(Node):
    def __init__(self):
        super().__init__("pointlio_feeder")
        self.pub_l = self.create_publisher(CustomMsg, "/livox/lidar", 10)
        self.pub_i = self.create_publisher(Imu, "/livox/imu", 10)
        self.odom, self.cloud, self.map, self.path, self.tf = [], [], [], [], []
        self.create_subscription(Odometry, "/aft_mapped_to_init", lambda m: self.odom.append(m.pose.pose.position), 10)
        self.create_subscription(PointCloud2, "/cloud_registered", lambda m: self.cloud.append(m.width * m.height), 10)
        self.create_subscription(PointCloud2, "/Laser_map", lambda m: self.map.append(m.width * m.height), 10)
        self.create_subscription(Path, "/path", lambda m: self.path.append(len(m.poses)), 10)
        # tf2 在 ROS2 里是 /tf 话题，迁移后的 publish_odometry 通过
        # tf2_ros::TransformBroadcaster 广播 camera_init -> body，这里直接收来验证
        self.create_subscription(TFMessage, "/tf", lambda m: self.tf.extend(m.transforms), 10)


rclpy.init()
nd = Feeder()
rng = np.random.default_rng(7)
WINDOW = float(os.environ.get("FEED_SECONDS", "25.0"))
t0 = time.time()
scan = imu_ct = 0
last_scan = last_imu = -1.0
while rclpy.ok() and (time.time() - t0) < WINDOW:
    now = time.time() - t0
    # IMU 200Hz：静止姿态，加速度 = 1g 朝上（acc_norm=1.0 的单位约定）
    while last_imu < now - 1.0 / IMU_HZ + 1e-9:
        last_imu += 1.0 / IMU_HZ
        if last_imu < 0:
            continue
        m = Imu()
        m.header.frame_id = "imu"
        m.header.stamp = stamp(last_imu)
        m.angular_velocity.x = m.angular_velocity.y = m.angular_velocity.z = 0.0
        m.linear_acceleration.x = 0.0
        m.linear_acceleration.y = 0.0
        m.linear_acceleration.z = 1.0        # g 单位！见文件头说明 (3)
        nd.pub_i.publish(m)
        imu_ct += 1
    # LiDAR 10Hz
    if now - last_scan >= 1.0 / LIDAR_HZ:
        last_scan = now
        s = make_scan(rng)
        s.header.stamp = stamp(now)
        nd.pub_l.publish(s)
        scan += 1
    rclpy.spin_once(nd, timeout_sec=0.002)

print("SCANS_SENT      %d" % scan)
print("IMU_SENT        %d" % imu_ct)
print("ODOM_MSGS       %d" % len(nd.odom))
print("CLOUD_MSGS      %d  max_pts=%s" % (len(nd.cloud), max(nd.cloud) if nd.cloud else 0))
print("MAP_MSGS        %d  max_pts=%s" % (len(nd.map), max(nd.map) if nd.map else 0))
print("PATH_MSGS       %d  max_poses=%s" % (len(nd.path), max(nd.path) if nd.path else 0))
print("TF_MSGS         %d" % len(nd.tf))
for p in nd.odom[:3]:
    print("  odom pos = (%.4f, %.4f, %.4f)" % (p.x, p.y, p.z))
# 漂移统计：静止场景下不发散必须用**全程**的极差来证明，
# 只打印前 3 条不足以支撑60 s 内毫米级不漂这种说法（第一版报告就犯了这个错）。
if nd.odom:
    for ax in ("x", "y", "z"):
        vs = [getattr(p, ax) for p in nd.odom]
        print("  odom %s: first=%+.4f last=%+.4f min=%+.4f max=%+.4f span=%.4f"
              % (ax, vs[0], vs[-1], min(vs), max(vs), max(vs) - min(vs)))
if nd.tf:
    t = nd.tf[-1]
    print("  last tf: %s -> %s  z=%.4f" % (t.header.frame_id, t.child_frame_id, t.transform.translation.z))
finite = all(math.isfinite(v) for p in nd.odom for v in (p.x, p.y, p.z))
ok = (len(nd.odom) >= 5 and len(nd.cloud) >= 5 and max(nd.cloud) > 100
      and len(nd.path) >= 1 and len(nd.tf) >= 1 and finite)
print("VERDICT", "PASS" if ok else "FAIL")
nd.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)

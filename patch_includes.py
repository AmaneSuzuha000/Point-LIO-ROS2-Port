#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""补齐显式头文件包含（考试要求的调整头文件包含路径，解决依赖库问题）。

根因：ROS1 时代 #include <ros/ros.h> 会间接把 <deque>/<vector>/pcl/point_cloud.h
全部拖进来；换成 rclcpp 后这条隐式链断了，于是
  - pcl_conversions.h 里 pcl::PointCloud 只是前向声明，
    typedef pcl::PointCloud<PointType> PointCloudXYZI 得到的是不完整类型，
    PointCloudXYZI::Ptr 直接编译失败；
  - deque 未声明。
必须显式 include，不能指望传递依赖。
"""
import io, os, re
D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "point_lio_ros2")

# ---------- 1. preprocess.h 显式包含 ----------
p = os.path.join(D, "src/preprocess.h")
s = io.open(p, encoding="utf-8").read()
s = s.replace("#include <rclcpp/rclcpp.hpp>\n#include <ros2_time_compat.h>\n#include <pcl_conversions/pcl_conversions.h>\n",
"""#include <rclcpp/rclcpp.hpp>
#include <ros2_time_compat.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <deque>
#include <vector>
#include <cstdint>
#include <cmath>
""")
io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("preprocess.h:", "pcl/point_cloud.h" in s and "deque" in s)

# ---------- 2. common_lib.h 去掉残留的重复 from_sec ----------
p = os.path.join(D, "include/common_lib.h")
s = io.open(p, encoding="utf-8").read()
i = s.find("inline builtin_interfaces::msg::Time from_sec(double s)")
if i > 0:
    j = s.find("\n}\n", i) + 3
    s = s[:i] + s[j:]
io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("common_lib.h from_sec defs:", s.count("from_sec(double"), "| to_sec defs:", s.count("to_sec(const"))

# ---------- 3. 其它文件同样补 deque/vector 显式包含 ----------
for rel in ["src/laserMapping.cpp", "src/li_initialization.h", "src/parameters.h"]:
    p = os.path.join(D, rel)
    s = io.open(p, encoding="utf-8").read()
    if "#include <deque>" not in s:
        s = s.replace("#include <rclcpp/rclcpp.hpp>", "#include <rclcpp/rclcpp.hpp>\n#include <deque>\n#include <vector>", 1)
        io.open(p, "w", encoding="utf-8", newline="\n").write(s)
        print("deque/vector added:", rel)

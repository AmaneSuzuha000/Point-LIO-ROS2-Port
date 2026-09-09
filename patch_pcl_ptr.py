#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
1) 规则 '::Ptr -> ::SharedPtr' 无差别套用是错的：rosidl 生成的消息有 SharedPtr，
   但 PCL 1.12 的 pcl::PointCloud 只有 Ptr（point_cloud.h:413），没有 SharedPtr。
   -> 把 PCL 类型的 ::SharedPtr 全部还原成 ::Ptr，只保留消息类型的 SharedPtr。
2) to_sec()/from_sec() 定义在 common_lib.h，而 preprocess.h 不 include 它
   -> 抽成独立头 include/ros2_time_compat.h，两边都 include。
"""
import io, os, re
D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "point_lio_ros2")

# ---------- 1. 独立时间兼容头 ----------
w = """// ===== ROS1 -> ROS2 时间戳适配 =====
// ROS1 的 ros::Time 自带 toSec()/fromSec()；ROS2 消息头里是
// builtin_interfaces::msg::Time（sec + nanosec 两个整数字段），没有这些方法。
// 单独成头，因为 preprocess.h 并不 include common_lib.h。
#ifndef POINT_LIO_ROS2_TIME_COMPAT_H
#define POINT_LIO_ROS2_TIME_COMPAT_H

#include <builtin_interfaces/msg/time.hpp>
#include <cmath>
#include <cstdint>

inline double to_sec(const builtin_interfaces::msg::Time & t)
{
    return t.sec + 1e-9 * t.nanosec;
}

inline builtin_interfaces::msg::Time from_sec(double s)
{
    builtin_interfaces::msg::Time t;
    t.sec = (int32_t)std::floor(s);
    t.nanosec = (uint32_t)std::round((s - t.sec) * 1e9);
    if (t.nanosec >= 1000000000u) { t.sec += 1; t.nanosec -= 1000000000u; }
    if (t.sec < 0) { t.sec -= 1; t.nanosec += 1000000000u; }
    return t;
}

#endif
"""
io.open(os.path.join(D, "include/ros2_time_compat.h"), "w", encoding="utf-8", newline="\n").write(w)
print("WROTE include/ros2_time_compat.h")

# ---------- 2. 从 common_lib.h 删掉内联版本，改为 include ----------
p = os.path.join(D, "include/common_lib.h")
s = io.open(p, encoding="utf-8").read()
i = s.find("// ===== ROS1->ROS2 时间戳适配 =====")
j = s.find("#include <queue>")
assert i > 0, "helper block missing"
# 删到下一个非 helper 行
end = s.find("\n\n", i)
s = s[:i] + "" + s[end:]
s = s.replace("#include <builtin_interfaces/msg/time.hpp>\n#include <cmath>\n", "")
s = s.replace("#include <so3_math.h>", "#include <so3_math.h>\n#include <ros2_time_compat.h>")
io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("common_lib.h now includes the compat header:", "ros2_time_compat.h" in s, "| to_sec defs left:", s.count("inline double to_sec"))

# ---------- 3. preprocess.h 也 include ----------
p = os.path.join(D, "src/preprocess.h")
s = io.open(p, encoding="utf-8").read()
if "ros2_time_compat.h" not in s:
    s = s.replace("#include <rclcpp/rclcpp.hpp>", "#include <rclcpp/rclcpp.hpp>\n#include <ros2_time_compat.h>")
io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("preprocess.h include added:", "ros2_time_compat.h" in s)

# ---------- 4. PCL 类型还原 ::Ptr ----------
PCL_OWNERS = ("PointCloudXYZI", "PointCloudXYZRGB", "PointCloud<PointType>", "PointCloud<PointTypeRGB>")
tot = 0
for root, _, files in os.walk(D):
    if "IKFoM" in root: continue
    for fn in files:
        if not fn.endswith((".cpp", ".h", ".hpp")): continue
        p = os.path.join(root, fn)
        s = io.open(p, encoding="utf-8").read()
        out = []
        n = 0
        for line in s.split("\n"):
            if any(o in line for o in PCL_OWNERS) and "::SharedPtr" in line:
                line = line.replace("::SharedPtr", "::Ptr"); n += 1
            out.append(line)
        if n:
            io.open(p, "w", encoding="utf-8", newline="\n").write("\n".join(out))
            tot += n
            print("  %-28s 还原 %d 行" % (os.path.relpath(p, D), n))
print("PCL ::SharedPtr -> ::Ptr 共 %d 行" % tot)

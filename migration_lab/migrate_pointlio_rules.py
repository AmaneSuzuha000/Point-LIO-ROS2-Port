#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Point-LIO (ROS1) -> ROS2 Humble
用法：
    python3 migrate_rules.py            # 就地迁移仓库里的 point_lio_ros2/
    python3 migrate_rules.py --check    # 只报告，不写文件
"""
import argparse
import io
import os
import re
import sys

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "point_lio_ros2")
PKG = os.path.normpath(PKG)

# ---------------------------------------------------------------------------
# 规则一：头文件。ROS2 的消息头统一多一层 /msg/ 且后缀改 .hpp。
# ---------------------------------------------------------------------------
INCLUDE_RULES = [
    (r"#include\s*<ros/ros\.h>",                  "#include <rclcpp/rclcpp.hpp>"),
    (r"#include\s*<ros/console\.h>",             "#include <rclcpp/rclcpp.hpp>"),
    (r"#include\s*<sensor_msgs/Imu\.h>",         "#include <sensor_msgs/msg/imu.hpp>"),
    (r"#include\s*<sensor_msgs/PointCloud2\.h>", "#include <sensor_msgs/msg/point_cloud2.hpp>"),
    (r"#include\s*<sensor_msgs/NavSatFix\.h>",   "#include <sensor_msgs/msg/nav_sat_fix.hpp>"),
    (r"#include\s*<nav_msgs/Odometry\.h>",       "#include <nav_msgs/msg/odometry.hpp>"),
    (r"#include\s*<nav_msgs/Path\.h>",           "#include <nav_msgs/msg/path.hpp>"),
    (r"#include\s*<visualization_msgs/Marker\.h>", "#include <visualization_msgs/msg/marker.hpp>"),
    (r"#include\s*<geometry_msgs/Vector3\.h>",   "#include <geometry_msgs/msg/vector3.hpp>"),
    (r"#include\s*<tf/transform_broadcaster\.h>", "#include <tf2_ros/transform_broadcaster.h>"),
    (r"#include\s*<tf/transform_datatypes\.h>",  "#include <tf2/LinearMath/Transform.h>\n#include <geometry_msgs/msg/transform.hpp>"),
    (r"#include\s*<livox_ros_driver/CustomMsg\.h>", "#include <livox_ros_driver2/msg/custom_msg.hpp>"),
    # eigen_conversions 在 ROS2 已被删除。本工程只在 common_lib.h 里 include 了它，
    # 从未调用其中任何函数（grep tf::pointMsgToEigen|poseMsgToEigen|eigenTo* = 0 命中），
    # 所以直接去掉即可，不需要手写等价转换。
    (r"#include\s*<eigen_conversions/eigen_msg\.h>",
     "// (ROS2 无 eigen_conversions；本工程只 include 未调用，已删除)"),
    (r"#include\s*<pcl_conversions/pcl_conversions\.h>",
     "#include <pcl_conversions/pcl_conversions.hpp>"),
]

# ---------------------------------------------------------------------------
# 规则二：消息类型命名空间（多一层 msg::）+ 智能指针别名。
# ---------------------------------------------------------------------------
TYPE_RULES = [
    (r"\blivox_ros_driver::CustomMsg\b", "livox_ros_driver2::msg::CustomMsg"),
    (r"\bsensor_msgs::Imu\b",            "sensor_msgs::msg::Imu"),
    (r"\bsensor_msgs::PointCloud2\b",    "sensor_msgs::msg::PointCloud2"),
    (r"\bnav_msgs::Odometry\b",          "nav_msgs::msg::Odometry"),
    (r"\bnav_msgs::Path\b",              "nav_msgs::msg::Path"),
    (r"\bgeometry_msgs::Vector3\b",      "geometry_msgs::msg::Vector3"),
    (r"\bvisualization_msgs::Marker\b",  "visualization_msgs::msg::Marker"),
    (r"::ConstPtr\b", "::ConstSharedPtr"),
    # pcl::PointCloud<T>::Ptr 也必须改：ROS2 里 PointCloud2::Ptr 与 PCL 的 ::Ptr
    # 同名，统一改成 ::SharedPtr 两边都成立（PCL 的 shared_ptr 别名也叫 SharedPtr）。
    (r"(?<!Const)::Ptr\b", "::SharedPtr"),
]

# ---------------------------------------------------------------------------
# 规则三：日志宏。RCLCPP_* 第一个参数必须是 logger；这里统一用
# rclcpp::get_logger("point_lio")，避免为了拿 node 指针去改一堆函数签名。
# ---------------------------------------------------------------------------
LOG_RULES = [
    (r"\bROS_INFO\s*\(",  'RCLCPP_INFO(rclcpp::get_logger("point_lio"), '),
    (r"\bROS_WARN\s*\(",  'RCLCPP_WARN(rclcpp::get_logger("point_lio"), '),
    (r"\bROS_ERROR\s*\(", 'RCLCPP_ERROR(rclcpp::get_logger("point_lio"), '),
    (r"\bROS_DEBUG\s*\(", 'RCLCPP_DEBUG(rclcpp::get_logger("point_lio"), '),
]

# ---------------------------------------------------------------------------
# 规则四：时间。ROS2 没有 ros::Time().fromSec(s)。
# ---------------------------------------------------------------------------
TIME_RULES = [
    (r"ros::Time\(\)\.fromSec\(", "sec_to_ros_time("),
    (r"\bros::Time::now\(\)", "rclcpp::Clock().now()"),
]

# ---------------------------------------------------------------------------
# 规则五：参数读取。ROS1 的 nh.param<T>(key, var, def) 是"读并写进 var"，
# ROS2 必须先 declare 再 get，且没有这种三参数形式。用一个模板 helper 承接，
# 保持"一行一个参数"的原有结构，diff 才看得清。
# 同时把 ROS1 的斜杠嵌套键改成 ROS2 的点号嵌套键（common/lid_topic ->
# common.lid_topic），因为 ROS2 的 YAML 参数文件是按点号扁平化的。
# ---------------------------------------------------------------------------
PARAM_RE = re.compile(
    r'nh\.param<([A-Za-z_][\w:<>\s,]*?)>\(\s*"([^"]+)"\s*,\s*([^,]+?)\s*,\s*(.+?)\)\s*;')

# 少数几处 ROS1 写法不带显式模板参数，单独兜底
PARAM_RE2 = re.compile(
    r'nh\.param\(\s*"([^"]+)"\s*,\s*([^,]+?)\s*,\s*(.+?)\)\s*;')


def fix_key(k):
    return k.replace("/", ".")


ALL_SIMPLE = INCLUDE_RULES + TYPE_RULES + LOG_RULES + TIME_RULES

TARGETS = ["src/laserMapping.cpp", "src/li_initialization.cpp", "src/parameters.cpp",
           "src/preprocess.cpp", "src/IMU_Processing.cpp", "src/Estimator.cpp",
           "src/parameters.h", "src/preprocess.h", "src/IMU_Processing.h",
           "src/Estimator.h", "src/li_initialization.h",
           "include/common_lib.h", "include/so3_math.h"]


def apply(text, rules, counts):
    for pat, rep in rules:
        text, n = re.subn(pat, rep, text)
        counts[pat] = counts.get(pat, 0) + n
    return text


def apply_params(text, counts):
    def sub1(m):
        counts["nh.param<T>()"] = counts.get("nh.param<T>()", 0) + 1
        t, k, var, d = m.groups()
        return f'{var} = get_param<{t}>(nh, "{fix_key(k)}", {d});'
    def sub2(m):
        counts["nh.param()"] = counts.get("nh.param()", 0) + 1
        k, var, d = m.groups()
        return f'{var} = get_param(nh, "{fix_key(k)}", {d});'
    text = PARAM_RE.sub(sub1, text)
    text = PARAM_RE2.sub(sub2, text)
    return text


def run(check_only=False):
    counts = {}
    changed = []
    for rel in TARGETS:
        p = os.path.join(PKG, rel)
        if not os.path.exists(p):
            print("  SKIP (missing):", rel)
            continue
        src = io.open(p, encoding="utf-8", errors="replace").read()
        out = apply(src, ALL_SIMPLE, counts)
        out = apply_params(out, counts)
        if out != src:
            changed.append(rel)
            if not check_only:
                io.open(p, "w", encoding="utf-8").write(out)
    print("=== 规则命中统计 ===")
    for pat, rep in ALL_SIMPLE:
        print("  %-56s x%-3d -> %s" % (pat[:54], counts.get(pat, 0), rep[:40]))
    for k in ("nh.param<T>()", "nh.param()"):
        print("  %-56s x%-3d" % (k, counts.get(k, 0)))
    zero = [p for p, _ in ALL_SIMPLE if counts.get(p, 0) == 0]
    if zero:
        print("\n0 命中的规则（本工程不存在该写法，属正常，列出以防误判）：")
        for z in zero:
            print("   ", z)
    print("\n改动文件：", ", ".join(changed))
    return counts


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    run(a.check)
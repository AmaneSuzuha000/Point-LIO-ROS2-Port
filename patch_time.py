#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""时间戳 API（ros::Time 的 toSec/fromSec 在 ROS2 中不存在）+ 兜底清扫。"""
import io, os, re, sys
D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "point_lio_ros2")

def rw(rel, fn):
    p = os.path.join(D, rel)
    s = io.open(p, encoding="utf-8").read()
    s2 = fn(s)
    io.open(p, "w", encoding="utf-8", newline="\n").write(s2)
    return s, s2

# ---------- common_lib.h: 注入 ROS2 时间换算工具 ----------
HELPERS = """
// ===== ROS1->ROS2 时间戳适配 =====
// ROS1 的 ros::Time 自带 toSec()/fromSec()；ROS2 消息头里是
// builtin_interfaces::msg::Time（sec + nanosec 两个整数字段），没有这些方法。
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
"""
def add_helpers(s):
    anchor = "#include <queue>\n"
    assert s.count(anchor) == 1, "queue anchor"
    s = s.replace(anchor, anchor + "#include <builtin_interfaces/msg/time.hpp>\n#include <cmath>\n" + HELPERS)
    return s
rw("include/common_lib.h", add_helpers)

# ---------- 全量替换 toSec()/sec_to_ros_time ----------
TOSEC = re.compile(r"([A-Za-z_][A-Za-z0-9_]*(?:\.front\(\))?(?:->|\.)header\.stamp)\.toSec\(\)")
def fix_time(s):
    n1 = len(TOSEC.findall(s))
    s, n2 = TOSEC.subn(r"to_sec(\1)", s)
    n3 = s.count("sec_to_ros_time(")
    s = s.replace("sec_to_ros_time(", "from_sec(")
    print("  toSec->to_sec %d, sec_to_ros_time->from_sec %d" % (n2, n3))
    return s
for f in ["src/laserMapping.cpp", "src/li_initialization.cpp", "src/preprocess.cpp", "src/IMU_Processing.cpp", "src/parameters.cpp"]:
    print(f); rw(f, fix_time)

# ---------- 兜底清扫：任何残留 ROS1 标识 ----------
bad = []
for root, _, files in os.walk(D):
    if "IKFoM" in root or "ivox" in root: continue
    for fn in files:
        if not fn.endswith((".cpp", ".h", ".hpp")): continue
        p = os.path.join(root, fn)
        for i, line in enumerate(io.open(p, encoding="utf-8", errors="replace"), 1):
            if re.search(r"\brosspy\b|\bros::|\bROS_INFO\b|\bROS_WARN\b|\bROS_ERROR\b|\btf::|\bboost::|\bmessage_filters\b|\bdynamic_reconfigure\b|\.toSec\(\)|\brosparam\b", line):
                if "ROS1" in line or "迁移" in line or "删除" in line: continue
                bad.append("%s:%d: %s" % (os.path.relpath(p, D), i, line.strip()))
print("=== RESIDUAL ===")
print("\n".join(bad) if bad else "(none)")

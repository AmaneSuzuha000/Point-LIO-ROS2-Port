#!/usr/bin/env python3
"""Mechanical ROS1 -> ROS2 migration of FAST-LIO algorithm-core files.

This is the recipe documented in migration_guide.md sections 3 and 6, applied to
the real upstream files, so the recipe gets falsified by a compiler instead of by
my reading of it.
"""
import io
import re
import sys

W = sys.argv[1] if len(sys.argv) > 1 else "/root/t4try_ws/src/fast_lio_ros2"
FILES = [
    "include/common_lib.h",
    "src/IMU_Processing.hpp",
    "src/preprocess.h",
    "src/preprocess.cpp",
]

MSG_PKG = {"sensor_msgs", "nav_msgs", "geometry_msgs", "std_msgs"}
# ROS1-only headers whose symbols are not actually referenced (verified by grep)
DEAD_INC = re.compile(r"^#include <(?:tf/[a-z_/.]+|eigen_conversions/[a-z_/.]+)>[ \t]*\r?\n", re.M)
INC = re.compile(r"#include <([a-z_0-9]+)/([A-Z][A-Za-z0-9]*)\.h>")


def snake(name):
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


def fix_include(m):
    pkg, typ = m.group(1), m.group(2)
    if pkg == "fast_lio":
        return "#include <fast_lio/msg/pose6_d.hpp>"
    if pkg in MSG_PKG:
        return "#include <%s/msg/%s.hpp>" % (pkg, snake(typ))
    return m.group(0)


changed = 0
for rel in FILES:
    p = "%s/%s" % (W, rel)
    try:
        s = io.open(p, encoding="utf-8").read()
    except OSError as e:
        print("SKIP %s (%s)" % (rel, e))
        continue
    orig = s

    # rules the compiler proved were missing, matching what the official ROS2
    # branch actually does to these files:
    s = s.replace("#include <ros/ros.h>", "#include <rclcpp/rclcpp.hpp>")
    s = s.replace("#include <livox_ros_driver/CustomMsg.h>",
                  "#include <livox_ros_driver2/msg/custom_msg.hpp>")
    s = re.sub(r"^\s*ros::Publisher\s+\w[\w,\s]*;[ \t]*$",
               lambda m: "// " + m.group(0).strip() + "   // ROS2: templated, owned by the node",
               s, flags=re.M)
    s = re.sub(r"\blivox_ros_driver::([A-Za-z0-9_]+)\b", r"livox_ros_driver2::msg::\1", s)
    s = re.sub(r"\bros::Subscriber\b", "/* ROS2: rclcpp::Subscription<T>::SharedPtr */", s)
    s = re.sub(r"\bros::ServiceServer\b", "/* ROS2: rclcpp::Service<T> */", s)
    s = re.sub(r"\bros::Duration\b", "rclcpp::Duration", s)
    s = re.sub(r"\bros::Rate\b", "rclcpp::Rate", s)
    s = re.sub(r"\bros::ok\b", "rclcpp::ok", s)
    s = re.sub(r"\bROS_ERROR\s*\(", "RCLCPP_ERROR(rclcpp::get_logger(\"fast_lio\"), ", s)
    s = re.sub(r"\bROS_DEBUG\s*\(", "RCLCPP_DEBUG(rclcpp::get_logger(\"fast_lio\"), ", s)
    s = re.sub(r"\bpcl::fromROSMsg\b", "pcl::fromROSMsg", s)
    # ROS_ASSERT is a ROS1 macro; the official ROS2 branch uses plain assert()
    s = re.sub(r"\bROS_ASSERT\s*\(", "assert(", s)
    s = re.sub(r"\bROS_ERROR\s*\(", "RCLCPP_ERROR(rclcpp::get_logger(\"fast_lio\"), ", s)
    # upstream common_lib.h uses deque<> without including <deque>; it only
    # compiles because laserMapping.cpp happens to include <deque> first. Once
    # the code is split into separate translation units that accident breaks, so
    # make the header self-sufficient instead of relying on include order.
    if rel.endswith("common_lib.h") and "#include <deque>" not in s:
        s = s.replace("#include <so3_math.h>",
                      "#include <deque>\n#include <mutex>\n#include <vector>\n#include <string>\n#include <cmath>\n#include <rclcpp/rclcpp.hpp>\n#include <so3_math.h>", 1)
        # The official ROS2 branch defines these two helpers at the END of
        # common_lib.h, INSIDE the header guard, as plain (non-inline) functions --
        # safe only because upstream keeps everything in one translation unit.
        # This migration splits the core into several .cpp files, so a non-inline
        # definition in a header would be a duplicate symbol at link time, and
        # appending after #endif puts the helpers outside the guard so a second
        # include redefines them. Both mistakes were caught by the compiler.
        helpers = ("\n// added by the migration: helpers the upstream expects from ros/time.h\n"
                   "inline double get_time_sec(const builtin_interfaces::msg::Time &time)\n"
                   "{\n    return rclcpp::Time(time).seconds();\n}\n\n"
                   "inline rclcpp::Time get_ros_time(double timestamp)\n"
                   "{\n    int32_t sec = std::floor(timestamp);\n"
                   "    auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;\n"
                   "    uint32_t nanosec = nanosec_d;\n"
                   "    return rclcpp::Time(sec, nanosec);\n}\n\n")
        guard = s.rfind("#endif")
        if guard < 0:
            s = s.rstrip() + "\n" + helpers
        else:
            s = s[:guard] + helpers + s[guard:]
    # ROS1 stamps are seconds-as-double via toSec(); ROS2 stamps are
    # builtin_interfaces Time structs with no toSec(). The official branch
    # rewrites every one of these inline as rclcpp::Time(...).seconds() --
    # including in preprocess.cpp, which never sees common_lib.h.
    # The character class must allow () because real call sites read
    # v_imu.front()->header.stamp.toSec(); omitting () left 2 errors behind.
    # A permissive character class here is a trap: with "(" allowed, the lazy
    # capture swallowed the keyword in `if(head->header.stamp.toSec())` and
    # produced `rclcpp::Time(if(head->header.stamp).seconds()`. A real receiver
    # grammar fixes it: identifier, then only .member / ->member / .call() --
    # so `if` cannot be a receiver (next char is "(") and the engine restarts at
    # `head`. v_imu.front() and imu_buffer.back() still match.
    RECV = r"[A-Za-z_]\w*(?:\.[A-Za-z_]\w*(?:\([^()]*\))?|->[A-Za-z_]\w*)*"
    s = re.sub("(" + RECV + r")->header\.stamp\.toSec\(\)",
               r"rclcpp::Time(\1->header.stamp).seconds()", s)
    s = re.sub("(" + RECV + r")\.header\.stamp\.toSec\(\)",
               r"rclcpp::Time(\1.header.stamp).seconds()", s)
    s = DEAD_INC.sub("", s)
    s = INC.sub(fix_include, s)
    s = re.sub(r"\b(sensor_msgs|nav_msgs|geometry_msgs|std_msgs)::([A-Za-z0-9_]+)ConstPtr\b",
               r"\1::msg::\2::ConstSharedPtr", s)
    # (?!msg) + [A-Z] keep this from re-matching its own output: the earlier
    # version of this rule let the engine backtrack "msg" -> "ms" and produced
    # sensor_msgs::msg::msg::Imu, which the compiler then read as a type named
    # "msg". Caught by the build, not by reading.
    s = re.sub(r"\b(sensor_msgs|nav_msgs|geometry_msgs|std_msgs)::(?!msg\b)([A-Z][A-Za-z0-9_]*)(?!::)",
               r"\1::msg::\2", s)
    # ROS2 has no ::ConstPtr on generated messages
    s = re.sub(r"\b(sensor_msgs|nav_msgs|geometry_msgs|std_msgs|livox_ros_driver2)::msg::([A-Za-z0-9_]+)::ConstPtr\b",
               r"\1::msg::\2::ConstSharedPtr", s)
    # non-const ::Ptr alias, same reason as above; scoped so pcl typedefs survive
    s = re.sub(r"\b(sensor_msgs|nav_msgs|geometry_msgs|std_msgs|livox_ros_driver2)::msg::([A-Za-z0-9_]+)::Ptr\b",
               r"\1::msg::\2::SharedPtr", s)
    s = re.sub(r"\bfast_lio::([A-Z][A-Za-z0-9_]+)(?!:)", r"fast_lio::msg::\1", s)
    s = s.replace("ros::Time", "rclcpp::Time")
    s = s.replace("ros::Duration", "rclcpp::Duration")
    s = s.replace("ros::Publisher", "rclcpp::Publisher")
    s = re.sub(r"\bROS_INFO\s*\(", 'RCLCPP_INFO(rclcpp::get_logger("fast_lio"), ', s)
    s = re.sub(r"\bROS_WARN\s*\(", 'RCLCPP_WARN(rclcpp::get_logger("fast_lio"), ', s)

    if s != orig:
        io.open(p, "w", encoding="utf-8", newline="\n").write(s)
        changed += 1
        print("patched %s" % rel)

print("files_changed %d" % changed)
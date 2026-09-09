// ===== ROS1 -> ROS2 时间戳适配 =====
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

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
1) Humble 里 pcl_conversions 只有 .h（无 .hpp）。
2) rclcpp 没有 ParameterValueFromType / Parameter::get_as；真实 API 是
   declare_parameter(name, default) + get_parameter(name).get_value<T>()。"""
import io, os, re
D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "point_lio_ros2")

def rw(rel, fn):
    p = os.path.join(D, rel)
    s = io.open(p, encoding="utf-8").read()
    io.open(p, "w", encoding="utf-8", newline="\n").write(fn(s))

n = 0
for root, _, files in os.walk(D):
    for fn in files:
        if not fn.endswith((".cpp", ".h", ".hpp")): continue
        p = os.path.join(root, fn)
        s = io.open(p, encoding="utf-8").read()
        if "pcl_conversions/pcl_conversions.hpp" in s:
            io.open(p, "w", encoding="utf-8", newline="\n").write(
                s.replace("pcl_conversions/pcl_conversions.hpp", "pcl_conversions/pcl_conversions.h"))
            n += 1
print("pcl_conversions include fixed in %d files" % n)

BAD = """template <typename T>
T get_param(rclcpp::Node::SharedPtr &nh, const std::string &name, const T &default_value)
{
  nh->declare_parameter(name, rclcpp::ParameterValueFromType(default_value));
  return nh->get_parameter(name).get_as<T>();
}"""
GOOD = """template <typename T>
T get_param(rclcpp::Node::SharedPtr &nh, const std::string &name, const T &default_value)
{
  // ROS1: nh.param<T>(key, var, def) ROS2 必须先声明再取，
  // 否则 get_parameter 抛 ParameterNotDeclaredException。
  nh->declare_parameter(name, default_value);
  return nh->get_parameter(name).get_value<T>();
}"""
def fix_gp(s):
    if BAD in s: return s.replace(BAD, GOOD)
    if "ParameterValueFromType" in s:
        raise SystemExit("FAIL: get_param body drifted, manual fix needed")
    return s
rw("src/parameters.cpp", fix_gp)
print("get_param fixed:", "ParameterValueFromType" not in io.open(os.path.join(D,"src/parameters.cpp"),encoding="utf-8").read())

# 检查 vector 型参数（ROS1 nh.param<vector<double>> 需要 rclcpp 的 double_array）
s = io.open(os.path.join(D,"src/parameters.cpp"),encoding="utf-8").read()
print("vector get_param types:", sorted(set(re.findall(r"get_param<\s*(std::vector<[^>]+>)", s))))
print("remaining get_as/ParameterValueFromType:", s.count("get_as"), s.count("ParameterValueFromType"))

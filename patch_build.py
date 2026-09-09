#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""构建系统（catkin->ament）、ROS2 参数 YAML、launch 文件、残留头文件。"""
import io, os
D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "point_lio_ros2")
def w(rel, text):
    p = os.path.join(D, rel)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    io.open(p, "w", encoding="utf-8", newline="\n").write(text)
    print("WROTE", rel)

# ---------- get_param 幂等声明 ----------
p = os.path.join(D, "src/parameters.cpp")
s = io.open(p, encoding="utf-8").read()
s = s.replace("  nh->declare_parameter(name, default_value);",
"""  // 上游 parameters.cpp 把 mapping.lidar_meas_cov 读了两次，ROS2 重复声明会抛
  // ParameterAlreadyDeclaredException，所以先 has_parameter 再声明
  if (!nh->has_parameter(name))
    nh->declare_parameter(name, default_value);""")
io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("get_param idempotent-declare OK")

# ---------- parameters.h: 去掉未使用的 Python.h ----------
p = os.path.join(D, "src/parameters.h")
s = io.open(p, encoding="utf-8").read()
s = s.replace("#include <Python.h>\n", "// (迁移删除：#include <Python.h> 只服务于 matplotlibcpp，本工程从未调用)\n")
io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("Python.h removed")

# ---------- laserMapping.cpp: TransformStamped 头文件 ----------
p = os.path.join(D, "src/laserMapping.cpp")
s = io.open(p, encoding="utf-8").read()
if "transform_stamped.hpp" not in s:
    s = s.replace("#include <geometry_msgs/msg/transform.hpp>",
                  "#include <geometry_msgs/msg/transform.hpp>\n#include <geometry_msgs/msg/transform_stamped.hpp>\n#include <rclcpp/rclcpp.hpp>")
    io.open(p, "w", encoding="utf-8", newline="\n").write(s)
print("transform_stamped include OK")

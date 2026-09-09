# 自定义消息：ROS1 的 msg/VIO.msg 语法在 ROS2 中完全兼容，仅头文件包含方式变化
# ROS1: #include "fast_lio/VIO.h"   ->  ROS2: #include "fast_lio/msg/vio.hpp"
std_msgs/Header header
geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Accel accel

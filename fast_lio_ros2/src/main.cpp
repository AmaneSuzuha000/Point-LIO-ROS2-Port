// main.cpp —— ROS1 -> ROS2 迁移：rclcpp 初始化入口
// 原版(ROS1):
//   ros::init(argc, argv, "fastlio_mapping");
//   ros::NodeHandle nh("~");
//   FAST_LIO_Cpp_Node n(nh);
//   ros::spin();
#include <rclcpp/rclcpp.hpp>
#include "ros2node.h"

int main(int argc, char ** argv)
{
    // 【迁移点】ROS2 必须先 rclcpp::init，结束时要显式 shutdown
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FAST_LIO_Cpp_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

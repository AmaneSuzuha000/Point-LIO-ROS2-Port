#pragma once
// ros2node.h -- FAST-LIO publish/subscribe node (ROS2 Humble).
//
// Structural differences from the ROS1 version, all of them documented in
// migration_guide.md section 3:
//   1. the node inherits rclcpp::Node (ROS1 used free functions + a NodeHandle)
//   2. ros::Subscriber -> rclcpp::Subscription<Msg>::SharedPtr
//   3. nh.subscribe(...) -> create_subscription<...>(...)
//   4. dynamic_reconfigure -> declare_parameter + get_parameter_or
//   5. tf::TransformBroadcaster -> tf2_ros::TransformBroadcaster
//   6. ros::spinOnce() in a while loop -> a wall timer driving one scan per tick
//
// This file includes ONLY fastlio_core.h. It never includes common_lib.h,
// use-ikfom.hpp or IMU_Processing.hpp: those upstream headers carry non-inline
// definitions, so keeping them in a single translation unit is what makes the
// core/node split linkable at all.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "fastlio_core.h"
#include "fast_lio/msg/vio.hpp"

class FAST_LIO_Cpp_Node : public rclcpp::Node
{
public:
    FAST_LIO_Cpp_Node();  // NOLINT -- plain ctor keeps the .cpp definition signature identical

private:
    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg);
    void step();          // one full FAST-LIO scan per timer tick
    void publishOdometry(const double s[13], const rclcpp::Time & stamp);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRegisteredCloud;
    rclcpp::Publisher<fast_lio::msg::VIO>::SharedPtr pubVIO;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    rclcpp::TimerBase::SharedPtr stepTimer;
    nav_msgs::msg::Path path_;
    bool path_en_ = true;
    bool scan_pub_en_ = true;
};

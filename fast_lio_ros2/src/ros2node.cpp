// ros2node.cpp -- the ROS2 I/O shell around the FAST-LIO algorithm core.
//
// Every rclcpp call in the migrated package lives in this file. That is the
// point of the split: run the grep from migration_guide.md section 8 over the
// algorithm core and it comes back empty, because nothing ROS-facing is in it.
#include "ros2node.h"
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

using namespace std;  // NOLINT build

FAST_LIO_Cpp_Node::FAST_LIO_Cpp_Node() : Node("fast_lio")
{
    // ---- parameters. ROS1 read these from the param server / dynamic_reconfigure.
    //      ROS2 declares them on the node, which is what makes them overridable
    //      from the launch file (see launch/mapping.launch.py).
    this->declare_parameter<double>("mapping.gyr_cov", 0.1);
    this->declare_parameter<double>("mapping.acc_cov", 0.1);
    this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
    this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
    this->declare_parameter<double>("preprocess.blind", 0.01);
    this->declare_parameter<int>("preprocess.lidar_type", 1);
    this->declare_parameter<int>("preprocess.scan_line", 16);
    this->declare_parameter<int>("preprocess.timestamp_unit", 2);
    this->declare_parameter<int>("preprocess.scan_rate", 10);
    this->declare_parameter<int>("point_filter_num", 2);
    this->declare_parameter<int>("max_iteration", 4);
    this->declare_parameter<double>("mapping.filter_size_surf", 0.5);
    this->declare_parameter<double>("mapping.filter_size_map_min", 0.5);
    this->declare_parameter<double>("mapping.cube_side_length", 200.0);
    this->declare_parameter<double>("mapping.det_range", 300.0);
    this->declare_parameter<double>("mapping.fov_degree", 180.0);
    this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
    this->declare_parameter<bool>("publish.path_en", true);
    // Upstream laserMapping.cpp:762-763 reads these two with a `true` default;
    // without them /cloud_registered is created but never fed (see 7.3).
    this->declare_parameter<bool>("publish.scan_publish_en", true);
    this->declare_parameter<bool>("publish.dense_publish_en", true);
    this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
    this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());
    this->declare_parameter<string>("common.lid_topic", "/lidar_points");
    this->declare_parameter<string>("common.imu_topic", "/imu");
    this->declare_parameter<double>("publish.frequency", 10.0);

    CoreParams p;
    this->get_parameter_or<double>("mapping.gyr_cov", p.gyr_cov, p.gyr_cov);
    this->get_parameter_or<double>("mapping.acc_cov", p.acc_cov, p.acc_cov);
    this->get_parameter_or<double>("mapping.b_gyr_cov", p.b_gyr_cov, p.b_gyr_cov);
    this->get_parameter_or<double>("mapping.b_acc_cov", p.b_acc_cov, p.b_acc_cov);
    this->get_parameter_or<double>("preprocess.blind", p.blind, p.blind);
    this->get_parameter_or<int>("preprocess.lidar_type", p.lidar_type, p.lidar_type);
    this->get_parameter_or<int>("preprocess.scan_line", p.scan_line, p.scan_line);
    this->get_parameter_or<int>("preprocess.timestamp_unit", p.time_unit, p.time_unit);
    this->get_parameter_or<int>("preprocess.scan_rate", p.scan_rate, p.scan_rate);
    this->get_parameter_or<int>("point_filter_num", p.point_filter_num, p.point_filter_num);
    this->get_parameter_or<int>("max_iteration", p.max_iteration, p.max_iteration);
    this->get_parameter_or<double>("mapping.filter_size_surf", p.filter_size_surf, p.filter_size_surf);
    this->get_parameter_or<double>("mapping.filter_size_map_min", p.filter_size_map_min, p.filter_size_map_min);
    this->get_parameter_or<double>("mapping.cube_side_length", p.cube_len, p.cube_len);
    this->get_parameter_or<double>("mapping.det_range", p.det_range, p.det_range);
    this->get_parameter_or<double>("mapping.fov_degree", p.fov_deg, p.fov_deg);
    this->get_parameter_or<bool>("mapping.extrinsic_est_en", p.extrinsic_est_en, p.extrinsic_est_en);
    this->get_parameter_or<bool>("publish.path_en", p.path_en, p.path_en);
    this->get_parameter_or<bool>("publish.scan_publish_en", p.scan_publish_en, p.scan_publish_en);
    this->get_parameter_or<bool>("publish.dense_publish_en", p.dense_publish_en, p.dense_publish_en);
    path_en_ = p.path_en;
    scan_pub_en_ = p.scan_publish_en;
    vector<double> t3 = {0, 0, 0}, r9 = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    this->get_parameter_or<vector<double>>("mapping.extrinsic_T", t3, t3);
    this->get_parameter_or<vector<double>>("mapping.extrinsic_R", r9, r9);
    if (t3.size() == 3) p.extrinT = t3;
    if (r9.size() == 9) p.extrinR = r9;

    core_init(p);

    // ---- QoS. FAST-LIO consumes a 200Hz+ IMU stream; the ROS2 default QoS is
    //      reliable with depth 10, which drops or blocks at that rate. See 3.3.
    rclcpp::QoS sensor_qos(rclcpp::KeepLast(2000));
    sensor_qos.best_effort();

    const string lidar_topic = this->get_parameter("common.lid_topic").as_string();
    const string imu_topic = this->get_parameter("common.imu_topic").as_string();
    subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic, sensor_qos,
        std::bind(&FAST_LIO_Cpp_Node::cloudHandler, this, std::placeholders::_1));
    subImu = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, sensor_qos,
        std::bind(&FAST_LIO_Cpp_Node::imuHandler, this, std::placeholders::_1));

    pubOdometry = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
    pubPath = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
    pubRegisteredCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);
    pubVIO = this->create_publisher<fast_lio::msg::VIO>("/fast_lio/vio", 20);
    tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ---- ROS1 spun a while loop on a condition variable; ROS2 drives one scan
    //      per timer tick so the executor owns the scheduling.
    double freq = 10.0;
    this->get_parameter_or<double>("publish.frequency", freq, freq);
    stepTimer = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / max(freq, 0.1))),
        std::bind(&FAST_LIO_Cpp_Node::step, this));

    RCLCPP_INFO(this->get_logger(), "FAST-LIO ROS2 node ready: lidar=%s imu=%s publish=%.1fHz",
                lidar_topic.c_str(), imu_topic.c_str(), freq);
}

void FAST_LIO_Cpp_Node::cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    core_lidar_pc2(msg);
}

void FAST_LIO_Cpp_Node::imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    core_imu(msg);
}

void FAST_LIO_Cpp_Node::step()
{
    double s[13] = {0};
    if (!core_step(this->get_clock()->now().seconds(), s)) {
        return;
    }
    publishOdometry(s, this->get_clock()->now());

    // Upstream publishes the registered scan from the same code path that
    // publishes odometry (laserMapping.cpp:981). core_step() returning true
    // means a full scan was just aligned, so the world-frame cloud is fresh.
    if (scan_pub_en_) {
        sensor_msgs::msg::PointCloud2 cloud;
        if (core_registered_cloud(cloud) > 0) {
            pubRegisteredCloud->publish(cloud);
        }
    }
}

void FAST_LIO_Cpp_Node::publishOdometry(const double s[13], const rclcpp::Time & stamp)
{
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "camera_init";
    odom.child_frame_id = "body";
    odom.header.stamp = stamp;
    odom.pose.pose.position.x = s[0];
    odom.pose.pose.position.y = s[1];
    odom.pose.pose.position.z = s[2];
    odom.pose.pose.orientation.x = s[9];
    odom.pose.pose.orientation.y = s[10];
    odom.pose.pose.orientation.z = s[11];
    odom.pose.pose.orientation.w = s[12];
    odom.twist.twist.linear.x = s[3];
    odom.twist.twist.linear.y = s[4];
    odom.twist.twist.linear.z = s[5];
    pubOdometry->publish(odom);

    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "camera_init";
    t.child_frame_id = "body";
    t.header.stamp = stamp;
    // ROS2 splits Point and Vector3 into distinct types (ROS1 shared them via
    // tf:: poses); the assignment that compiled under ROS1 needs field copies.
    t.transform.translation.x = odom.pose.pose.position.x;
    t.transform.translation.y = odom.pose.pose.position.y;
    t.transform.translation.z = odom.pose.pose.position.z;
    t.transform.rotation = odom.pose.pose.orientation;
    tfBroadcaster->sendTransform(t);

    fast_lio::msg::VIO vio;
    vio.header.stamp = stamp;
    vio.pose = odom.pose.pose;
    vio.twist = odom.twist.twist;
    pubVIO->publish(vio);

    if (path_en_) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = "camera_init";
        ps.header.stamp = stamp;
        ps.pose = odom.pose.pose;
        path_.poses.push_back(ps);
        pubPath->publish(path_);
    }
}

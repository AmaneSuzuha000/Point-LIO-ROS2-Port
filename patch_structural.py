#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Point-LIO ROS1->ROS2 结构化补丁（正则规则表之外的部分）。
每条补丁 = (文件, 唯一锚点原文, 替换文本)。锚点不唯一/不存在即报错退出，
"""
import io, os, sys
D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "point_lio_ros2")
LOG = []

def patch(rel, old, new, count=1):
    p = os.path.join(D, rel)
    s = io.open(p, encoding="utf-8").read()
    if old not in s and new in s:
        LOG.append("%s: SKIP (already applied) %r" % (rel, old[:40]))
        return
    n = s.count(old)
    if n != count:
        print("FAIL %s: anchor count=%d (want %d) for %r" % (rel, n, count, old[:60]))
        sys.exit(1)
    io.open(p, "w", encoding="utf-8", newline="\n").write(s.replace(old, new))
    LOG.append("%s: %d x %r -> %r" % (rel, count, old[:40], new[:40]))

# ---------- 1. parameters.cpp: get_param 模板 + readParameters 签名 ----------
patch("src/parameters.cpp",
"""void readParameters(ros::NodeHandle &nh)
{""",
"""// ROS2 参数读取：ROS1 的 nh.param<T>(key, var, def) 在 rclcpp 中声明与读取分离，
template <typename T>
T get_param(rclcpp::Node::SharedPtr &nh, const std::string &name, const T &default_value)
{
  nh->declare_parameter(name, rclcpp::ParameterValueFromType(default_value));
  return nh->get_parameter(name).get_as<T>();
}

void readParameters(rclcpp::Node::SharedPtr &nh)
{""")

patch("src/parameters.h",
"void readParameters(ros::NodeHandle &n);",
"void readParameters(rclcpp::Node::SharedPtr &n);")

# ---------- 2. preprocess.h: 悬空 publisher 成员与 pub_func 声明 ----------
patch("src/preprocess.h",
"""  bool given_offset_time;
  ros::Publisher pub_full, pub_surf, pub_corn;
""",
"""  bool given_offset_time;
  // ROS1 的 pub_full/pub_surf/pub_corn 从未被赋值或使用，迁移时删除
""")
patch("src/preprocess.h",
"  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);",
"  void pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct);")

# ---------- 3. preprocess.cpp: pub_func 定义 ----------
patch("src/preprocess.cpp",
"void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)",
"void Preprocess::pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct)")

# ---------- 4. laserMapping.cpp: publish_* 签名与 .publish() ----------
patch("src/laserMapping.cpp", "void publish_init_map(const ros::Publisher & pubLaserCloudFullRes)",
    "void publish_init_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)")
patch("src/laserMapping.cpp", "void publish_frame_world(const ros::Publisher & pubLaserCloudFullRes)",
    "void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)")
patch("src/laserMapping.cpp", "void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)",
    "void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull_body)")
patch("src/laserMapping.cpp", "void publish_odometry(const ros::Publisher & pubOdomAftMapped)",
    "void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pubOdomAftMapped,\n                           tf2_ros::TransformBroadcaster & tfBr)")
patch("src/laserMapping.cpp", "void publish_path(const ros::Publisher pubPath)",
    "void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)")
patch("src/laserMapping.cpp", "pubLaserCloudFullRes.publish(laserCloudmsg);",
    "pubLaserCloudFullRes->publish(laserCloudmsg);", 2)
patch("src/laserMapping.cpp", "pubLaserCloudFull_body.publish(laserCloudmsg);",
    "pubLaserCloudFull_body->publish(laserCloudmsg);")
patch("src/laserMapping.cpp", "pubOdomAftMapped.publish(odomAftMapped);",
    "pubOdomAftMapped->publish(odomAftMapped);")
patch("src/laserMapping.cpp", "pubPath.publish(path);", "pubPath->publish(path);")

# ---------- 5. tf::TransformBroadcaster -> tf2_ros ----------
patch("src/laserMapping.cpp",
"""    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \\
                                    odomAftMapped.pose.pose.position.y, \\
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body") );""",
"""    // ROS1 tf 广播 -> ROS2 tf2_ros：StampedTransform 拆成 TransformStamped，
    // 旋转直接拷贝四元数（tf::Quaternion 的 setW/X/Y/Z 接口在 tf2 中不存在）
    geometry_msgs::msg::TransformStamped tfMsg;
    tfMsg.header.stamp = odomAftMapped.header.stamp;
    tfMsg.header.frame_id = "camera_init";
    tfMsg.child_frame_id = "body";
    tfMsg.transform.translation.x = odomAftMapped.pose.pose.position.x;
    tfMsg.transform.translation.y = odomAftMapped.pose.pose.position.y;
    tfMsg.transform.translation.z = odomAftMapped.pose.pose.position.z;
    tfMsg.transform.rotation = odomAftMapped.pose.pose.orientation;
    tfBr.sendTransform(tfMsg);""")

# ---------- 6. main(): 节点/订阅/发布/自旋 ----------
patch("src/laserMapping.cpp",
"""    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(0);
    spinner.start();""",
"""    rclcpp::init(argc, argv);
    // ROS1 私有句柄 nh("~") -> ROS2 节点：参数名直接挂在节点上，语义等价
    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("laser_mapping");
    // tf2 广播器需要节点时钟，在 main 创建、按引用传给 publish_odometry
    tf2_ros::TransformBroadcaster tfBr(nh);""")

patch("src/laserMapping.cpp",
"""    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \\
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \\
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);""",
"""    // ROS1 回调签名即订阅类型；ROS2 用 create_subscription<Message> 显式模板参数
    auto sub_pcl = p_pre->lidar_type == AVIA ? \\
        nh->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 200000, livox_pcl_cbk) : \\
        nh->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, 200000, standard_pcl_cbk);
    auto sub_imu = nh->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 200000, imu_cbk);""")

patch("src/laserMapping.cpp",
"""    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::msg::PointCloud2>
            ("/cloud_registered", 1000);
    ros::Publisher pubLaserCloudFullRes_body = nh.advertise<sensor_msgs::msg::PointCloud2>
            ("/cloud_registered_body", 1000);""",
"""    auto pubLaserCloudFullRes = nh->create_publisher<sensor_msgs::msg::PointCloud2>
            ("/cloud_registered", 1000);
    auto pubLaserCloudFullRes_body = nh->create_publisher<sensor_msgs::msg::PointCloud2>
            ("/cloud_registered_body", 1000);""")
patch("src/laserMapping.cpp",
"""    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::msg::PointCloud2>
            ("/Laser_map", 1000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::msg::Odometry> 
            ("/aft_mapped_to_init", 1000);
    ros::Publisher pubPath          = nh.advertise<nav_msgs::msg::Path> 
            ("/path", 1000);""",
"""    auto pubLaserCloudMap = nh->create_publisher<sensor_msgs::msg::PointCloud2>
            ("/Laser_map", 1000);
    auto pubOdomAftMapped = nh->create_publisher<nav_msgs::msg::Odometry> 
            ("/aft_mapped_to_init", 1000);
    auto pubPath          = nh->create_publisher<nav_msgs::msg::Path> 
            ("/path", 1000);""")

patch("src/laserMapping.cpp",
"""    signal(SIGINT, SigHandle);
    ros::Rate loop_rate(500);
    bool status = ros::ok();
    while (status)
    {
        if (flg_exit) break;
        ros::spinOnce();""",
"""    signal(SIGINT, SigHandle);
    rclcpp::Rate loop_rate(500);
    bool status = rclcpp::ok();
    while (status)
    {
        if (flg_exit) break;
        rclcpp::spin_some(nh);""")

patch("src/laserMapping.cpp",
"""        status = ros::ok();
        loop_rate.sleep();
    }""",
"""        status = rclcpp::ok();
        loop_rate.sleep();
    }""")

patch("src/laserMapping.cpp",
"""    fout_out.close();
    fout_imu_pbp.close();
    return 0;
}""",
"""    fout_out.close();
    fout_imu_pbp.close();
    rclcpp::shutdown();
    return 0;
}""")

# publish_odometry / publish_path 调用点补 tfBr 实参
patch("src/laserMapping.cpp", "publish_odometry(pubOdomAftMapped);", "publish_odometry(pubOdomAftMapped, tfBr);", 3)

io.open(os.path.join(D, "MIGRATION_LOG_structural.txt"), "w", encoding="utf-8").write("\n".join(LOG) + "\n")
print("STRUCTURAL PATCHES OK: %d" % len(LOG))

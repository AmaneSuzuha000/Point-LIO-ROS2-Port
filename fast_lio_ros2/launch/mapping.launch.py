# -*- coding: utf-8 -*-
# mapping.launch.py —— ROS1 mapping.launch 的 ROS2 等价物
# ROS1 是 XML：<node pkg="fast_lio" type="fastlio_mapping" name="driver" output="screen">
#                <rosparam command="load" file="$(find fast_lio)/config/robot.yaml"/>
#              </node>
# ROS2 是 Python：LaunchDescription + Node() 动作，参数用 parameters=[yaml] 传入
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("fast_lio")
    default_config = os.path.join(pkg_share, "config", "robot.yaml")
    default_rviz = os.path.join(pkg_share, "rviz", "fast_lio2.rviz")

    # ROS1 里用 <arg>，ROS2 用 DeclareLaunchArgument，语义一致
    config_arg = DeclareLaunchArgument(
        "config", default_value=default_config,
        description="IMU/LiDAR 外参与 ESKF 噪声配置文件")

    driver = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="driver",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("config")],  # 替代 <rosparam command="load">
    )

    rviz = Node(
        package="rviz2",                     # ROS1: package="rviz"
        executable="rviz2",                  # ROS1: executable="rviz"
        arguments=["-d", default_rviz],
        output="screen",
    )

    return LaunchDescription([config_arg, driver, rviz])

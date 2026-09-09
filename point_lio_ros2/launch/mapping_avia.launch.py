# -*- coding: utf-8 -*-
"""ROS2 launch（由 ROS1 的 mapping_avia.launch 迁移而来）。

ROS1 XML 与 ROS2 Python 的对应关系：
  <rosparam command="load" file="$(find point_lio)/config/avia.yaml" />
      -> parameters=[os.path.join(pkg, 'config', 'avia.yaml')]
  <param name="k" type="double" value="0.5"/>
      -> parameters=[{..., 'k': 0.5}]（ROS2 不再需要 type=，值本身带类型）
  <node pkg="rviz" type="rviz" args="-d ...">
      -> Node(package='rviz2', executable='rviz2', arguments=['-d', ...])
  $(find point_lio)
      -> get_package_share_directory('point_lio')
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('point_lio')
    # 与 ROS1 版逐条对应的节点参数覆盖（原来写在 <param> 里，现在放 parameters 列表，
    # 排在 YAML 之后，后者生效——语义与 ROS1 的 rosparam load + <param> 相同）
    overrides = {
        'use_imu_as_input': False,          # 1 = 用 IMU 作为 ESKF 输入
        'prop_at_freq_of_imu': True,
        'check_satu': True,
        'init_map_size': 10,
        'point_filter_num': 1,
        'space_down_sample': True,
        'filter_size_surf': 0.5,
        'filter_size_map': 0.4,
        'ivox_nearby_type': 6,
        'runtime_pos_log_enable': False,
    }
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='point_lio',
            executable='pointlio_mapping',
            name='point_lio',                # 必须与 YAML 顶层键一致，否则参数不生效
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'avia.yaml'), overrides],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg, 'rviz_cfg', 'loam_livox.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])

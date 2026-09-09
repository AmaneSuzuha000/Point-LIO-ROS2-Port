# -*- coding: utf-8 -*-
"""调试用 launch（迁移自 ROS1 gdb_debug_example.launch）。
ROS1 的 launch-prefix="gdb -ex run --args" 在 ROS2 里写法相同，但需要终端交互，
默认不加 prefix；需要 gdb 时把 launch-prefix 参数传进来：
  ros2 launch point_lio gdb_debug_example.launch.py launch_prefix="gdb -ex run --args"
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('point_lio')
    return LaunchDescription([
        DeclareLaunchArgument('launch_prefix', default_value=''),
        Node(
            package='point_lio',
            executable='pointlio_mapping',
            name='point_lio',
            output='screen',
            prefix=LaunchConfiguration('launch_prefix'),
            parameters=[os.path.join(pkg, 'config', 'avia.yaml'),
                        {'filter_size_surf': 0.3, 'filter_size_map': 0.2}],
        ),
    ])

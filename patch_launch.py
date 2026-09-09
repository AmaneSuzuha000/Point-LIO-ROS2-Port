#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""launch XML -> launch.py；rviz1 配置 -> rviz2；删除从未生成的死消息。"""
import io, os, shutil
D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "point_lio_ros2")
def w(rel, text):
    p = os.path.join(D, rel)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    io.open(p, "w", encoding="utf-8", newline="\n").write(text)
    print("WROTE", rel)

TPL = '''# -*- coding: utf-8 -*-
"""ROS2 launch（由 ROS1 的 {xml} 迁移而来）。

ROS1 XML 与 ROS2 Python 的对应关系：
  <rosparam command="load" file="$(find point_lio)/config/{cfg}" />
      -> parameters=[os.path.join(pkg, 'config', '{cfg}')]
  <param name="k" type="double" value="0.5"/>
      -> parameters=[{{..., 'k': 0.5}}]（ROS2 不再需要 type=，值本身带类型）
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
    overrides = {{
        'use_imu_as_input': {imu_in},          # 1 = 用 IMU 作为 ESKF 输入
        'prop_at_freq_of_imu': True,
        'check_satu': True,
        'init_map_size': 10,
        'point_filter_num': {pfn},
        'space_down_sample': True,
        'filter_size_surf': {fsurf},
        'filter_size_map': {fmap},
        'ivox_nearby_type': 6,
        'runtime_pos_log_enable': False,
    }}
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='point_lio',
            executable='pointlio_mapping',
            name='point_lio',                # 必须与 YAML 顶层键一致，否则参数不生效
            output='screen',
            parameters=[os.path.join(pkg, 'config', '{cfg}'), overrides],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg, 'rviz_cfg', 'loam_livox.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
'''

w("launch/mapping_avia.launch.py", TPL.format(xml="mapping_avia.launch", cfg="avia.yaml", imu_in="False", pfn="1", fsurf="0.5", fmap="0.4"))
w("launch/mapping_horizon.launch.py", TPL.format(xml="mapping_horizon.launch", cfg="horizon.yaml", imu_in="False", pfn="3", fsurf="0.5", fmap="0.5"))
w("launch/mapping_ouster64.launch.py", TPL.format(xml="mapping_ouster64.launch", cfg="ouster64.yaml", imu_in="False", pfn="4", fsurf="0.5", fmap="0.5"))
w("launch/mapping_velody16.launch.py", TPL.format(xml="mapping_velody16.launch", cfg="velody16.yaml", imu_in="False", pfn="4", fsurf="0.5", fmap="0.5"))
w("launch/gdb_debug_example.launch.py", '''# -*- coding: utf-8 -*-
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
''')

# 删除 ROS1 XML launch
for f in sorted(os.listdir(os.path.join(D, "launch"))):
    if f.endswith(".launch"):
        os.remove(os.path.join(D, "launch", f)); print("DELETED launch/" + f)

# msg/LocalSensorExternalTrigger.msg：ROS1 的 CMakeLists 只有 generate_messages()，
if os.path.isdir(os.path.join(D, "msg")):
    shutil.rmtree(os.path.join(D, "msg")); print("DELETED msg/ (从未被构建的死消息)")

w("rviz_cfg/loam_livox.rviz", """Panels:
  - Class: displays/Displays
    Help Height: 0
    Name: Displays
    Property Tree Widget:
      Expanded: ~
      Splitter Ratio: 0.6432291865348816
    Tree Height: 500
  - Class: views/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: time/Time
    DisplayName: Time
    Name: Time
    SyncMode: 0
    SyncSource: surround
Preferences:
  PromptSaveOnExit: true
Toolbars:
  toolButtonStyle: 2
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 10
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 40
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/Axes
      Enabled: true
      Length: 0.7
      Name: Axes_body
      Radius: 0.06
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/PointCloud2
      Color: 238; 238; 236
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 255
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: surround
      Position Transformer: XYZ
      Selectable: false
      Size (Pixels): 3
      Size (m): 0.05
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /cloud_registered
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Min Color: 0; 0; 0
      Name: Laser_map
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 2
      Size (m): 0.03
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /Laser_map
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 1
      Axes Length: 1
      Axes Radius: 0.1
      Class: rviz_default_plugins/Pose
      Color: 255; 25; 0
      Enabled: true
      Head Length: 0.1
      Head Radius: 0.05
      Name: Pose
      Shaft Length: 0.2
      Shaft Radius: 0.02
      Shape: Axes
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /aft_mapped_to_init
      Value: true
    - Alpha: 1
      Buffer Length: 2
      Class: rviz_default_plugins/Path
      Color: 25; 255; 255
      Enabled: true
      Head Diameter: 0
      Head Length: 0
      Length: 0.3
      Line Style: Billboards
      Line Width: 0.05
      Name: Path
      Offset:
        X: 0
        Y: 0
        Z: 0
      Pose Color: 255; 85; 255
      Pose Style: None
      Radius: 0.03
      Shaft Diameter: 0.1
      Shaft Length: 0.1
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /path
      Value: true
  Enabled: true
  Global Options:
    Background Color: 0; 0; 0
    Fixed Frame: camera_init
    Frame Rate: 10
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Covariance x: 0.25
      Covariance y: 0.25
      Covariance yaw: 0.06853891909122467
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /move_base_simple/goal
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 40
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.4
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 4.2
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 800
  Hide Left Dock: false
  Hide Right Dock: true
  Views:
    collapsed: false
  Width: 1200
  X: 60
  Y: 60
""")
print("launch + rviz done")

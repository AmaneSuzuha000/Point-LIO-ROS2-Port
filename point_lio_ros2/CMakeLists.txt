cmake_minimum_required(VERSION 3.16)
project(point_lio)

# ===== ROS1(catkin) -> ROS2(ament_cmake) 构建系统迁移 =====
# 原文件用 find_package(catkin) + catkin_package() + generate_messages()，
# 这三者在 ROS2 中分别对应 ament_cmake / ament_package / rosidl
# （本工程没有自定义消息，见 migration_guide.md 对 msg/ 的说明）。
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)          # Humble 的 rclcpp 头文件要求 C++17（ROS1 版是 C++14）
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processor number: ${N}")
  if(N GREATER 5)
    add_definitions(-DMP_EN -DMP_PROC_NUM=4)
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN -DMP_PROC_NUM="${PROC_NUM}")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

# 迁移删除：find_package(PythonLibs REQUIRED) 与 matplotlibcpp —— 源码里只有
# 被注释掉的 #include "matplotlibcpp.h"，从未调用任何绘图函数，属于上游遗留依赖。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)   # ROS1: livox_ros_driver -> ROS2: livox_ros_driver2

find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 REQUIRED)            # ROS1 版要求 1.8；Humble 自带 PCL 1.12

include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(pointlio_mapping
  src/laserMapping.cpp
  src/li_initialization.cpp
  src/parameters.cpp
  src/preprocess.cpp
  src/Estimator.cpp
  src/IMU_Processing.cpp)
ament_target_dependencies(pointlio_mapping
  rclcpp sensor_msgs nav_msgs geometry_msgs visualization_msgs
  pcl_conversions tf2_ros tf2_geometry_msgs livox_ros_driver2)
target_link_libraries(pointlio_mapping ${PCL_LIBRARIES})

install(TARGETS pointlio_mapping DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz_cfg DESTINATION share/${PROJECT_NAME})

ament_package()

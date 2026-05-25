cmake_minimum_required(VERSION 3.8)
project(livox_custom_downsample_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

add_executable(
  livox_custom_voxel_downsample
  src/livox_custom_voxel_downsample.cpp
)

ament_target_dependencies(
  livox_custom_voxel_downsample
  rclcpp
  sensor_msgs
  pcl_conversions
  livox_ros_driver2
)

target_link_libraries(
  livox_custom_voxel_downsample
  ${PCL_LIBRARIES}
)

install(
  TARGETS livox_custom_voxel_downsample
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

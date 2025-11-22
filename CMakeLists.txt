cmake_minimum_required(VERSION 3.8)
project(lio_ros_wrapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Force system spdlog to avoid conflicts
find_package(PkgConfig REQUIRED)
pkg_check_modules(SPDLOG REQUIRED spdlog)

# LIO directory (no Pangolin needed for ROS2 - use RViz instead)
set(LIO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lidar_inertial_odometry)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${LIO_DIR}/src/core
  ${LIO_DIR}/src/util
)

# Collect LIO source files (no viewer - use RViz for visualization)
set(LIO_SOURCES
    ${LIO_DIR}/src/core/Estimator.cpp
    ${LIO_DIR}/src/core/State.cpp
    ${LIO_DIR}/src/core/VoxelMap.cpp
    ${LIO_DIR}/src/core/ProbabilisticKernelOptimizer.cpp
    ${LIO_DIR}/src/util/ConfigUtils.cpp
    ${LIO_DIR}/src/util/PointCloudUtils.cpp
    ${LIO_DIR}/src/util/LieUtils.cpp
)

# LIO ROS2 Node
add_executable(lio_node
  src/lio_node.cpp
  ${LIO_SOURCES}
)

ament_target_dependencies(lio_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(lio_node
  yaml-cpp
  ${SPDLOG_LIBRARIES}
)

target_include_directories(lio_node PRIVATE
  ${SPDLOG_INCLUDE_DIRS}
)

if(TARGET Eigen3::Eigen)
    target_link_libraries(lio_node Eigen3::Eigen)
else()
    target_include_directories(lio_node PRIVATE ${EIGEN3_INCLUDE_DIR})
endif()

# Install targets
install(TARGETS
  lio_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz config files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install LIO config files (from lidar_inertial_odometry)
install(DIRECTORY
  lidar_inertial_odometry/config/
  DESTINATION share/${PROJECT_NAME}/config/lidar_inertial_odometry
  FILES_MATCHING PATTERN "*.yaml"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

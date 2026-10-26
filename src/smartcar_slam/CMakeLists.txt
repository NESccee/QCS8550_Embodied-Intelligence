cmake_minimum_required(VERSION 3.14)
project(smartcar_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(lio_sam_node src/lio_sam_node.cpp)
ament_target_dependencies(lio_sam_node rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs tf2 tf2_ros)

add_executable(semantic_projection_node src/semantic_projection_node.cpp)
ament_target_dependencies(semantic_projection_node rclcpp sensor_msgs std_msgs)

add_executable(semantic_planner_node src/semantic_planner_node.cpp)
ament_target_dependencies(semantic_planner_node rclcpp geometry_msgs nav_msgs std_msgs)

install(
  TARGETS lio_sam_node semantic_projection_node semantic_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_slam_algorithms test/test_slam_algorithms.cpp)
  target_include_directories(test_slam_algorithms PRIVATE include ${EIGEN3_INCLUDE_DIRS})
  ament_target_dependencies(test_slam_algorithms rclcpp sensor_msgs geometry_msgs nav_msgs)
endif()

ament_package()


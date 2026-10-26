cmake_minimum_required(VERSION 3.14)
project(smartcar_teleop)

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

# 纯数学部分（死区 / 缩比 / 反向 / 使能键）单独拆成 header-only 目标，便于单元测试。
add_library(smartcar_teleop_mapper INTERFACE)
target_include_directories(smartcar_teleop_mapper INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(smartcar_teleop_mapper INTERFACE cxx_std_17)

add_executable(joy_teleop src/joy_teleop_node.cpp)
target_link_libraries(joy_teleop smartcar_teleop_mapper)
ament_target_dependencies(joy_teleop rclcpp sensor_msgs geometry_msgs)

add_executable(joy_monitor src/joy_monitor_node.cpp)
target_link_libraries(joy_monitor smartcar_teleop_mapper)
ament_target_dependencies(joy_monitor rclcpp sensor_msgs)

add_executable(drive_test src/drive_test_node.cpp)
ament_target_dependencies(drive_test rclcpp geometry_msgs)

install(TARGETS joy_teleop joy_monitor drive_test DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_twist_mapper test/test_twist_mapper.cpp)
  target_link_libraries(test_twist_mapper smartcar_teleop_mapper)
endif()

ament_package()

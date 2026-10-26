cmake_minimum_required(VERSION 3.14)
project(smartcar_sim)

find_package(ament_cmake REQUIRED)

# 纯数据包：模型 / 世界 / launch，没有需要编译的代码
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()

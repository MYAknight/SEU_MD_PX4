cmake_minimum_required(VERSION 3.10)
project(huaqiccc_arm_ros_plugin)

# C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

# 包含目录
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

# 编译插件
add_library(${PROJECT_NAME} SHARED arm_rotation_ros_plugin.cc)

target_link_libraries(${PROJECT_NAME}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
)

# 安装目标
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

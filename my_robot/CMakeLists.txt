cmake_minimum_required(VERSION 3.8)
project(my_robot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)   # YDLidar SDK V1.1.3와 맞춘 표준
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

install(
  DIRECTORY urdf launch config
  DESTINATION share/${PROJECT_NAME}
)

include_directories(
  /usr/local/include
  /usr/local/include/src
)

add_executable(ydlidar_node src/ydlidar_node.cpp)
ament_target_dependencies(ydlidar_node rclcpp sensor_msgs)

target_link_libraries(ydlidar_node
  /usr/local/lib/libydlidar_sdk.a
)

install(TARGETS
  ydlidar_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  my_robot/base_controller.py
  my_robot/keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
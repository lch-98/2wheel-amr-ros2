cmake_minimum_required(VERSION 3.8)
project(my_dual_axis_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 필수 (ros2_canopen이 요구)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(canopen_core REQUIRED)
find_package(canopen_base_driver REQUIRED)
find_package(canopen_proxy_driver REQUIRED)
find_package(canopen_interfaces REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(canopen_ros2_control REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_srvs
  sensor_msgs
  diagnostic_updater
  canopen_core
  canopen_base_driver
  canopen_proxy_driver
  canopen_interfaces
  lely_core_libraries
  hardware_interface
  pluginlib
  canopen_ros2_control
)

include_directories(include)

# ── 공유 라이브러리 (device_container가 dlopen 함) ──
add_library(${PROJECT_NAME} SHARED
  src/node_canopen_dual_axis_driver.cpp
  src/dual_axis_driver.cpp
  src/dual_axis_system.cpp
)
target_compile_definitions(${PROJECT_NAME} PRIVATE "MY_DUAL_AXIS_DRIVER_BUILDING_LIBRARY")
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

# ── ROS2 컴포넌트로 등록 ──
rclcpp_components_register_nodes(${PROJECT_NAME} "ros2_canopen::DualAxisDriver")
pluginlib_export_plugin_description_file(hardware_interface dual_axis_hardware.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

ament_package()

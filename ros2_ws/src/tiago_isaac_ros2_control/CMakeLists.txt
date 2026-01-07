cmake_minimum_required(VERSION 3.8)
project(tiago_isaac_ros2_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/isaac_system.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_lifecycle sensor_msgs hardware_interface pluginlib
)

# ✅ pluginlib export (이거 하나면 충분)
pluginlib_export_plugin_description_file(hardware_interface tiago_isaac_ros2_control.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)


ament_package()

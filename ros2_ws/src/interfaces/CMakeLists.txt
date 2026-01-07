cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# --- Interface generation (msg/srv/action) ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ApproachBox.action"
  "action/LiftBox.action"
  "action/DropBox.action"
  "action/NavigateToRoom.action"
  "msg/BoxDetection2D.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# 런타임에 다른 패키지들이 이 인터페이스를 사용할 수 있도록 export
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

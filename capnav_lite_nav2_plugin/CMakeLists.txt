cmake_minimum_required(VERSION 3.8)
project(capnav_lite_nav2_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

add_library(capnav_lite_controller SHARED
  src/capnav_lite_controller.cpp
)
ament_target_dependencies(capnav_lite_controller
  rclcpp nav2_core nav2_util geometry_msgs nav_msgs pluginlib
)

pluginlib_export_plugin_description_file(nav2_core plugins/capnav_lite_controller_plugin.xml)

install(TARGETS capnav_lite_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY plugins
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

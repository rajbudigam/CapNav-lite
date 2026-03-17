cmake_minimum_required(VERSION 3.16)
project(capnav_lite_nav2_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(include)

add_library(capnav_lite_controller SHARED
  src/capnav_lite_controller.cpp
  src/profile_loader.cpp
)

ament_target_dependencies(capnav_lite_controller
  rclcpp rclcpp_lifecycle nav2_core nav2_util nav2_costmap_2d
  nav_msgs geometry_msgs tf2_ros pluginlib nlohmann_json
)

pluginlib_export_plugin_description_file(nav2_core plugins/capnav_controller_plugin.xml)

install(TARGETS capnav_lite_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(FILES plugins/capnav_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}/plugins)

ament_export_include_directories(include)
ament_export_libraries(capnav_lite_controller)
ament_package()

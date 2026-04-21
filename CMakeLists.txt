cmake_minimum_required(VERSION 3.8)
project(nav2_waypoint_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
#set(CMAKE_AUTOMOC ON)

include_directories(
  include
)

add_library(nav2_waypoint_goal_tool SHARED
  src/waypoint_goal_tool.cpp
)

target_compile_definitions(nav2_waypoint_goal_tool PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")
target_link_libraries(nav2_waypoint_goal_tool Qt5::Widgets)

ament_target_dependencies(nav2_waypoint_goal_tool
  rclcpp
  geometry_msgs
  pluginlib
  rviz_common
  rviz_default_plugins
  tf2
)

add_library(nav2_waypoint_control_panel SHARED
  src/waypoint_control_panel.cpp
)

target_link_libraries(nav2_waypoint_control_panel Qt5::Widgets)

ament_target_dependencies(nav2_waypoint_control_panel
  rclcpp
  std_msgs
  std_srvs
  pluginlib
  rviz_common
)

add_executable(pose_collector_node
  src/pose_collector_node.cpp
)

ament_target_dependencies(pose_collector_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  std_msgs
  std_srvs
  visualization_msgs
  nav_msgs
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(
  TARGETS
    nav2_waypoint_goal_tool
    nav2_waypoint_control_panel
    pose_collector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
cmake_minimum_required(VERSION 3.16)
project(mars_yard_sim_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mujoco_vendor REQUIRED)
find_package(mujoco_ros2_control_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(mars_yard_sim_plugins SHARED
  src/ray_scan_plugin.cpp
  src/teleport_plugin.cpp
)
target_compile_features(mars_yard_sim_plugins PUBLIC cxx_std_17)
target_include_directories(mars_yard_sim_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mars_yard_sim_plugins PUBLIC
  mujoco_ros2_control_plugins::mujoco_ros2_control_plugins
  pluginlib::pluginlib
  realtime_tools::realtime_tools
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)
target_compile_definitions(mars_yard_sim_plugins PRIVATE
  REALTIME_TOOLS_VERSION_MAJOR=${realtime_tools_VERSION_MAJOR}
)

pluginlib_export_plugin_description_file(
  mujoco_ros2_control_plugins
  mars_yard_sim_plugins.xml
)

install(
  TARGETS mars_yard_sim_plugins
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(mars_yard_sim_plugins)
ament_export_dependencies(
  geometry_msgs
  mujoco_ros2_control_plugins
  mujoco_vendor
  pluginlib
  rclcpp
  realtime_tools
  sensor_msgs
  std_msgs
)

ament_package()

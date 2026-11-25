cmake_minimum_required(VERSION 3.16)
project(mars_yard_metrics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(metrics_core STATIC
  src/metrics_core.cpp
)
target_compile_features(metrics_core PUBLIC cxx_std_17)
target_include_directories(metrics_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(metrics_node
  src/metrics_node.cpp
)
target_compile_features(metrics_node PRIVATE cxx_std_17)
target_link_libraries(metrics_node PRIVATE
  metrics_core
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

install(
  TARGETS metrics_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_metrics_core test/test_metrics_core.cpp)
  if(TARGET test_metrics_core)
    target_compile_features(test_metrics_core PRIVATE cxx_std_17)
    target_link_libraries(test_metrics_core metrics_core)
  endif()
endif()

ament_package()

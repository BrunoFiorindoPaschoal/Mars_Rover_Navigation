cmake_minimum_required(VERSION 3.8)
project(mars_yard_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/twist_stamper.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

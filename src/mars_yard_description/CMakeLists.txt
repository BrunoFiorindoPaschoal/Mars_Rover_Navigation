cmake_minimum_required(VERSION 3.8)
project(mars_yard_description)
find_package(ament_cmake REQUIRED)
# data-only package: install the model files into share/mars_yard_description
install(DIRECTORY urdf mjcf DESTINATION share/${PROJECT_NAME})
ament_package()

cmake_minimum_required(VERSION 3.8)
project(mars_yard_worlds)
find_package(ament_cmake REQUIRED)
# data-only package: install the model files into share/mars_yard_worlds
# terrain/ too: mars.xml loads ../terrain/*.bin and texture.png, and Nav2 loads slope.yaml
install(DIRECTORY worlds terrain DESTINATION share/${PROJECT_NAME})
ament_package()

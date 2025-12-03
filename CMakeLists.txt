cmake_minimum_required(VERSION 3.8)
project(minimal_slam)

find_package(ament_cmake REQUIRED)

# Install essential directories
install(DIRECTORY 
  launch 
  config 
  urdf
  DESTINATION share/${PROJECT_NAME}/)

ament_package()


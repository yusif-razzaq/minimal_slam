cmake_minimum_required(VERSION 3.8)
project(minimal_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf guides
  DESTINATION share/${PROJECT_NAME}/)

install(PROGRAMS
  scripts/imu_combiner.py
  DESTINATION lib/${PROJECT_NAME}/)

ament_package()


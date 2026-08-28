cmake_minimum_required(VERSION 3.8)
project(vgr_nav2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps rviz behavior_trees DESTINATION share/${PROJECT_NAME})

ament_package()

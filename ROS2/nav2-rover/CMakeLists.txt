cmake_minimum_required(VERSION 3.5)
project(nav2_rover)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  PROGRAMS
    src/cmdvel_listener.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

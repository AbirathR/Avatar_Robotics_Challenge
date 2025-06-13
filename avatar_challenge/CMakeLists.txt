cmake_minimum_required(VERSION 3.8)
project(avatar_challenge)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/simple_arm_mover.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
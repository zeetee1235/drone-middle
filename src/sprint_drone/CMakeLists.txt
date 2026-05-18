cmake_minimum_required(VERSION 3.16)
project(sprint_drone)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

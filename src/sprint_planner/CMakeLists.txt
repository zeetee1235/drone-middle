cmake_minimum_required(VERSION 3.16)
project(sprint_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(sprint_planner_core
  src/time_cost.cpp
)
target_include_directories(sprint_planner_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(sprint_planner_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(sprint_planner_core_test
  test/time_cost_test.cpp
)
target_link_libraries(sprint_planner_core_test sprint_planner_core)
add_test(NAME sprint_planner_core_test COMMAND sprint_planner_core_test)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sprint_drone_msgs REQUIRED)

  add_executable(sprint_planner_node
    src/sprint_planner_node.cpp
  )
  target_link_libraries(sprint_planner_node sprint_planner_core)
  ament_target_dependencies(sprint_planner_node
    rclcpp
    geometry_msgs
    std_msgs
    sprint_drone_msgs
  )

  install(TARGETS
    sprint_planner_core
    sprint_planner_node
    DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY include/
    DESTINATION include
  )

  ament_package()
endif()

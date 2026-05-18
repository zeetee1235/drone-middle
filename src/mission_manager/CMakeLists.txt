cmake_minimum_required(VERSION 3.16)
project(mission_manager LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(mission_manager_core
  src/mission_core.cpp
)
target_include_directories(mission_manager_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(mission_manager_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(mission_manager_core_test
  test/mission_core_test.cpp
)
target_link_libraries(mission_manager_core_test mission_manager_core)
add_test(NAME mission_manager_core_test COMMAND mission_manager_core_test)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sprint_drone_msgs REQUIRED)

  add_executable(mission_manager_node
    src/mission_manager_node.cpp
  )
  target_link_libraries(mission_manager_node mission_manager_core)
  ament_target_dependencies(mission_manager_node
    rclcpp
    std_msgs
    std_srvs
    geometry_msgs
    sprint_drone_msgs
  )

  install(TARGETS
    mission_manager_core
    mission_manager_node
    DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY include/
    DESTINATION include
  )

  ament_package()
endif()

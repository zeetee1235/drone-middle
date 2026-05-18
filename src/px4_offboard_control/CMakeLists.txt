cmake_minimum_required(VERSION 3.16)
project(px4_offboard_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(px4_offboard_control_core
  src/control_core.cpp
)
target_include_directories(px4_offboard_control_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(px4_offboard_control_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(px4_offboard_control_core_test
  test/control_core_test.cpp
)
target_link_libraries(px4_offboard_control_core_test px4_offboard_control_core)
add_test(NAME px4_offboard_control_core_test COMMAND px4_offboard_control_core_test)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sprint_drone_msgs REQUIRED)
  find_package(px4_msgs QUIET)

  if(px4_msgs_FOUND)
    add_executable(px4_offboard_control_node
      src/px4_offboard_control_node.cpp
    )
    target_link_libraries(px4_offboard_control_node px4_offboard_control_core)
    ament_target_dependencies(px4_offboard_control_node
      rclcpp
      std_msgs
      geometry_msgs
      sprint_drone_msgs
      px4_msgs
    )

    install(TARGETS
      px4_offboard_control_core
      px4_offboard_control_node
      DESTINATION lib/${PROJECT_NAME}
    )
  else()
    message(WARNING "px4_msgs not found; px4_offboard_control_node will not be built")
    install(TARGETS px4_offboard_control_core DESTINATION lib/${PROJECT_NAME})
  endif()

  install(DIRECTORY include/
    DESTINATION include
  )

  ament_package()
endif()

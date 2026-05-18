cmake_minimum_required(VERSION 3.16)
project(grid_detector LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(grid_detector_core
  src/grid_geometry.cpp
)
target_include_directories(grid_detector_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(grid_detector_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(grid_detector_core_test
  test/grid_geometry_test.cpp
)
target_link_libraries(grid_detector_core_test grid_detector_core)
add_test(NAME grid_detector_core_test COMMAND grid_detector_core_test)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(sprint_drone_msgs REQUIRED)
  find_package(OpenCV QUIET)

  if(OpenCV_FOUND)
    add_executable(grid_detector_node
      src/grid_detector_node.cpp
    )
    target_link_libraries(grid_detector_node
      grid_detector_core
      ${OpenCV_LIBS}
    )
    target_include_directories(grid_detector_node PRIVATE ${OpenCV_INCLUDE_DIRS})
    ament_target_dependencies(grid_detector_node
      rclcpp
      sensor_msgs
      std_msgs
      geometry_msgs
      cv_bridge
      sprint_drone_msgs
    )

    install(TARGETS
      grid_detector_core
      grid_detector_node
      DESTINATION lib/${PROJECT_NAME}
    )
  else()
    message(WARNING "OpenCV not found; grid_detector_node will not be built")
    install(TARGETS grid_detector_core DESTINATION lib/${PROJECT_NAME})
  endif()

  install(DIRECTORY include/
    DESTINATION include
  )

  ament_package()
endif()

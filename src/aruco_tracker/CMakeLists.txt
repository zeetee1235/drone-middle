cmake_minimum_required(VERSION 3.16)
project(aruco_tracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(aruco_tracker_core
  src/confidence_filter.cpp
)
target_include_directories(aruco_tracker_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(aruco_tracker_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(aruco_tracker_core_test
  test/confidence_filter_test.cpp
)
target_link_libraries(aruco_tracker_core_test aruco_tracker_core)
add_test(NAME aruco_tracker_core_test COMMAND aruco_tracker_core_test)

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
    add_executable(aruco_tracker_node
      src/aruco_tracker_node.cpp
    )
    target_link_libraries(aruco_tracker_node
      aruco_tracker_core
      ${OpenCV_LIBS}
    )
    target_include_directories(aruco_tracker_node PRIVATE ${OpenCV_INCLUDE_DIRS})
    ament_target_dependencies(aruco_tracker_node
      rclcpp
      sensor_msgs
      std_msgs
      geometry_msgs
      cv_bridge
      sprint_drone_msgs
    )

    install(TARGETS
      aruco_tracker_core
      aruco_tracker_node
      DESTINATION lib/${PROJECT_NAME}
    )
  else()
    message(WARNING "OpenCV not found; aruco_tracker_node will not be built")
    install(TARGETS aruco_tracker_core DESTINATION lib/${PROJECT_NAME})
  endif()

  install(DIRECTORY include/
    DESTINATION include
  )

  ament_package()
endif()

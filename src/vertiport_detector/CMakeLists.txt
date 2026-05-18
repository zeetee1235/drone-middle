cmake_minimum_required(VERSION 3.16)
project(vertiport_detector LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(vertiport_detector_core
  src/descent_filter.cpp
)
target_include_directories(vertiport_detector_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(vertiport_detector_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(vertiport_detector_core_test
  test/descent_filter_test.cpp
)
target_link_libraries(vertiport_detector_core_test vertiport_detector_core)
add_test(NAME vertiport_detector_core_test COMMAND vertiport_detector_core_test)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(OpenCV QUIET)

  if(OpenCV_FOUND)
    add_executable(vertiport_detector_node
      src/vertiport_detector_node.cpp
    )
    target_link_libraries(vertiport_detector_node
      vertiport_detector_core
      ${OpenCV_LIBS}
    )
    target_include_directories(vertiport_detector_node PRIVATE ${OpenCV_INCLUDE_DIRS})
    ament_target_dependencies(vertiport_detector_node
      rclcpp
      sensor_msgs
      std_msgs
      geometry_msgs
      cv_bridge
    )

    install(TARGETS
      vertiport_detector_core
      vertiport_detector_node
      DESTINATION lib/${PROJECT_NAME}
    )
  else()
    message(WARNING "OpenCV not found; vertiport_detector_node will not be built")
    install(TARGETS vertiport_detector_core DESTINATION lib/${PROJECT_NAME})
  endif()

  install(DIRECTORY include/
    DESTINATION include
  )

  ament_package()
endif()

cmake_minimum_required(VERSION 3.16)
project(visual_odometry LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(visual_odometry_core
  src/localization_core.cpp
)
target_include_directories(visual_odometry_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(visual_odometry_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(visual_odometry_core_test
  test/localization_core_test.cpp
)
target_link_libraries(visual_odometry_core_test visual_odometry_core)
add_test(NAME visual_odometry_core_test COMMAND visual_odometry_core_test)

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
    add_executable(visual_odometry_node
      src/visual_odometry_node.cpp
    )
    target_link_libraries(visual_odometry_node
      visual_odometry_core
      ${OpenCV_LIBS}
    )
    target_include_directories(visual_odometry_node PRIVATE ${OpenCV_INCLUDE_DIRS})
    ament_target_dependencies(visual_odometry_node
      rclcpp
      sensor_msgs
      std_msgs
      geometry_msgs
      cv_bridge
      sprint_drone_msgs
    )

    install(TARGETS
      visual_odometry_core
      visual_odometry_node
      DESTINATION lib/${PROJECT_NAME}
    )
  else()
    message(WARNING "OpenCV not found; visual_odometry_node will not be built")
    install(TARGETS visual_odometry_core DESTINATION lib/${PROJECT_NAME})
  endif()

  install(DIRECTORY include/
    DESTINATION include
  )

  ament_package()
endif()

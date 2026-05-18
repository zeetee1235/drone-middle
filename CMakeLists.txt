cmake_minimum_required(VERSION 3.16)
project(sprint_drone_prototype LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

add_subdirectory(src/sprint_planner)
add_subdirectory(src/grid_detector)
add_subdirectory(src/aruco_tracker)
add_subdirectory(src/visual_odometry)
add_subdirectory(src/mission_manager)
add_subdirectory(src/px4_offboard_control)
add_subdirectory(src/vertiport_detector)

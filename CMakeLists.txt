cmake_minimum_required(VERSION 3.8)
project(gazebo_continuous_track_example_ros2_gazebo11)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(gazebo_continuous_track_example_ros2_gazebo11_drive SHARED
  src/example_track_drive_plugin.cpp
)
target_include_directories(gazebo_continuous_track_example_ros2_gazebo11_drive PRIVATE
  ${GAZEBO_INCLUDE_DIRS}
)
ament_target_dependencies(gazebo_continuous_track_example_ros2_gazebo11_drive
  geometry_msgs
  rclcpp
)
target_link_libraries(gazebo_continuous_track_example_ros2_gazebo11_drive
  ${GAZEBO_LIBRARIES}
)

install(TARGETS gazebo_continuous_track_example_ros2_gazebo11_drive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  urdf_xacro
  world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

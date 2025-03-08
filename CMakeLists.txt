cmake_minimum_required(VERSION 3.0.2)
project(final_project)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  gazebo
  gazebo_plugins
  gazebo_ros
  roscpp
  rospy
  urdf
  xacro
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(final_project_plugin SHARED src/final_project_plugin.cc)
target_link_libraries(final_project_plugin ${GAZEBO_LIBRARIES})
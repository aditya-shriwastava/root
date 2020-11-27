cmake_minimum_required(VERSION 2.8.3)
project(root_gazebo)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2_ros
  gazebo_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    tf2_ros
    gazebo_msgs
    nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(gt_pub
  src/gt_pub_node.cpp
  src/modules/gt_pub.cpp
)
target_link_libraries(gt_pub
  ${catkin_LIBRARIES}
)

cmake_minimum_required(VERSION 3.0.2)
project(stereo_extrinsic_calib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
)

add_executable(calib_node src/calib_node.cpp)
target_link_libraries(calib_node ${catkin_LIBRARIES})

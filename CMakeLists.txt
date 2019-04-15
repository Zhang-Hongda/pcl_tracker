cmake_minimum_required(VERSION 2.8.3)
project(pcl_tracker)
add_compile_options(-std=c++11)
#set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ")
#set (CMAKE_VERBOSE_MAKEFILE ON)
find_package(  catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  tf
  tf_conversions
  eigen_conversions
  roscpp
  sensor_msgs
)
catkin_package() 
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(marker_tracker_node src/marker_tracker.cpp)
target_link_libraries(marker_tracker_node ${catkin_LIBRARIES} )

cmake_minimum_required(VERSION 2.8.3)
project(cluster_extraction)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  message_generation
)

find_package(PCL 1.3 REQUIRED COMPONENTS common io)

add_message_files(
  FILES
  Cluster.msg
  ClusterArray.msg
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES cluster_extraction
 CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_ros roscpp std_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
 add_library(cluster_extraction
   src/cluster_extraction.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(cluster_extraction ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(cluster_extraction
  ${catkin_LIBRARIES}
)

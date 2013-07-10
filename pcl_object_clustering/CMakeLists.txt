cmake_minimum_required(VERSION 2.8.3)
project(pcl_object_clustering)

find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros sensor_msgs dynamic_reconfigure clustered_clouds_msgs)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/dynamic_config.cfg
)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  cfg/cpp
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}
  src/pcl_object_clustering.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)




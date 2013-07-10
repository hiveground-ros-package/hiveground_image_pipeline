cmake_minimum_required(VERSION 2.8.3)
project(pcl_hand_arm_detector)

find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros sensor_msgs dynamic_reconfigure cv_bridge clustered_clouds_msgs interaction_msgs)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

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


## Declare a cpp library
add_library(kalman_filter3d
   src/kalman_filter3d.cpp
)

target_link_libraries(kalman_filter3d
  ${OpenCV_LIBS}
)


## Declare a cpp executable
add_executable(${PROJECT_NAME}
  src/pcl_hand_arm_detector.cpp    
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  kalman_filter3d
)



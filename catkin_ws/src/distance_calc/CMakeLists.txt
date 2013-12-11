cmake_minimum_required(VERSION 2.8.3)
project(distance_calc)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  stereo_msgs
  message_generation
  tf
  clustering
)

generate_messages(DEPENDENCIES
  std_msgs
  stereo_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
CATKIN_DEPENDS message_runtime
)

add_executable(distance_calc src/distanceCalculation.cpp)
target_link_libraries(distance_calc ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_INCLUDE_DIRS})

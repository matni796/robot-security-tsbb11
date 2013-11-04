cmake_minimum_required(VERSION 2.8.3)
project(distanceCalc)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  stereo_msgs
  message_generation
)
#add_message_files(FILES )
#add_service_files()
#add_action_files()
generate_messages(DEPENDENCIES std_msgs)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
CATKIN_DEPENDS message_runtime
)

add_executable(distanceCalc src/distanceCalculation.cpp)
target_link_libraries(distanceCalc ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_INCLUDE_DIRS})
#add_dependencies(distanceCalc distanceCalc_gencpp)



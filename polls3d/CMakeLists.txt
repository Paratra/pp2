cmake_minimum_required(VERSION 2.8.3)
project(polls3d)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS

  
  roscpp
  rospy
  std_msgs
  cv_bridge
  velodyne_msgs
  message_generation
  sensor_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  imu_3dm_gx4


)

## System dependencies are found with CMake's conventions
 find_package(OpenCV REQUIRED )
 find_package(PCL 1.4.3 REQUIRED COMPONENTS common io)


add_message_files(
	FILES
	Pole.msg
	Poles.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
	

)




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES barrier_detection
#  CATKIN_DEPENDS roscpp rospy std_mags
#  DEPENDS system_lib
)

link_directories(
  ${OpenCV_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)


catkin_package(

CATKIN_DEPENDS 
message_runtime

)



add_definitions(${PCL_DEFINITIONS})



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)




add_executable(polls3d_ming_node
  src/polls3d.cpp
  include/polls3d/polls3d_ming.h
)

add_executable(polesDetc_ming_node
  src/polesDetc.cpp
  include/polls3d/polls3d_ming.h
)

add_executable(ming_polls_node
  src/main.cpp
  src/ming_polls.cpp
  include/polls3d/ming_polls.h
)



add_dependencies(ming_polls_node polls3d_gencpp)



## Specify libraries to link a library or executable target against
 target_link_libraries(polls3d_ming_node

   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${PCL_COMMON_LIBRARIES}
   ${PCL_IO_LIBRARIES}
 )

## Specify libraries to link a library or executable target against
 target_link_libraries(polesDetc_ming_node

   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${PCL_COMMON_LIBRARIES}
   ${PCL_IO_LIBRARIES}
 )

## Specify libraries to link a library or executable target against
 target_link_libraries(ming_polls_node

   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${PCL_COMMON_LIBRARIES}
   ${PCL_IO_LIBRARIES}
 )

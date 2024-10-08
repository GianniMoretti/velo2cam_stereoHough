cmake_minimum_required(VERSION 3.0.2)
project(velo2cam_stereoHough)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -w ")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  message_filters
  roscpp
  pcl_ros
  sensor_msgs
  std_msgs
  stereo_msgs
  tf
  tf_conversions
  dynamic_reconfigure
  cmake_modules
)

find_package(TinyXML REQUIRED)
find_package(OpenCV 4.4 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/Lidar.cfg
  cfg/Stereo.cfg
  cfg/Monocular.cfg
  cfg/Plane.cfg
)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ClusterCentroids.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  DEPENDS TinyXML
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_LIBRARIES}
  ${TinyXML_INCLUDE_DIRS}
)

#link_directories()

#add_definitions()

## Declare cpp executables
add_executable(lidar_pattern src/lidar_pattern.cpp)
add_executable(stereo_pattern src/stereo_pattern.cpp)
add_executable(velo2cam_stereoHough src/velo2cam_stereoHough.cpp)
add_executable(v2c_plane_segmentation src/plane.cpp)
add_executable(lidar_to_cameraview src/lidar_to_cameraview.cpp)
add_executable(saveImg src/saveImg.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(lidar_pattern
  ${catkin_EXPORTED_TARGETS}
  velo2cam_stereoHough_generate_messages_cpp
  ${PROJECT_NAME}_gencfg
)
add_dependencies(stereo_pattern
  ${catkin_EXPORTED_TARGETS}
  velo2cam_stereoHough_generate_messages_cpp
  ${PROJECT_NAME}_gencfg
)
add_dependencies(velo2cam_stereoHough
  ${catkin_EXPORTED_TARGETS}
  velo2cam_stereoHough_generate_messages_cpp
  ${PROJECT_NAME}_gencfg
)
add_dependencies(v2c_plane_segmentation 
  ${PROJECT_NAME}_gencfg
)
add_dependencies(lidar_to_cameraview
  ${catkin_EXPORTED_TARGETS}
  velo2cam_stereoHough_generate_messages_cpp
  ${PROJECT_NAME}_gencfg
)
add_dependencies(saveImg
  ${catkin_EXPORTED_TARGETS}
  velo2cam_stereoHough_generate_messages_cpp
  ${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(stereo_pattern
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(lidar_pattern
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(velo2cam_stereoHough
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${TinyXML_LIBRARIES}
)
target_link_libraries(v2c_plane_segmentation
  ${catkin_LIBRARIES}
)
target_link_libraries(lidar_to_cameraview
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(saveImg
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

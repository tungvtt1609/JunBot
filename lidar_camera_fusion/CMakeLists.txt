cmake_minimum_required(VERSION 3.0.2)
project(lidar_camera_fusion)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  cv_bridge
  image_transport
)

find_package(PCL REQUIRED)
find_package(OpenCV 4.0 REQUIRED)
find_package(Threads)
find_package(Ceres REQUIRED)
find_package(realsense2 REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(colorCamera2Lidar src/color_lidar_display.cpp src/common.h)
target_link_libraries(colorCamera2Lidar ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

add_executable(imageTransport src/imageTransport.cpp)
target_link_libraries(imageTransport ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES} ${realsense2_LIBRARY})
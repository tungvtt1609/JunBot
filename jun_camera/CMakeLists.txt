cmake_minimum_required(VERSION 3.0.2)
project(jun_camera)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  tf
  ddynamic_reconfigure
  diagnostic_updater
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jun_camera
  CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs
  cv_bridge
  image_transport
  ddynamic_reconfigure
  nav_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/jun_camera.cpp
# )

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

catkin_install_python(PROGRAMS scripts/set_cams_transforms.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )

    
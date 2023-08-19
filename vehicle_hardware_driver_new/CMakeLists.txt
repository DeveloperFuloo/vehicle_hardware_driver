cmake_minimum_required(VERSION 3.0.2)
project(vehicle_hardware_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  autoware_msgs
  visualization_msgs
)

add_message_files(
  FILES
	DoorLightDebug.msg
)

generate_messages(
  DEPENDENCIES
)
###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES chassis_driver
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  lib
  ${catkin_LIB_DIRS}
)
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/chassis_driver.cpp
# )


add_executable(vehicle_hardware_driver src/vehicle_hardware_driver.cpp src/main.cpp src/hardware_can_trans.cpp)
target_link_libraries(vehicle_hardware_driver ${catkin_LIBRARIES})
add_dependencies(vehicle_hardware_driver jtap_msgs_generate_messages)
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

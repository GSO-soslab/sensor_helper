cmake_minimum_required(VERSION 3.0.2)
project(sensor_helper)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## CMAKE module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

### Find Library

find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Eigen3 REQUIRED)


### Find ROS package
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs sensor_msgs nav_msgs
  geometry_msgs ds_sensor_msgs nortek_dvl
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 CATKIN_DEPENDS   roscpp rospy std_msgs sensor_msgs nav_msgs
                  geometry_msgs ds_sensor_msgs nortek_dvl
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

list(APPEND thirdparty_libraries
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

##########################################
############### build lib ###############
##########################################


##########################################
############### build ros ################
##########################################

add_executable(dvl_helper_node src/dvl_helper.cpp)
target_link_libraries(dvl_helper_node ${thirdparty_libraries})

add_executable(truth_helper_node src/truth_helper.cpp)
target_link_libraries(truth_helper_node ${thirdparty_libraries})

##########################################
############### build test ###############
##########################################

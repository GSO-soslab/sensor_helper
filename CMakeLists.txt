cmake_minimum_required(VERSION 3.0.2)
project(sensor_helper)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## CMAKE module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

### Find Library
SET(OpenCV_DIR /home/lin/develop/3rd/opencv/install/opencv-4.2.0/lib/cmake/opencv4/)
find_package(OpenCV 4.2.0 REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Eigen3 REQUIRED)


### Find ROS package
SET(cv_bridge_DIR /home/lin/develop/ros/ros_ws/devel/share/cv_bridge/cmake/)
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs sensor_msgs nav_msgs
  geometry_msgs ds_sensor_msgs nortek_dvl cv_bridge
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 CATKIN_DEPENDS   roscpp rospy std_msgs sensor_msgs nav_msgs
                  geometry_msgs ds_sensor_msgs nortek_dvl cv_bridge
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

list(APPEND thirdparty_libraries
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
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

add_executable(fls_helper_node src/fls_helper.cpp)
target_link_libraries(fls_helper_node ${thirdparty_libraries})

##########################################
############### build test ###############
##########################################

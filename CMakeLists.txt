cmake_minimum_required(VERSION 2.8.3)
project(throwing_ds_ros)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
#   mathlib
#   sgaussians
#   lpvDS
#   dynamic_reconfigure
)
#
find_package(Eigen3 REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
#  generate_dynamic_reconfigure_options(
#   cfg/seDS_params.cfg
#   cfg/LINEAR_params.cfg
# #   cfg/lpvDS_params.cfg
# )  

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES tossing_ds_ros
 CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs #lpvDS mathlib sgaussians dynamic_reconfigure 
)

###########
## Build ##
###########
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(H_FILES ${INCLUDE_DIR}/Utils.hpp                        # TO DO put this into a thirdarty folder
            ${INCLUDE_DIR}/throwingMotionGeneration.h
            ${INCLUDE_DIR}/throwingDS.h
            )

## Specify locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
# include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)


## Declare a C++ executable
set(SRCS
    src/throwingMotionGeneration.cpp
    src/throwingDS.cpp
    )

add_library(throwing_ds_ros ${H_FILES} ${SRCS})
target_link_libraries(throwing_ds_ros ${catkin_LIBRARIES})

#### Analytically Parametrized DS
add_executable(throwing_ds_node src/throwingMotionGeneration_node.cpp)
target_link_libraries(throwing_ds_node throwing_ds_ros)




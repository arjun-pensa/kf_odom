cmake_minimum_required(VERSION 2.8.3)
project(kf_odom)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

find_package(Eigen REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kf_odom
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs 
 DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(kf_odom src/kf_odom.cpp
                       src/helper.cpp)
add_dependencies(kf_odom ${catkin_EXPORTED_TARGETS})
target_link_libraries(kf_odom ${catkin_LIBRARIES})

install(TARGETS kf_odom
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
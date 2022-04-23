cmake_minimum_required(VERSION 3.0.2)
project(cdc-scripts)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
 CATKIN_DEPENDS roscpp rospy std_msgs
 DEPENDS EIGEN3
)

###########
## Build ##
###########


include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(extended_kalman_filter src/ekf.cpp)

target_link_libraries(extended_kalman_filter ${catkin_LIBRARIES})

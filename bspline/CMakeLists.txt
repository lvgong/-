cmake_minimum_required(VERSION 2.8.3)
project(bspline)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
#   plan_env
  cv_bridge
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES bspline
 CATKIN_DEPENDS
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_library( bspline 
    src/non_uniform_bspline.cpp 
    )
target_link_libraries( bspline
    ${catkin_LIBRARIES} 
    )  

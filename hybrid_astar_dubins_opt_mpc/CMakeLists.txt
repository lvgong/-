cmake_minimum_required(VERSION 3.0.2)
project(hybrid_astar_dubins_opt_mpc)

add_compile_options(-std=c++17)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(casadi REQUIRED)
find_package(ompl REQUIRED)
if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)
find_package(casadi REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  filters
  geometry_msgs
  message_generation
  nav_msgs
  roslaunch      
  roscpp
  rospy
  hybrid_astar_searcher
  mpc_controller
  bspline
  mpc_tracking
  tf2_ros
  tf2
  actionlib
  actionlib_msgs
  costmap_2d
  gazebo_msgs
  mbf_msgs
  dynamic_simulator
  astar_planner
)

## 配置 msg 源文件
add_message_files(
  FILES
  mpc_pose.msg
  mpc_arg.msg
  fusion_analysis.msg
)

add_service_files(
        DIRECTORY srv
        FILES
        chibot_task.srv
)

# 生成消息时依赖于 std_msgs
generate_messages(
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES create_maps
 CATKIN_DEPENDS 
 LIBRARIES ego_planner
 filters 
 geometry_msgs 
 message_runtime 
 roscpp 
 rospy 
 std_msgs 
 nav_msgs 
 hybrid_astar_searcher
 mpc_controller
 bspline
 mpc_tracking
#  DEPENDS system_lib
 actionlib_msgs
 costmap_2d
 gazebo_msgs
 mbf_msgs
 tf2
 tf2_ros
 dynamic_simulator
 astar_planner
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
#   ${CMAKE_SOURCE_DIR}/motion_control/decompUtil
  ${CMAKE_SOURCE_DIR}/mpc_controller/include
  ${CMAKE_SOURCE_DIR}/bspline/include
  ${CMAKE_SOURCE_DIR}/mpc_tracking/include
  ${CMAKE_SOURCE_DIR}/astar_planner/include
)
include_directories(include ${OMPL_INCLUDE_DIRS})
include_directories(include ${mpc_controller_INCLUDE_DIRS})
include_directories(include ${astar_planner_INCLUDE_DIRS})


# 为了使用#include <ros/ros.h>需要${catkin_INCLUDE_DIRS}
# include_directories(include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS})
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/create_maps.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(mpc_bridge_node node/mpc_bridge_node.cpp src/mpc_bridge.cpp)
add_dependencies(mpc_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_bridge_node ${catkin_LIBRARIES})
install(TARGETS mpc_bridge_node
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#  add_executable(simple_demo_node src/simple_demo_node.cpp)
#  add_executable(ompl_test_node src/ompl_test.cpp)
#  add_executable(hybrid_astar_test src/hybrid_astar_test.cpp)
 add_executable(chibot_hybrid_astar_dubins_opt 
                node/hybrid_astar_dubins_opt_node.cpp
                src/hybrid_astar_dubins_opt.cpp
                src/half_struct_planner.cpp)

add_executable(chibot_hybrid_astar_mpc 
node/hybrid_astar_mpc_node.cpp
src/hybrid_astar_mpc.cpp
src/half_struct_planner.cpp)


# add_executable(hybrid_astar_Bspline_LMPC 
# node/hybrid_astar_Bspline_LMPC_node.cpp
# src/hybrid_astar_Bspline_LMPC.cpp
# src/half_struct_planner.cpp)

# add_executable(hybrid_astar_Bspline_NMPC 
# node/hybrid_astar_Bspline_NMPC_node.cpp
# src/hybrid_astar_Bspline_NMPC.cpp
# src/half_struct_planner.cpp)

add_executable(astar_NMPC_CBF 
node/astar_NMPC_CBF_node.cpp
src/astar_NMPC_CBF.cpp
src/half_struct_planner.cpp)

add_executable(simple_astar_test 
node/simple_astar_test_node.cpp
src/simple_astar_test.cpp
src/half_struct_planner.cpp)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#  target_link_libraries(simple_demo_node
#    ${catkin_LIBRARIES}
#  )
#  target_link_libraries(ompl_test_node
#  ${catkin_LIBRARIES}
#  ${OMPL_LIBRARIES}
#  hybrid_astar_searcher
# )

# target_link_libraries(hybrid_astar_test
# ${catkin_LIBRARIES}
# ${OMPL_LIBRARIES}
# hybrid_astar_searcher

# )
target_link_libraries(chibot_hybrid_astar_dubins_opt
${catkin_LIBRARIES}
${OMPL_LIBRARIES}
mpc_controller
libqpOASES.so
hybrid_astar_searcher
)

target_link_libraries(chibot_hybrid_astar_mpc
${catkin_LIBRARIES}
${OMPL_LIBRARIES}
hybrid_astar_searcher

)

# target_link_libraries(hybrid_astar_Bspline_LMPC
# ${catkin_LIBRARIES}
# ${OMPL_LIBRARIES}
# mpc_controller
# libqpOASES.so
# hybrid_astar_searcher
# )

# target_link_libraries(hybrid_astar_Bspline_NMPC
# ${catkin_LIBRARIES}
# ${OMPL_LIBRARIES}
# bspline
# mpc_tracking
# /usr/local/lib/libcasadi.so.3.7
# hybrid_astar_searcher
# )

target_link_libraries(astar_NMPC_CBF
${catkin_LIBRARIES}
${OMPL_LIBRARIES}
bspline
astar_planner
mpc_tracking
/usr/local/lib/libcasadi.so.3.7
hybrid_astar_searcher
)

target_link_libraries(simple_astar_test
${catkin_LIBRARIES}
${OMPL_LIBRARIES}
hybrid_astar_searcher
)
add_dependencies(chibot_hybrid_astar_dubins_opt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(chibot_hybrid_astar_mpc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(hybrid_astar_Bspline_LMPC ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(hybrid_astar_Bspline_NMPC ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(astar_NMPC_CBF ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(simple_astar_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(TARGETS chibot_hybrid_astar_dubins_opt chibot_hybrid_astar_mpc  hybrid_astar_Bspline_LMPC hybrid_astar_Bspline_NMPC
# install(TARGETS astar_NMPC_CBF simple_astar_test
install(TARGETS simple_astar_test chibot_hybrid_astar_dubins_opt chibot_hybrid_astar_mpc astar_NMPC_CBF
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

install(DIRECTORY launch param rviz 
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_create_maps.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

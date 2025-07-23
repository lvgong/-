cmake_minimum_required(VERSION 2.8.3)
project(chibot_slam)
set(PCL_H "/usr/include/pcl-1.10")
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

find_package(catkin REQUIRED COMPONENTS
		geometry_msgs
		nav_msgs
		roscpp
		roslaunch
		sensor_msgs
		tf
		tf2
		tf2_ros
		std_msgs 
		sensor_msgs
		message_filters
		)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(CATKIN_DEPENDS
		geometry_msgs
		nav_msgs
		roscpp
		std_msgs
		sensor_msgs
		tf2
		tf2_ros
		)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_H})


add_executable(chibot_relocation node/relocation.cpp)
add_dependencies(chibot_relocation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(chibot_relocation ${catkin_LIBRARIES})
install(TARGETS chibot_relocation
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)

add_executable(rotate_map node/rotate_map.cpp)
add_dependencies(rotate_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rotate_map ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_include_directories(rotate_map PRIVATE
                           include
                           ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS})

install(TARGETS rotate_map
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)

add_executable(laser_double src/laser_double.cpp)
add_dependencies(laser_double ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(laser_double ${Boost_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS laser_double
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)


install(DIRECTORY launch param rviz map path
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
		)

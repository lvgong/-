cmake_minimum_required(VERSION 2.8.3)

project(chibot_simu)

find_package(catkin REQUIRED COMPONENTS
		roscpp
		rospy
		std_msgs
		roslaunch
		message_generation
		message_runtime
		)

add_message_files(FILES
		chibot_pose.msg
		fusion_analysis.msg
		)

generate_messages()

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs message_generation message_runtime )

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(chibot_bridge_node node/chibot_bridge_node.cpp src/chibot_bridge.cpp)
add_dependencies(chibot_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(chibot_bridge_node ${catkin_LIBRARIES})
install(TARGETS chibot_bridge_node
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)

catkin_install_python(PROGRAMS
		scripts/teleop_key.py
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)
		
install(DIRECTORY include
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		FILES_MATCHING PATTERN "*.h"
		)
install(DIRECTORY config launch meshes urdf worlds
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
		)

# foreach(dir config launch meshes urdf)
# 	install(DIRECTORY ${dir}/
# 		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

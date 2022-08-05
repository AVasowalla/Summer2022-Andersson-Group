cmake_minimum_required(VERSION 2.8.3)
project(offboard_test)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(offboard_test src/pub_setpoint.cpp)
add_executable(offboard_test_att src/pub_att.cpp)
target_link_libraries(offboard_test ${catkin_LIBRARIES})
target_link_libraries(offboard_test_att ${catkin_LIBRARIES})

install(
	TARGETS offboard_test
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

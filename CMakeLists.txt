cmake_minimum_required(VERSION 2.8.3)
project(pincher_commander)
find_package(catkin REQUIRED
  	actionlib 
	tf
	tf_conversions
	trajectory_msgs
	cmake_modules
	moveit_core
	moveit_ros_perception
	moveit_ros_planning_interface
	moveit_ros_control_interface
)

## Enable C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
  	actionlib 
	tf
	tf_conversions
	trajectory_msgs
	cmake_modules
	moveit_core
	moveit_ros_perception
	moveit_ros_planning_interface
	moveit_ros_control_interface
	pluginlib
  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_moveit_node src/pincher_commander_moveit/pincher_commander_moveit.cpp)
add_dependencies(${PROJECT_NAME}_moveit_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_moveit_node ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_moveit_node PROPERTIES OUTPUT_NAME node PREFIX "moveit_")


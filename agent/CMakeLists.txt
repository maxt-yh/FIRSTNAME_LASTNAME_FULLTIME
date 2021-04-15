cmake_minimum_required(VERSION 3.0.2)
project(agent)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES log
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
	src/libAgent.cc
)

add_executable(agent_node src/Agent.cc)
target_link_libraries(agent_node ${PROJECT_NAME} ${catkin_LIBRARIES})

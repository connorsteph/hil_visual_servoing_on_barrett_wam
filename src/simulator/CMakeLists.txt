cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-ignored-attributes")

project(simulator)

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  std_msgs
  message_generation
  cmake_modules
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
)


find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  INCLUDE_DIRS include
  #  LIBRARIES sim
  CATKIN_DEPENDS message_runtime
  #  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(simulator src/simulator.cpp)
target_link_libraries(simulator
  ${catkin_LIBRARIES}
)

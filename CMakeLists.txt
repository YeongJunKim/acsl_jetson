cmake_minimum_required(VERSION 2.8.3)
project(acsl_jetson)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES acsl_jetson
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  node/get_pos_from_jetson.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


cmake_minimum_required(VERSION 2.8.3)
project(acsl_jetson)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  boxInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
#  INCLUDE_DIRS include
#  LIBRARIES acsl_jetson
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(acsl_jetson_node node/get_pos_from_jetson_cpp.cpp)

target_link_libraries(acsl_jetson_node
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  node/get_pos_from_jetson.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


cmake_minimum_required(VERSION 3.0.2)
project(limo_application)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib
  actionlib_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    roscpp
    rospy
    std_msgs
    message_generation
    actionlib
    actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/Line_detect.py
  scripts/control.py
  scripts/follow_yellow_line.py
  scripts/limo_control_node.py
  scripts/stop.py
  scripts/turn_absolute.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
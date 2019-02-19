cmake_minimum_required(VERSION 2.8.3)
project(refills_perception_interface)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  refills_msgs
  geometry_msgs
  rospy)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS actionlib_msgs refills_msgs geometry_msgs)

catkin_install_python(PROGRAMS scripts/dummy_interface.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

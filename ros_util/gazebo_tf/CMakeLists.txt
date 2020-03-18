cmake_minimum_required(VERSION 2.8.3)
project(gazebo_tf)

find_package(catkin REQUIRED COMPONENTS
    rospy
    tf2_ros
    geometry_msgs
    gazebo_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES slam
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


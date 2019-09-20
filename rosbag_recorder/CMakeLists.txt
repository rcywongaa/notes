cmake_minimum_required(VERSION 2.8.3)
project(rosbag_recorder)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    #INCLUDE_DIRS include
    #LIBRARIES
    #CATKIN_DEPENDS roscpp
    #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(rosbag_recorder_node src/rosbag_recorder_node.cpp)

add_dependencies(rosbag_recorder_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(rosbag_recorder_node
    ${catkin_LIBRARIES} 
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS rosbag_recorder_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

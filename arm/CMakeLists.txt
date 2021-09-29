cmake_minimum_required(VERSION 3.0.2)
project(arm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
geometry_msgs
message_generation
moveit_msgs
roscpp
rospy
std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################



## Generate services in the 'srv' folder
add_service_files(
   FILES
   step_srv.srv

 )


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   moveit_msgs
   std_msgs  # Or other packages containing msgs
 )



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arm_move
CATKIN_DEPENDS geometry_msgs message_runtime moveit_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)




#############
## Install ##
#############



## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   nodes/mover.py
   nodes/arm_tutorial.py
   nodes/show_off.py
   tests/test_collision.test
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )



#############
## Testing ##
#############



if(CATKIN_ENABLE_TESTING)
   find_package(rostest REQUIRED)
   add_rostest(tests/arm.test)
endif()

cmake_minimum_required(VERSION 2.8.3)
project(localization_tests)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
              atf_core
              roslaunch
              rospy
              rostopic
              )
catkin_package(
   CATKIN_DEPENDS rospy
)


#############
## Install ##
#############

install(PROGRAMS scripts/application.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############

roslaunch_add_file_check(launch)
atf_test()
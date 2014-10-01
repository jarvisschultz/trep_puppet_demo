# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(trep_puppet_demo)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp roslib rospy geometry_msgs sensor_msgs std_srvs interactive_markers tf)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})## Generate added messages and services with any dependencies listed here
#generate_messages(
#    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp roslib rospy geometry_msgs sensor_msgs std_srvs interactive_markers kdl tf skeletonmsgs_nu skeletontracker_nu
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

include_directories(
 include ${catkin_INCLUDE_DIRS}
)
add_executable(keyboard_interface src/keyboard_interface.cpp)
target_link_libraries(keyboard_interface ${catkin_LIBRARIES})
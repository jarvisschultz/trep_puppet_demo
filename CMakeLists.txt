# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(trep_puppet_demo)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  interactive_markers
  roscpp
  roslib
  rospy
  sensor_msgs
  std_srvs
  tf)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS geometry_msgs interactive_markers roscpp roslib rospy sensor_msgs std_srvs tf
)

include_directories(
 include ${catkin_INCLUDE_DIRS}
)
add_executable(keyboard_interface src/keyboard_interface.cpp)
target_link_libraries(keyboard_interface ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  src/frame_mappings.py
  src/interactive_marker_controls.py
  src/marker_path.py
  src/puppet_definitions.py
  src/puppet_simulator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS keyboard_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

cmake_minimum_required(VERSION 3.0.2)
project(homework)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
    std_msgs
    geometry_msgs
)

add_service_files(
  FILES
  directKinematics.srv
  inverseKinematics.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES move
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)


add_executable(move src/move.cpp)
target_link_libraries(move ${catkin_LIBRARIES})
add_dependencies(move move)

catkin_install_python(PROGRAMS 
  scripts/directKinematics.py
  scripts/inverseKinematics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
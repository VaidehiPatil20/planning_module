cmake_minimum_required(VERSION 3.0.2)
project(planning_module)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
)

catkin_python_setup()

## Generate messages in the 'msg' folder
# add_message_files(
#   DIRECTORY msg
# )

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  trajectory_msgs
)




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES location_manager
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
# include
${catkin_INCLUDE_DIRS}
)

# Find python scripts
file(GLOB_RECURSE PYTHON_SCRIPTS
  "nodes/*"
  "scripts/*"
)

# Install python scripts
catkin_install_python(PROGRAMS
	${PYTHON_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



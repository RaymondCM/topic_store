cmake_minimum_required(VERSION 2.8.3)
project(topic_store)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  genmsg
  std_msgs
  sensor_msgs
  actionlib_msgs
  ros_numpy
  message_generation
)

# ==============================================
# Python scripts setup
# ==============================================
catkin_python_setup()

# ==============================================
# Service/Message files setup
# ==============================================
# add_service_files( FILES )

add_message_files( FILES )

add_action_files(DIRECTORY action FILES CollectData.action)

generate_messages(DEPENDENCIES std_msgs sensor_msgs topic_store actionlib_msgs)

# ==============================================
# Setup
# ==============================================
catkin_package(CATKIN_DEPENDS std_msgs message_runtime)

# ==============================================
# Install
# ==============================================
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

file(GLOB python_scripts_DIR "scripts/*.py")
catkin_install_python(PROGRAMS ${python_scripts_DIR} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

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
if(DEFINED ENV{INSTALL_TOPIC_STORE_PYTHON_FROM_PIP})
    # Install python dependancies (first try to install locally, then to user site_packages)
    find_package(PythonInterp REQUIRED)
    set(return_status "1")
    set(failed_commands "")
    foreach(flag "" "--user")
        set(command "pip install -r ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt ${flag}")
        execute_process (COMMAND bash -c "${command}" OUTPUT_VARIABLE output_var ERROR_VARIABLE error_var
                         RESULT_VARIABLE return_status)
        message(WARNING "${command}Not a warning just output of pip install dependancies:\n${output_var}")

        if(return_status EQUAL "0")
            break()
        endif()
        list(APPEND failed_commands "\n\t- '${command}'")
    endforeach()
    if(NOT return_status EQUAL "0")
        message(SEND_ERROR "${output_var}")
        message(SEND_ERROR "${error_var}")
        message(FATAL_ERROR "Failed to install python requirements with:${failed_commands}")
    endif()
endif()

catkin_python_setup()

# ==============================================
# Service/Message files setup
# ==============================================
# add_service_files( FILES )

# add_message_files( FILES )

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
file(GLOB python_src_DIR "src/**/*.py")
catkin_install_python(
    PROGRAMS ${python_src_DIR} ${python_scripts_DIR}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ==============================================
# Tests
# ==============================================
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/topic_store.test)
endif()
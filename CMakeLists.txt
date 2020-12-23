cmake_minimum_required(VERSION 2.8.3)
project(auv_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  genmsg
  roscpp
  sensor_msgs
  std_msgs
  tf
  uuv_gazebo_ros_plugins_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# catkin_python_setup()

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  uuv_gazebo_ros_plugins_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    auv_controller
  CATKIN_DEPENDS 
    roscpp 
    sensor_msgs 
    std_msgs 
    tf 
    uuv_gazebo_ros_plugins_msgs
  DEPENDS 
    system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/AUVController.cpp
  src/ArmsAUVControllerROS.cpp
)

add_library(file_writer
  src/file_writer.cpp
) 

add_executable(armsauv_controller_test_node
  test/ArmsAUVControllerTest.cpp
)

add_dependencies(armsauv_controller_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(armsauv_controller_test_node ${PROJECT_NAME} file_writer ${catkin_LIBRARIES})

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
  armsauv_msgs
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
add_service_files(
  FILES
  ResetCtrlState.srv
  SetMissionStatus.srv
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

add_library(auv_base_controller
  src/Common.cpp
  src/AUVBaseController.cpp
  src/AUVControllerWithFF.cpp
  src/AUVControllerNoFF.cpp
  src/AUVControllerROS.cpp
  src/PIDController.cpp
  src/AUVControllerXF.cpp
)

add_library(auv_pid_controller
  src/Common.cpp
  src/AUVPIDController.cpp
  src/PIDController.cpp
  src/AUVPIDControllerXF.cpp
  src/AUVPIDControllerNoFF.cpp
  src/AUVPIDControllerROS.cpp
)

add_library(auv_controller_exp
  src/Common.cpp
  src/AUVBaseController.cpp
  src/AUVControllerWithFF.cpp
  src/AUVControllerNoFF.cpp
  src/PIDController.cpp
  src/AUVControllerXF.cpp
  src/AUVControllerExp.cpp
)

add_library(auv_traj_follow_manager_ros
  src/AUVTrajFollowManagerROS.cpp
)

add_library(file_writer
  src/file_writer.cpp
) 

add_library(auv_ctrlmsgs_recorder
  src/AUVCtrlMsgsRecorder.cpp
  src/AUVCtrlMsgsRecorderROS.cpp
)

add_library(pid_factory
  src/PIDController.cpp
)

add_library(async_tcp_server
  src/async_tcp_server.cpp
)

add_library(async_tcp_client
  src/async_tcp_client.cpp
)

add_library(tcp_server
  src/tcp_server.cpp
)

add_library(tcp_client
  src/tcp_client.cpp  
)

add_executable(reset_control_state_node
    nodes/ResetCtrlStateClient.cpp
)

add_executable(armsauv_controller_test_node
  test/ArmsAUVControllerTest.cpp
)

add_executable(auv_controller_test_node
  test/AUVControllerTest.cpp
)

add_executable(auv_pid_controller_test_node
  test/AUVPIDControllerTest.cpp
)

add_executable(auv_controller_exp_test_node
  test/AUVControllerExpTest.cpp
)

add_executable(auv_traj_follow_manager_node
  nodes/AUVTrajFollowMissionNode.cpp
)

add_executable(auv_ctrlmsgs_recorder_node
  nodes/AUVCtrlMsgsRecordNode.cpp
)

add_executable(async_tcp_client_test_node
  test/async_tcp_client_test.cpp
)

add_executable(async_tcp_server_test_node
  test/async_tcp_server_test.cpp
)

add_executable(tcp_server_test_node
  test/tcp_server_test.cpp
)

add_executable(tcp_client_test_node
  test/tcp_client_test.cpp
)

# add_executable(auv_db_parser_test_node
#   test/AUVDBParser.cpp
# )

add_dependencies(reset_control_state_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(armsauv_controller_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(auv_controller_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(auv_pid_controller_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(auv_controller_exp_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(auv_traj_follow_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(auv_ctrlmsgs_recorder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(async_tcp_client_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(async_tcp_server_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(tcp_server_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(tcp_client_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_dependencies(auv_db_parser_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(reset_control_state_node ${PROJECT_NAME} ${catkin_LIBRARIES})

target_link_libraries(armsauv_controller_test_node ${PROJECT_NAME} file_writer ${catkin_LIBRARIES})

target_link_libraries(auv_controller_test_node auv_base_controller file_writer ${catkin_LIBRARIES})

target_link_libraries(auv_pid_controller_test_node auv_pid_controller file_writer ${catkin_LIBRARIES})

target_link_libraries(auv_controller_exp_test_node auv_controller_exp file_writer ${catkin_LIBRARIES})

target_link_libraries(auv_traj_follow_manager_node auv_traj_follow_manager_ros auv_controller_exp file_writer ${catkin_LIBRARIES})

target_link_libraries(auv_ctrlmsgs_recorder_node auv_ctrlmsgs_recorder file_writer ${catkin_LIBRARIES})

target_link_libraries(async_tcp_client_test_node async_tcp_client ${catkin_LIBRARIES})

target_link_libraries(async_tcp_server_test_node async_tcp_server ${catkin_LIBRARIES})

target_link_libraries(tcp_server_test_node tcp_server ${catkin_LIBRARIES})

target_link_libraries(tcp_client_test_node tcp_client ${catkin_LIBRARIES})

# Install python scripts
catkin_install_python(PROGRAMS script/auv_csv_parser_node script/auv324_csv_parser_node
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


## auv_controller  
auv_controller is a ROS package which implement a common slide controller for AUV with dynamic model.  

# slide model control  
Controller integrated with slide model control algorithm in decoupled lateral and horizontal plane.  

# ROS nodes   
1. reset_control_state_node  
node file: nodes/ResetCtrlStateClient.cpp  
Usage: rosrun auv_controller reset_ctrl_state <type>  
<type>: target state of AUV  
  
2. armsauv_controller_test_node  
node file: nodes/ArmsAUVControllerTest.cpp  
Usage: rosrun auv_controller armsauv_controller_test_node  
   
3. auv_controller_test_node  
node file: nodes/AUVControllerTest.cpp  
Usage: rosrun auv_controller auv_controller_test_node <no_ff> <x> <debug>  
<no_ff>: launch model without front fins, arg should be string 'no_ff'  
<x>: launch model with X type rudder, arg should be string 'x'  
<debug>: launch model in debug mode, arg should be string 'debug'  
  
4. auv_pid_controller_test_node  
node file: nodes/AUVPIDControllerTest.cpp  
Usage: rosrun auv_controller auv_pid_controller_test_node <no_ff> <x> <debug>
<no_ff>: launch model without front fins, arg should be string 'no_ff'  
<x>: launch model with X type rudder, arg should be string 'x'  
<debug>: launch model in debug mode, arg should be string 'debug'  
  
5. auv_traj_follow_manager_node  
node file: nodes/AUVTrajFollowMissionNode.cpp  
Usage: rosrun auv_controller auv_traj_follow_manager_node <auv_ns> <no_ff> <x> <debug>  
<auv_ns>: namespace of AUV model  
<no_ff>: launch model without front fins, arg should be string 'no_ff'  
<x>: launch model with X type rudder, arg should be string 'x'  
<debug>: launch model in debug mode, arg should be string 'debug'  
  
6. auv_ctrlmsgs_recorder_node  
node file: nodes/AUVCtrlMsgsRecordNode.cpp  
Usage: rosrun auv_ctrlmsgs_recorder  
  
7. async_tcp_client_test_node  
node file: nodes/async_tcp_client_test_node 
Usage: rosrun async_tcp_client_test_node <host> <port>  
<host>: IP of TCP server  
<port>: Port of TCP server  
  
8. async_tcp_server_test_node  
node file: nodes/async_tcp_server_test_node  
Usage: rosrun async_tcp_server_test_node <port>  
<port>: Listen port of TCP server  


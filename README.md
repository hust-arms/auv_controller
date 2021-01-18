## auv_controller  
auv_controller is a ROS package which implement a common slide controller for AUV with dynamic model.  

# slide model control  
Controller integrated with slide model control algorithm in lateral and horizontal plane.  
Rotary velocity of propeller should be 1400 or higher while desired depth is set as 20m and desired y is set as 10m.  

# Run controller  
rosrun auv_controller armsauv_controller_test_node  

/**
  ******************************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  tcp_server_test.cpp
  * - Author:    Zhao Wang
  * - Date:      2021/3/16
  * - Brief:     tcp server test
********************************************************************************************
**/

#include "ros/ros.h"

#include "auv_controller/tcp_server.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tcp_server_test");
	ros::NodeHandle nh;
	
	ROS_INFO("Prepare to create server");
	TCPServer tcp_s(6694);
	
	uint8_t grid_data_stack[10];
    for(int i = 0; i < 10; ++i)
    {
        grid_data_stack[i] = 0x08;
    }
	
	try{
		ROS_INFO("Set buffer");
		tcp_s.setWriteBuffer(grid_data_stack, sizeof(grid_data_stack) / sizeof(uint8_t)); 
		ROS_INFO("Finish buffer setting");
	}
	catch(std::exception& e){
		ROS_INFO("Error to set write buffer!");
	}
	
	try{
		tcp_s.run();
	}
	catch(std::exception& e){
		ROS_ERROR("Error to start tcp server");
	}
	
	return 0;
}

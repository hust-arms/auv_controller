
/**
  ******************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  tcp_client_test.cpp
  * - Author:    Zhao Wang
  * - Date:      2021/3/16
  * - Brief:     tcp client test code
  ******************************************************************************
*/

#include "ros/ros.h"
#include <iostream>
#include "auv_controller/tcp_client.h"

void testPrintBuffer(std::vector<uint8_t>& buf){
	if(buf.size() > 0){
		for(auto element : buf){
			std::cout << static_cast<int>(element) << " ";
		}
		std::cout << std::endl;
	}
	else{
		ROS_WARN("Buffer is empty!");
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "tcp_server_test_node");
	ros::NodeHandle nh;

	std::vector<uint8_t> data_buf;

	// set tcp client
	TCPClient tcp_c("127.0.0.1", 2114, 15);
	tcp_c.setRecvProcess(boost::bind(&testPrintBuffer, _1));
	
	try{
		tcp_c.run();
	}
	catch(std::exception& e){
		ROS_ERROR("Cannot start tcp client normally");
	}

	return 0;
}

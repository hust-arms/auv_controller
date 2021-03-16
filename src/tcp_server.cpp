/**
  ******************************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  tcp_server.cpp
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2020/1/2
  * - Brief:     Implementation of TCPServer class to realize communication through TCP protocol
********************************************************************************************
**/

#include "ros/ros.h"
#include "auv_controller/tcp_server.h"
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

TCPServer::TCPServer(int port_num) : acceptor_(tcp_io_, endpoint_type(boost::asio::ip::tcp::v4(), port_num)){
	TCPServer::accept();
}

TCPServer::~TCPServer(){
	// if(sock_t_){
	// 	delete sock_t_;
	// }
	// sock_t_ = nullptr;
}

void TCPServer::setWriteBuffer(uint8_t* buf, int buf_len){
	{
		write_lock write_buf(buf_mutex_); // set write buffer data
		write_buf_ = buf;
		w_buf_len_ = buf_len;
	}
}

void TCPServer::accept(){
	// sock_t_ = new socket_type(tcp_io_);
	// sock_ptr sock(sock_t_); // create socket pointer
	sock_ptr sock(new socket_type(tcp_io_));
	acceptor_.async_accept(*sock, boost::bind(&this_type::acceptHandler, this, boost::asio::placeholders::error, sock));
}

void TCPServer::acceptHandler(const error_code& ec, sock_ptr sock){
	if(ec){
		return;
	}
	{
		read_lock read_buf(buf_mutex_); // read buffer data
	    	try{
			sock->async_write_some(boost::asio::buffer(write_buf_, w_buf_len_), boost::bind(&this_type::writeHandler, this, boost::asio::placeholders::error));
		}
		catch(std::exception& e){
			ROS_ERROR("tcp_server: error to send message");
			return;
		}
	}
	if(ros::ok()){
		TCPServer::accept();
	}
	else{
		return;
	}
}

void TCPServer::writeHandler(const error_code& ec){
	if(ec){
		return;
	}
	testPrint();
}

void TCPServer::testPrint(){
	// print data in write buffer
    std::cout << "Write test" << std::endl;
}

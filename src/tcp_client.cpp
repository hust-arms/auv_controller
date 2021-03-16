/**
  ******************************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  tcp_client.cpp
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2020/1/2
  * - Brief:     Implementation of TCPClient class to realize communication through TCP protocol
********************************************************************************************
**/
#include <iostream>
#include <string>
#include "auv_controller/tcp_client.h"
#include "ros/ros.h"

TCPClient::TCPClient(std::string ip, size_t port, size_t buf_size) : tcp_buf_(buf_size, 0), tcp_ep_(address_type::from_string(ip), port), sock_t_(nullptr), is_listen_(true)
{
    // Initialize components
    connect();
}

TCPClient::~TCPClient(){
    // if(sock_t_ != nullptr){
    //     delete sock_t_;
    // sock_t_ = nullptr;
    // }
}

void TCPClient::stopListen(){
    {
        write_lock set_status(listen_mutex_); // writting lock
        is_listen_ = false;
    }
}


void TCPClient::setRecvProcess(recv_proc_type recv_proc){
    recv_proc_ = recv_proc;
}


void TCPClient::getBuf(buffer_type& buf){
    {
        read_lock read_buf(buf_mutex_); // reading buffer
        buf.clear();
        buf = tcp_buf_;
    }
}

void TCPClient::connect()
{
    // sock_t_ = new socket_type(tcp_io_);
    // sock_ptr sock(sock_t_); // create socket pointer
    sock_ptr sock(new socket_type(tcp_io_));
    // boost::thread read_thread(boost::bind(&this_type::recvData, this, sock)); // start read thread   
    sock->async_connect(tcp_ep_, boost::bind(&this_type::connHandler, this, boost::asio::placeholders::error, sock));
}

void TCPClient::recvData(sock_ptr sock){
    while(is_listen_){
      sock->async_connect(tcp_ep_, boost::bind(&this_type::connHandler, 
                                                        this, boost::asio::placeholders::error, sock));
      boost::this_thread::sleep(boost::posix_time::millisec(200)); // thread sleep
    }
}

void TCPClient::connHandler(const error_code& ec, sock_ptr sock){
    if(ec){
	ROS_WARN("tcp_client: error in connect");
        return;
    }
    // read data from port
    {
        write_lock write_buf(buf_mutex_); // writting buffer
        sock->async_read_some(boost::asio::buffer(tcp_buf_), boost::bind(&this_type::readHandler, this, boost::asio::placeholders::error));
    }
    if(ros::ok()){
	connect();
    }
    else{
        return;
    }	
}


void TCPClient::readHandler(const error_code& ec){
    if(ec){
        return;
    }
    testPrint(tcp_buf_); 
    // testMavUnpack(tcp_buf_);
    
    {
        read_lock read_buf(buf_mutex_);
        recv_proc_(tcp_buf_);
    }
}

void TCPClient::testPrint(buffer_type& buf){
    // print data
    if(buf.size()){
	for(auto x : buf){
	    std::cout << unsigned(x) << " ";
	}
	std::cout << std::endl;
    }
    else{
        ROS_WARN("tcp_client: buffer is empty!");
    }
}

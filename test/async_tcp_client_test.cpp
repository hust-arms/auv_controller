/*
 * Create Date: Monday, Martch 1st 2021, 10:01:12 pm
 * Author: zhao wang
 * Copyright (c) hust-arms
 */
#include <ros/ros.h> 
#include "auv_controller/async_tcp_client.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "async_tcp_client_test");
  ros::NodeHandle nh;

  try
  {
    if (argc != 3)
    {
      std::cerr << "Usage: client <host> <port>\n";
      return 1;
    }

    boost::asio::io_service io_service;
    tcp::resolver r(io_service);
    tcp_client::AsyncTCPClient c(io_service);

    c.start(r.resolve(tcp::resolver::query(argv[1], argv[2])));

    std::cout << "TCP client has been created!" << std::endl;

    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  // ros::spin();

  return 0;
}

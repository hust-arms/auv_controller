/*
 * Create Date: Monday, Martch 1st 2021, 10:01:12 pm
 * Author: zhao wang
 * Copyright (c) hust-arms
 */
 
#include "async_tcp_client.h"

int main(int argc, char* argv[])
{
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

    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}
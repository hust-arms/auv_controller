#include <ros/ros.h>
#include "auv_controller/async_tcp_server.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "async_tcp_server_test");
  ros::NodeHandle nh;

  try
  {
    if(argc != 2)
    {
        std::cerr << "Usage: async_tcp_server_test <lisent_port>" << std::endl;
        return -1;
    }

    boost::asio::io_service io_service;
    // tcp_server::tcp_server server(io_service);
    tcp_server::tcp_server server(io_service, tcp::endpoint(tcp::v4(), atoi(argv[1])));

    std::cout << "TCP server has been created successfully" << std::endl;

    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}

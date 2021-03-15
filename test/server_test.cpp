#include "server.h"

int main(int argc, char* argv[])
{
  try
  {
    using namespace std; // For atoi.

    if (argc != 4)
    {
      std::cerr << "Usage: server <listen_port> <bcast_address> <bcast_port>\n";
      return 1;
    }

    boost::asio::io_service io_service;

    tcp::endpoint listen_endpoint(tcp::v4(), atoi(argv[1]));

    udp::endpoint broadcast_endpoint(
        boost::asio::ip::address::from_string(argv[2]), atoi(argv[3]));

    tcp_server::server s(io_service, listen_endpoint, broadcast_endpoint);

    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}
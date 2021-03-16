//
// async_tcp_server.h
// ~~~~~~~~~~
//
// Copyright (c) 2003-2011 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <ctime>
#include <iostream>
#include <string>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

std::string make_daytime_string()
{
  using namespace std; // For time_t, time and ctime;
  time_t now = time(0);
  return ctime(&now);
}

namespace tcp_server
{
class tcp_connection
  : public boost::enable_shared_from_this<tcp_connection>
{
public:
  typedef boost::shared_ptr<tcp_connection> pointer;

  static pointer create(boost::asio::io_service& io_service)
  {
    return pointer(new tcp_connection(io_service));
  }

  tcp::socket& socket()
  {
    return socket_;
  }

  void start_write();

  void start_read();

private:
  tcp_connection(boost::asio::io_service& io_service)
    : socket_(io_service)
  {
  }

  void handle_write(const boost::system::error_code& /*error*/,
      size_t /*bytes_transferred*/)
  {
	  std::cout << "Write message into buffer: " << message_ << std::endl;
  }

  void handle_read(const boost::system::error_code& ec,
      size_t /*bytes_transferred*/)
  {
      if(!ec)
      {
          std::string line;
          std::istream is(&read_buffer_);
          std::getline(is, line);

          if(!line.empty())
          {
              std::cout << "Received: " << line << std::endl;
          }
      }
  }

  tcp::socket socket_;
  std::string message_;
  boost::asio::streambuf read_buffer_;
}; // class tcp_connection

class tcp_server
{
public:
  tcp_server(boost::asio::io_service& io_service)
    : acceptor_(io_service, tcp::endpoint(tcp::v4(), 2115))
  {
    start_accept();
  }

private:
  void start_accept();
  
  void handle_accept(tcp_connection::pointer new_connection,
      const boost::system::error_code& error);

  tcp::acceptor acceptor_;
};// class tcp_server
};//ns

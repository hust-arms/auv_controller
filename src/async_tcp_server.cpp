//
// async_tcp_server.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2011 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include "auv_controller/async_tcp_server.h"

namespace tcp_server
{
	/* tcp_connection functions */
	///////////////////////////
	void tcp_connection::start_write()
    {
      message_ = make_daytime_string();
    
      boost::asio::async_write(socket_, boost::asio::buffer(message_),
          boost::bind(&tcp_connection::handle_write, shared_from_this(),
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));

    }

	///////////////////////////
    void tcp_connection::start_read()
    {
      boost::asio::async_read_until(socket_, read_buffer_, '\n', 
          boost::bind(&tcp_connection::handle_read, shared_from_this(), 
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
    }
	
	/* tcp_server functions */
	///////////////////////////
	void tcp_server::start_accept()
    {
      tcp_connection::pointer new_connection =
        tcp_connection::create(acceptor_.get_io_service());
    
      acceptor_.async_accept(new_connection->socket(),
          boost::bind(&tcp_server::handle_accept, this, new_connection,
            boost::asio::placeholders::error));
    }
	
	///////////////////////////
	void tcp_server::handle_accept(tcp_connection::pointer new_connection,
      const boost::system::error_code& error)
    {
      if (!error)
      {
        new_connection->start_write();
        new_connection->start_read();
        start_accept();
      }
      else
      {
        std::cerr << "Error in handle acceptance" << std::endl; 
      }
    }

}; // ns

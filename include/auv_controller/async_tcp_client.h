//
// async_tcp_client.h
// ~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2011 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
// 
// 
/*
 * Modified Date: Monday, Martch 1st 2021, 10:01:12 pm
 * Author: zhao wang
 */
 
#ifndef ASYNC_TCP_CLIENT_H_
#define ASYNC_TCP_CLIENT_H_

#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>
#include <boost/bind.hpp>
#include <iostream>

using boost::asio::deadline_timer;
using boost::asio::ip::tcp;

//
// This class manages socket timeouts by applying the concept of a deadline.
// Some asynchronous operations are given deadlines by which they must complete.
// Deadlines are enforced by an "actor" that persists for the lifetime of the
// AsyncTCPClient object:
//
//  +----------------+
//  |                |
//  | check_deadline |<---+
//  |                |    |
//  +----------------+    | async_wait()
//              |         |
//              +---------+
//
// If the deadline actor determines that the deadline has expired, the socket
// is closed and any outstanding operations are consequently cancelled.
//
// Connection establishment involves trying each endpoint in turn until a
// connection is successful, or the available endpoints are exhausted. If the
// deadline actor closes the socket, the connect actor is woken up and moves to
// the next endpoint.
//
//  +---------------+
//  |               |
//  | start_connect |<---+
//  |               |    |
//  +---------------+    |
//           |           |
//  async_-  |    +----------------+
// connect() |    |                |
//           +--->| handle_connect |
//                |                |
//                +----------------+
//                          :
// Once a connection is     :
// made, the connect        :
// actor forks in two -     :
//                          :
// an actor for reading     :       and an actor for
// inbound messages:        :       sending heartbeats:
//                          :
//  +------------+          :          +-------------+
//  |            |<- - - - -+- - - - ->|             |
//  | start_read |                     | start_write |<---+
//  |            |<---+                |             |    |
//  +------------+    |                +-------------+    | async_wait()
//          |         |                        |          |
//  async_- |    +-------------+       async_- |    +--------------+
//   read_- |    |             |       write() |    |              |
//  until() +--->| handle_read |               +--->| handle_write |
//               |             |                    |              |
//               +-------------+                    +--------------+
//
// The input actor reads messages from the socket, where messages are delimited
// by the newline character. The deadline for a complete message is 30 seconds.
//
// The heartbeat actor sends a heartbeat (a message that consists of a single
// newline character) every 10 seconds. In this example, no deadline is applied
// message sending.
//

namespace tcp_client{

class AsyncTCPClient
{
public:
  /**
   * @brief TCP asynchronous client class constructor
   * @param io_service boost asio lib io service
   */
  AsyncTCPClient(boost::asio::io_service& io_service);

  /**
   * @brief Called by the user of the AsyncTCPClient class to initiate the connection process.
   * The endpoint iterator will have been obtained using a tcp::resolver.
   * @param endpoint_iter TCP resolver iterator 
   */
  void start(tcp::resolver::iterator endpoint_iter);
  
  /**
   * @brief This function terminates all the actors to shut down the connection. It
   * may be called by the user of the AsyncTCPClient class, or by the class itself in
   * response to graceful termination or an unrecoverable error.
   */
  void stop();

private:
  /**
   * @brief This function start the connetion with TCP server which holds the target ip
   * address.
   * @param endpoint_iter TCP resolver interator, which is used to check existed connection 
   * requests.
   */
  void start_connect(tcp::resolver::iterator endpoint_iter);

  /**
   * @brief This function start the connetion with TCP server which holds the target ip
   * address.
   * @param ec Catched error code from client connection.
   * @param endpoint_iter TCP resolver interator.
   */
  void handle_connect(const boost::system::error_code& ec,
      tcp::resolver::iterator endpoint_iter);

  /**
   * @brief This function invokes the buffer read callback function and reads the bytes
   * from buffer.
   */
  void start_read();

  /**
   * @brief This function is used to handle read request and catch the error.
   * @param ec Catched error code from client connection.
   */
  void handle_read(const boost::system::error_code& ec);

  /**
   * @brief This function invokes the buffer write callback function and writes the bytes
   * into buffer
   */
  void start_write();

  /**
   * @brief This function is used to handle write request and catch the error.
   * @param ec Catched error code from client connection.
   */
  void handle_write(const boost::system::error_code& ec);

  /**
   * @brief This function is used to check the connection deadline.
   */
  void check_deadline();

private:
  bool stopped_;
  tcp::socket socket_;
  boost::asio::streambuf input_buffer_;
  deadline_timer deadline_;
  deadline_timer heartbeat_timer_;
  
}; // AsyncTCPClient
}; // ns

#endif

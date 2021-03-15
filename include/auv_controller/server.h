//
// server.h
// ~~~~~~~~~~
//
// Copyright (c) 2003-2011 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <algorithm>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <set>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>

using boost::asio::deadline_timer;
using boost::asio::ip::tcp;
using boost::asio::ip::udp;

namespace tcp_server
{
    //----------------------------------------------------------------------
    class subscriber
    {
    public:
      virtual ~subscriber() {}
      virtual void deliver(const std::string& msg) = 0;
    };
    
    typedef boost::shared_ptr<subscriber> subscriber_ptr;
    
    //----------------------------------------------------------------------
    class channel
    {
    public:
      void join(subscriber_ptr subscriber);
    
      void leave(subscriber_ptr subscriber);
      
      void deliver(const std::string& msg);
    
    private:
      std::set<subscriber_ptr> subscribers_;
    }; // class subscriber
    
    //----------------------------------------------------------------------
    //
    // This class manages socket timeouts by applying the concept of a deadline.
    // Some asynchronous operations are given deadlines by which they must complete.
    // Deadlines are enforced by two "actors" that persist for the lifetime of the
    // session object, one for input and one for output:
    //
    //  +----------------+                     +----------------+
    //  |                |                     |                |
    //  | check_deadline |<---+                | check_deadline |<---+
    //  |                |    | async_wait()   |                |    | async_wait()
    //  +----------------+    |  on input      +----------------+    |  on output
    //              |         |  deadline                  |         |  deadline
    //              +---------+                            +---------+
    //
    // If either deadline actor determines that the corresponding deadline has
    // expired, the socket is closed and any outstanding operations are cancelled.
    //
    // The input actor reads messages from the socket, where messages are delimited
    // by the newline character:
    //
    //  +------------+
    //  |            |
    //  | start_read |<---+
    //  |            |    |
    //  +------------+    |
    //          |         |
    //  async_- |    +-------------+
    //   read_- |    |             |
    //  until() +--->| handle_read |
    //               |             |
    //               +-------------+
    //
    // The deadline for receiving a complete message is 30 seconds. If a non-empty
    // message is received, it is delivered to all subscribers. If a heartbeat (a
    // message that consists of a single newline character) is received, a heartbeat
    // is enqueued for the client, provided there are no other messages waiting to
    // be sent.
    //
    // The output actor is responsible for sending messages to the client:
    //
    //  +--------------+
    //  |              |<---------------------+
    //  | await_output |                      |
    //  |              |<---+                 |
    //  +--------------+    |                 |
    //      |      |        | async_wait()    |
    //      |      +--------+                 |
    //      V                                 |
    //  +-------------+               +--------------+
    //  |             | async_write() |              |
    //  | start_write |-------------->| handle_write |
    //  |             |               |              |
    //  +-------------+               +--------------+
    //
    // The output actor first waits for an output message to be enqueued. It does
    // this by using a deadline_timer as an asynchronous condition variable. The
    // deadline_timer will be signalled whenever the output queue is non-empty.
    //
    // Once a message is available, it is sent to the client. The deadline for
    // sending a complete message is 30 seconds. After the message is successfully
    // sent, the output actor again waits for the output queue to become non-empty.
    //
    class tcp_session
      : public subscriber,
        public boost::enable_shared_from_this<tcp_session>
    {
    public:
      tcp_session(boost::asio::io_service& io_service, channel& ch);
    
      tcp::socket& socket();
    
      // Called by the server object to initiate the four actors.
      void start();
    
    private:
      void stop();
    
      bool stopped() const;
    
      void deliver(const std::string& msg);
    
      void start_read();
    
      void handle_read(const boost::system::error_code& ec);
    
      void await_output();
    
      void start_write();
    
      void handle_write(const boost::system::error_code& ec);
    
      void check_deadline(deadline_timer* deadline);
    
      channel& channel_;
      tcp::socket socket_;
      boost::asio::streambuf input_buffer_;
      deadline_timer input_deadline_;
      std::deque<std::string> output_queue_;
      deadline_timer non_empty_output_queue_;
      deadline_timer output_deadline_;
    }; // class tcp_session
    
    typedef boost::shared_ptr<tcp_session> tcp_session_ptr;
    
    //----------------------------------------------------------------------
    class udp_broadcaster
      : public subscriber
    {
    public:
      udp_broadcaster(boost::asio::io_service& io_service,
          const udp::endpoint& broadcast_endpoint);
    
    private:
      void deliver(const std::string& msg);
    
      udp::socket socket_;
    }; // class udp_broadcaster
    
    //----------------------------------------------------------------------
    class server
    {
    public:
      server(boost::asio::io_service& io_service,
          const tcp::endpoint& listen_endpoint,
          const udp::endpoint& broadcast_endpoint);
    
      void handle_accept(tcp_session_ptr session,
          const boost::system::error_code& ec);
    
    private:
      boost::asio::io_service& io_service_;
      tcp::acceptor acceptor_;
      channel channel_;
    }; // class server
	
}; //ns
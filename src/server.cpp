//
// server.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2011 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include "auv_controller/server.h"

namespace tcp_server
{
	/* Channel interface */
	/////////////////////////////////
    void channel::join(subscriber_ptr subscriber)
    {
      subscribers_.insert(subscriber);
    }

	/////////////////////////////////
	void channel::leave(subscriber_ptr subscriber)
    {
      subscribers_.erase(subscriber);
    }

	/////////////////////////////////
	void channel::deliver(const std::string& msg)
    {
      std::for_each(subscribers_.begin(), subscribers_.end(),
          boost::bind(&subscriber::deliver, _1, boost::ref(msg)));
    }
	
	/* tcp_session interface */
	/////////////////////////////////
    tcp_session::tcp_session(boost::asio::io_service& io_service, channel& ch)
      : channel_(ch),
        socket_(io_service),
        input_deadline_(io_service),
        non_empty_output_queue_(io_service),
        output_deadline_(io_service)
    {
      input_deadline_.expires_at(boost::posix_time::pos_infin);
      output_deadline_.expires_at(boost::posix_time::pos_infin);
    
      // The non_empty_output_queue_ deadline_timer is set to pos_infin whenever
      // the output queue is empty. This ensures that the output actor stays
      // asleep until a message is put into the queue.
      non_empty_output_queue_.expires_at(boost::posix_time::pos_infin);
    }

	/////////////////////////////////
    tcp::socket& tcp_session::socket()
    {
      return socket_;
    }
	
	/////////////////////////////////
    void tcp_session::start()
    {
      channel_.join(shared_from_this());
    
      start_read();
    
      input_deadline_.async_wait(
          boost::bind(&tcp_session::check_deadline,
          shared_from_this(), &input_deadline_));
    
      await_output();
    
      output_deadline_.async_wait(
          boost::bind(&tcp_session::check_deadline,
          shared_from_this(), &output_deadline_));
    }
	
	/////////////////////////////////
    void tcp_session::stop()
    {
      channel_.leave(shared_from_this());
    
      socket_.close();
      input_deadline_.cancel();
      non_empty_output_queue_.cancel();
      output_deadline_.cancel();
    }
	
	/////////////////////////////////
    bool tcp_session::stopped() const
    {
      return !socket_.is_open();
    }
	
	/////////////////////////////////
	void tcp_session::deliver(const std::string& msg)
    {
      output_queue_.push_back(msg + "\n");
    
      // Signal that the output queue contains messages. Modifying the expiry
      // will wake the output actor, if it is waiting on the timer.
      non_empty_output_queue_.expires_at(boost::posix_time::neg_infin);
    }
	
	/////////////////////////////////
	void tcp_session::start_read()
    {
      // Set a deadline for the read operation.
      input_deadline_.expires_from_now(boost::posix_time::seconds(30));
    
      // Start an asynchronous operation to read a newline-delimited message.
      boost::asio::async_read_until(socket_, input_buffer_, '\n',
          boost::bind(&tcp_session::handle_read, shared_from_this(), _1));
    }
	
	/////////////////////////////////
	void tcp_session::handle_read(const boost::system::error_code& ec)
    {
      if (stopped())
        return;
    
      if (!ec)
      {
        // Extract the newline-delimited message from the buffer.
        std::string msg;
        std::istream is(&input_buffer_);
        std::getline(is, msg);
    
        if (!msg.empty())
        {
          channel_.deliver(msg);
        }
        else
        {
          // We received a heartbeat message from the client. If there's nothing
          // else being sent or ready to be sent, send a heartbeat right back.
          if (output_queue_.empty())
          {
            output_queue_.push_back("\n");
    
            // Signal that the output queue contains messages. Modifying the
            // expiry will wake the output actor, if it is waiting on the timer.
            non_empty_output_queue_.expires_at(boost::posix_time::neg_infin);
          }
        }
    
        start_read();
      }
      else
      {
        stop();
      }
    }

	/////////////////////////////////
	void tcp_session::await_output()
    {
      if (stopped())
        return;
    
      if (output_queue_.empty())
      {
        // There are no messages that are ready to be sent. The actor goes to
        // sleep by waiting on the non_empty_output_queue_ timer. When a new
        // message is added, the timer will be modified and the actor will wake.
        non_empty_output_queue_.expires_at(boost::posix_time::pos_infin);
        non_empty_output_queue_.async_wait(
            boost::bind(&tcp_session::await_output, shared_from_this()));
      }
      else
      {
        start_write();
      }
    }

	/////////////////////////////////
	void tcp_session::start_write()
    {
      // Set a deadline for the write operation.
      output_deadline_.expires_from_now(boost::posix_time::seconds(30));
    
      // Start an asynchronous operation to send a message.
      boost::asio::async_write(socket_,
          boost::asio::buffer(output_queue_.front()),
          boost::bind(&tcp_session::handle_write, shared_from_this(), _1));
    }

	/////////////////////////////////
	void tcp_session::handle_write(const boost::system::error_code& ec)
    {
      if (stopped())
        return;
    
      if (!ec)
      {
        output_queue_.pop_front();
    
        await_output();
      }
      else
      {
        stop();
      }
    }

	/////////////////////////////////
	void tcp_session::check_deadline(deadline_timer* deadline)
    {
      if (stopped())
        return;
    
      // Check whether the deadline has passed. We compare the deadline against
      // the current time since a new asynchronous operation may have moved the
      // deadline before this actor had a chance to run.
      if (deadline->expires_at() <= deadline_timer::traits_type::now())
      {
        // The deadline has passed. Stop the session. The other actors will
        // terminate as soon as possible.
        stop();
      }
      else
      {
        // Put the actor back to sleep.
        deadline->async_wait(
            boost::bind(&tcp_session::check_deadline,
            shared_from_this(), deadline));
      }
    }
	
	/* udp_broadcaster interface */
	/////////////////////////////////
	udp_broadcaster::udp_broadcaster(boost::asio::io_service& io_service,
      const udp::endpoint& broadcast_endpoint)
    : socket_(io_service)
    {
      socket_.connect(broadcast_endpoint);
    }
	
	/////////////////////////////////
	void udp_broadcaster::deliver(const std::string& msg)
    {
      boost::system::error_code ignored_ec;
      socket_.send(boost::asio::buffer(msg), 0, ignored_ec);
    }
	
	/* server interface */
	/////////////////////////////////
	 server::server(boost::asio::io_service& io_service,
      const tcp::endpoint& listen_endpoint,
      const udp::endpoint& broadcast_endpoint)
    : io_service_(io_service),
      acceptor_(io_service, listen_endpoint)
    {
      subscriber_ptr bc(new udp_broadcaster(io_service_, broadcast_endpoint));
      channel_.join(bc);
    
      tcp_session_ptr new_session(new tcp_session(io_service_, channel_));
    
      acceptor_.async_accept(new_session->socket(),
          boost::bind(&server::handle_accept, this, new_session, _1));
    }
	
	/////////////////////////////////
	void server::handle_accept(tcp_session_ptr session,
      const boost::system::error_code& ec)
    {
      if (!ec)
      {
        session->start();
    
        tcp_session_ptr new_session(new tcp_session(io_service_, channel_));
    
        acceptor_.async_accept(new_session->socket(),
            boost::bind(&server::handle_accept, this, new_session, _1));
      }
    }
	
}; // ns

#ifndef UDP_CONNECTION_CPP
#define UDP_CONNECTION_CPP

#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include "message_queue.hpp"
#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <udp_connection.hpp>
//#include "config.hpp"
#include "spdlog/fmt/fmt.h"

using boost::asio::ip::udp;
using namespace std;


UDPConnection::UDPConnection( boost::asio::io_service &io_service,
 MessageQueue<std::string>* rxq_server, MessageQueue<std::string>* txq_server,
 std::string remote_address, unsigned short port_out, unsigned short port_in, std::string on_connect_message) :
      own_ep_(udp::endpoint(udp::v4(), port_out)),
      dflt_listen_ep_(udp::endpoint(udp::v4(), port_in)),
      remote_ep_(boost::asio::ip::address_v4::from_string(remote_address), port_in),
      socket_(io_service, dflt_listen_ep_), connection_status_(false),
      port_(port_in), txq(txq_server), rxq(rxq_server), on_connect_message(on_connect_message)
{
  logger_ = spdlog::get("butler_control_interface")->clone("udp_connection");

  try
  {
    start_receive();
    connection_status_ = true;
  }

  catch(std::exception &e)
  {
    logger_->debug("Exception:{}",e.what());
  }

  send_message(on_connect_message);
}


UDPConnection::~UDPConnection()
{
  connection_status_ = false;
  try
  {
    socket_.close();
  }
  catch(std::exception &e)
  {
    logger_->debug("Exception:{}",e.what());
  }
}

bool UDPConnection::pred_connection_status(void)
{
  return connection_status_;
}

void UDPConnection::send(std::string msg, udp::endpoint ep)
{
  using namespace std::chrono_literals;
  std::string addr_str;
  std::unique_lock<std::mutex> mtx(conn_mtx_);
  bool conn_status = conn_cv_.wait_for(mtx,100ms,[this]{return pred_connection_status();});
//  boost::asio::address addr = ep.address();
  logger_->debug("Sending msg to udp ip: {}, port: {}, msg: {}", addr_str, ep.port(), msg);
  std::cout <<"UDP Send adrress" << ep.address() <<std::endl;

  if (conn_status)
  {
    socket_.async_send_to(boost::asio::buffer(msg, msg.length()), ep,
                          boost::bind(
                              &UDPConnection::handle_send, this,
                              msg,
                              boost::asio::placeholders::error,
                              boost::asio::placeholders::bytes_transferred));
  }
}

void UDPConnection::send_message(const std::string &msg)
{

  send_to(msg, remote_ep_);
}

void UDPConnection::send_to(std::string msg,udp::endpoint to)
{
   try
  {
    send(msg, to);
  }
  catch(std::exception &e)
  {
    logger_->debug("Exception:{}",e.what());
  }
  
}

void UDPConnection::start() {

  std::cout <<"UDP Object created" <<std::endl;
  start_receive();
}

void UDPConnection::start_receive()
{
  socket_.async_receive_from(
      boost::asio::buffer(recv_buffer_), remote_ep_,
      boost::bind(&UDPConnection::handle_receive, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

void UDPConnection::handle_receive(const boost::system::error_code &error, std::size_t bytes_transferred)
{
  if (!error || error == boost::asio::error::message_size)
  {
    std::string message(reinterpret_cast<char const *>(recv_buffer_), bytes_transferred);
//    from_remote_q.push(message);
    rxq->push(message);
    logger_->debug("Received message bytes : {}",bytes_transferred);
    start_receive();
  }
}

void UDPConnection::retreive_message(std::string* msg) {
  rxq->wait();
  *msg = rxq->pop();
}
void UDPConnection::handle_send(std::string message,
                  const boost::system::error_code & error,
                  std::size_t bytes_transferred)
{

}
#endif /* #ifndef UDP_CONNECTION_CPP */
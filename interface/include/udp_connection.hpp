#ifndef UDP_CONNECTION_HPP
#define UDP_CONNECTION_HPP

#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <connection.hpp>
#include "message_queue.hpp"
#include "spdlog/spdlog.h"
//#include <pluginlib/class_list_macros.hpp>
#include "json/json.h"

using boost::asio::ip::udp;
using namespace std;

class UDPConnection: public Connection
{
 public:
  MessageQueue<std::string> from_remote_q;
  //  Reference of UDP Server queue
  MessageQueue<std::string>* rxq;
  MessageQueue<std::string>* txq;
  
  UDPConnection( boost::asio::io_service &io_service,
                                MessageQueue<std::string>* rxq_server, MessageQueue<std::string>* txq_server,
                                std::string remote_address, unsigned short port_out, unsigned short port_in, std::string on_connect_message);
                                
  //UDPConnection(){}
  ~UDPConnection();

  void send(std::string msg, udp::endpoint ep);
  void send_to(std::string msg,udp::endpoint to);
  void start() override;
  void send_message(const std::string &msg) override;
  void retreive_message(std::string* msg) override;

  bool get_conn_status() override {
    return connection_status_;
  }

  typedef std::shared_ptr<UDPConnection> UdpConnectionSPtrType;
  bool connection_status_;

 private:

  std::string on_connect_message;
  std::mutex conn_mtx_;
  std::condition_variable conn_cv_;
  udp::endpoint own_ep_;
  uint32_t port_;
  udp::endpoint dflt_listen_ep_;
  udp::endpoint remote_ep_;
  uint8_t recv_buffer_[1024];
  std::shared_ptr<spdlog::logger> logger_;
  udp::socket socket_;

  void start_receive();

  void handle_receive(const boost::system::error_code &error, std::size_t bytes_transferred);

  void handle_send(std::string /*message*/,
                   const boost::system::error_code & /*error*/,
                   std::size_t /*bytes_transferred*/);
  
  bool pred_connection_status();
};
//PLUGINLIB_EXPORT_CLASS(UDPConnection, Connection);

#endif /* #ifndef UDP_CONNECTION_CPP */
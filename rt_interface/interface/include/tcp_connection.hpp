/**
 * \file tcp_connection.hpp
 * @author Inderjeet Yadav (inderjeet.y@greyorange.sg)
 * \brief
 * @version 0.1
 * \date 15-07-2019
 *
 * @copyright Copyright (c) 2019 Inderjeet Yadav
 * GreyOrange India Pte Ltd
 *
 */

#ifndef TCP_CONNECTION_H
#define TCP_CONNECTION_H

#include <iostream>
#include <string>
#include <thread>

#include <connection.hpp>
#include <spdlog/fmt/bundled/printf.h>
#include <spdlog/spdlog.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include "message_queue.hpp"
//#include <pluginlib/class_list_macros.hpp>

#define TCP_BUFFER_SIZE 4096

using boost::asio::ip::tcp;

class TcpConnection : public Connection {

 public:
  TcpConnection(boost::asio::io_service& io_service, uint32_t port,
                MessageQueue<std::string>* rxq_server, MessageQueue<std::string>* txq_server,std::string on_connect_message);
  ~TcpConnection();

 private:
  std::string on_connect_message;

  std::mutex rx_mutexLock;
  std::mutex tx_mutexLock;
  char receive_buffer[TCP_BUFFER_SIZE];
  std::condition_variable conn_status_cv_;
  std::string received_msg_buffer;

  bool send_thread_handler();

 public:
  bool connection_status_;
  static std::condition_variable dataCondition;
  tcp::socket socket_;
  void start();
  bool get_conn_status();
  void retreive_message(std::string* msg);
  void send_message(const std::string &msg);

  tcp::socket& socket();

  std::shared_ptr<spdlog::logger> logger_;

  typedef std::shared_ptr<TcpConnection> TcpConnectionSPtrType;
  typedef std::unique_ptr<TcpConnection> TcpConnectionUPtrType;

 private:
  std::string name_;
//  Private queue for connection class
  // MessageQueue<std::string> _rxq;
  // MessageQueue<std::string> _txq;

  //  Reference of TCP Server queue
  MessageQueue<std::string>* rxq;
  MessageQueue<std::string>* txq;
  unsigned short port_;

  std::thread send_message_thread_;
  void start_write();
  void start_read();
  void close_socket();
  void handle_write(const boost::system::error_code& error,
                    size_t bytes_transferred);
  void handle_read(const boost::system::error_code& error,
                   size_t bytes_transferred);
  void wait_on_queue(MessageQueue<std::string>* queue);
  bool predicate_send();
};

//PLUGINLIB_EXPORT_CLASS(TcpConnection, Connection);

#endif /* TCP_CONNECTION_H */
/**
 * \file tcp_server.hpp
 * @author Inderjeet Yadav (inderjeet.y@greyorange.sg)
 * \brief
 * @version 0.1
 * \date 11-07-2019
 *
 * @copyright Copyright (c) 2019 Inderjeet Yadav
 * GreyOrange India Pte Ltd
 *
 */
#include <ctime>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include "tcp_connection.hpp"

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include "json/json.h"
#include "message_queue.hpp"

#define TCP_BUFFER_SIZE 4096

class TcpServer {
 public:
  int clients_total;
  unsigned short port;
  std::string iface_name;
  std::string on_connect_message;
  // Constructor: initializes an acceptor to listen on TCP port.
  TcpServer(boost::asio::io_service& io_service, unsigned short port,
            std::string name,std::string on_connect_message);
  ~TcpServer();

  MessageQueue<std::string> rxq;
  MessageQueue<std::string> txq;

  typedef std::unordered_map<std::string, TcpConnection::TcpConnectionSPtrType> TcpConnectionMapType;
  typedef std::unique_ptr<TcpServer> TcpServerUPtrType;

  boost::asio::io_service& io_service_;
  boost::asio::io_service::work work;
  tcp::endpoint ep;
  // boost::asio::io_service::work work_;
  tcp::acceptor acceptor_;
  std::shared_ptr<spdlog::logger> logger_;
  TcpConnectionMapType get_tcp_connection_map();
  TcpConnectionMapType tcp_connection_map_;
  TcpConnection::TcpConnectionSPtrType ConnectedClient;

 private:
//  TcpConnection::TcpConnectionSPtrType _default;
  TcpConnection::TcpConnectionSPtrType _Listener;
  TcpConnection::TcpConnectionSPtrType _default;
  std::thread io_service_thread;
  Json::Value tcp_connection_list_;
  std::vector<TcpConnection::TcpConnectionSPtrType> tcp_connection_vec_;

  void start_accept();
  void handle_accept(const boost::system::error_code& error);
  // void io_service_run();
};

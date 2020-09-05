/**
 * \file tcp_server.cpp
 * @author Inderjeet Yadav (inderjeet.y@greyorange.sg)
 * \brief
 * @version 0.1
 * \date 16-07-2019
 *
 * @copyright Copyright (c) 2019 Inderjeet Yadav
 * GreyOrange India Pte Ltd
 *
 */
#include "tcp_server.hpp"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <ctime>
#include <iostream>
#include <istream>
#include <string>

/**
 * \brief Construct a new Tcp Server:: Tcp Server object
 *
 */
TcpServer::TcpServer(boost::asio::io_service& io_service, unsigned short port,
                     std::string name, std::string on_connect_message)
    : iface_name(name),
      clients_total(0),
      port(port),
      io_service_(io_service),
      ep(boost::asio::ip::address_v4::any(), port),
      work(io_service),
      acceptor_(io_service, ep),
      on_connect_message(on_connect_message) {


  logger_ = spdlog::get("butler_control_interface")->clone("tcp_server");
    /*
  NOTE:
    The follwing is used as a static connection for the interface to manipulate queues.
    Application --> Server(push(txq) or pull(rxq)) --> the 'client', since the txq & rxq
    are common between all clients, so Application doesn't actually need to know "which
    client" or is it actually connected or not.
    Whereas clients will come and go, but since all tx & rx is in a 'q', it is persistent
    and does not need to keep a track of clients in general for simplicity.
    Of course, this implementation is rather simplistic, and will need to change later.
  */
  _default = std::make_shared<TcpConnection>(io_service_, port, &this->txq, &this->rxq,on_connect_message);
  _default->connection_status_ = true;
  tcp_connection_map_.emplace(iface_name, _default);
  // tcp_connection_map_.emplace(iface_name, _Listener);
  start_accept();
}

TcpServer::~TcpServer() {}

void TcpServer::start_accept() {
  _Listener =
      std::make_shared<TcpConnection>(io_service_, port, &this->txq, &this->rxq,on_connect_message);
  auto fn_accept_hrl = boost::bind(&TcpServer::handle_accept, this,
                                   boost::asio::placeholders::error);
  acceptor_.async_accept(_Listener->socket(), fn_accept_hrl);
}

void TcpServer::handle_accept(const boost::system::error_code& error) {
  logger_->debug("Connection Accept Handled:{}", error);
  if (!error) {
    ConnectedClient = _Listener;
    ConnectedClient->start();
    tcp_connection_map_[iface_name] = ConnectedClient; //See Note in Constructor
  }
  
  start_accept();
}

TcpServer::TcpConnectionMapType TcpServer::get_tcp_connection_map() {
  return tcp_connection_map_;
}
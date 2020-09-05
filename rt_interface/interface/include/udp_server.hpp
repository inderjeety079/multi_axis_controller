//
// Created by inderjeet on 3/24/20.
//

#ifndef UDP_SERVER_HPP_
#define UDP_SERVER_HPP_

#include <udp_connection.hpp>

class UDPServer {

 private:
  int clients_total;
  unsigned short port;
  std::string iface_name;
  std::string on_connect_message;
  std::shared_ptr<spdlog::logger> logger_;
  // Constructor: initializes an acceptor to listen on UDP port.

 public:
  UDPServer(boost::asio::io_service &io_service, unsigned short port, std::string name, std::string remote_address, std::string on_connect_message);
  ~UDPServer();

  MessageQueue<std::string> rxq;
  MessageQueue<std::string> txq;

  typedef std::unordered_map<std::string, UDPConnection::UdpConnectionSPtrType> UdpConnectionMapType;

  typedef std::unique_ptr<UDPServer> UdpServerUPtrType;

  boost::asio::io_service& io_service_;
  boost::asio::io_service::work work;
  udp::endpoint ep;
  udp::endpoint remote_ep;

  UdpConnectionMapType get_udp_connection_map();
  UdpConnectionMapType udp_connection_map_;
  UDPConnection::UdpConnectionSPtrType ConnectedClient;
  UDPConnection::UdpConnectionSPtrType _Listener;
  UDPConnection::UdpConnectionSPtrType _default;

 public:

};

#endif // UDP_SERVER_HPP_

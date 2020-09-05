//
// Created by inderjeet on 17/08/19.
//

#ifndef CONNECTION_MANAGER_HPP_
#define CONNECTION_MANAGER_HPP_

#include <boost/thread.hpp>
#include <tcp_server.hpp>
#include <udp_server.hpp>

class ConnectionManager {
 public:
  explicit ConnectionManager(std::string on_connect_message);
  ~ConnectionManager();
  void subscribe();
  boost::asio::io_service io_service;
  typedef std::unique_ptr<ConnectionManager> ConnectionManagerUPtrType;
  typedef std::shared_ptr<ConnectionManager> ConnectionManagerSPtrType;
  typedef std::unordered_map<std::string, std::unique_ptr<std::thread>> ThreadsUPtrMapType;
  typedef std::unordered_map<std::string, std::shared_ptr<TcpServer>> TcpInterfacesSPtrMapType;
  typedef std::unordered_map<std::string, std::shared_ptr<UDPServer>> UdpInterfacesSPtrMapType;
  typedef std::unordered_map<std::string, Connection::ConnectionSPtrType> NavInterfacesUPtrMapType;
  typedef std::unordered_map<std::string, Connection::ConnectionSPtrType> ConnectionsSPtrMapType;

  Connection::ConnectionSPtrType get_connection_handle(const std::string& iface_name);
  Connection::ConnectionSPtrType get_connection(const std::string& iface_name);

  bool load_connection_map();
  std::shared_ptr<spdlog::logger> logger_;
  //    void register_subscription(const std::string &iface, )
  bool load_connections(const Json::Value& connection_list);

  void spawn_tcp_server(std::string iface_name, unsigned short port);
  void spawn_udp_server(std::string iface_name, unsigned short port_in, unsigned short port_out, std::string remote_address);
  void spawn_serial(std::string iface_name, std::string port);

 private:
  boost::asio::io_service::work work_;
  Json::Value connection_list_;
  ConnectionsSPtrMapType connections_map_;
  UdpInterfacesSPtrMapType udp_interfaces_map_;
  TcpInterfacesSPtrMapType tcp_interfaces_map_;

  //   TcpServer::TcpConnectionMapType connection_map_;
  void run_io_service();
  void start_io_service();
  TcpServer::TcpServerUPtrType tcp_server_ptr_;
  std::thread io_service_thread_;
  std::string on_connect_message_;

  ConnectionManager::ThreadsUPtrMapType subscriber_threads_;
  ConnectionManager::ThreadsUPtrMapType publish_threads_;
};

#endif  // CONNECTION_MANAGER_HPP_

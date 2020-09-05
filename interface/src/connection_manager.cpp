//
// Created by inderjeet on 17/08/19.
//

#include <connection_manager.hpp>


void ConnectionManager::run_io_service()
{
  std::cout << "Starting IO Service from ConnectionManager::run_io_service" << std::endl;
  // while(true)
  {
    auto rv = io_service.run();
    /* Should not come here */
    std::cout << "IO Service from ConnectionManager::run_io_service Stopped with ec:" << rv << std::endl;
    std::cout << "Resrart io_service :: io_service.run()" << std::endl;
  }
}

ConnectionManager::ConnectionManager(std::string on_connect_message):
  work_(io_service),
  on_connect_message_(on_connect_message)
{
  logger_ = spdlog::get("butler_control_interface")->clone("connection_manager");
}

ConnectionManager::~ConnectionManager()
{
  io_service_thread_.join();
}


Connection::ConnectionSPtrType ConnectionManager::get_connection_handle(const std::string& iface_name) {

    auto udp_interface = udp_interfaces_map_.find(iface_name);

    if (udp_interface != udp_interfaces_map_.end()) {
      auto udp_server = udp_interface->second;
      Connection::ConnectionSPtrType connection_handle = udp_server->udp_connection_map_[iface_name];
      return connection_handle;
    }

    else {

      auto tcp_interface = tcp_interfaces_map_.find(iface_name);

      if (tcp_interface != tcp_interfaces_map_.end()) {
        auto tcp_server = tcp_interface->second;
        Connection::ConnectionSPtrType connection_handle = tcp_server->tcp_connection_map_[iface_name];
        return connection_handle;
      }

      else {
        return nullptr;
      }
    }

}

bool ConnectionManager::load_connection_map() {
  // connection_map_ = tcp_server_ptr_->get_tcp_connection_map();
}


bool ConnectionManager::load_connections(const Json::Value& connection_list)
{
  connection_list_ = connection_list;
  std::cout << "connections:" << connection_list << std::endl;
  for (auto subsystem_network_config : connection_list)
  {
    if ("tcp" == subsystem_network_config["connection_type"].asString()) {
      spawn_tcp_server(subsystem_network_config["name"].asString(),
                         subsystem_network_config["port"].asUInt());
    }

    else if ("udp" == subsystem_network_config["connection_type"].asString()) {
      spawn_udp_server(subsystem_network_config["name"].asString(),
                         subsystem_network_config["port"].asUInt(), subsystem_network_config["port"].asUInt(), subsystem_network_config["ip"].asString());
    }

    else if ("serial" == subsystem_network_config["connection_type"].asString()) {
      spawn_serial(subsystem_network_config["name"].asString(),
                         subsystem_network_config["port"].asString());
    }


  }
  start_io_service();
  return true;
}

void ConnectionManager::spawn_tcp_server(std::string iface_name,unsigned short port)
{
  auto new_TcpServer = std::make_shared<TcpServer>(io_service,port,iface_name,on_connect_message_);
  tcp_interfaces_map_.emplace(iface_name, new_TcpServer);
  logger_->info("Spawned TCP Server for Interface: {}, Port {}", iface_name, port);
}

void ConnectionManager::spawn_udp_server(std::string iface_name,unsigned short port_in, unsigned short port_out, std::string remote_address)
{
  auto new_UDPConnection = std::make_shared<UDPServer>(io_service, port_in, iface_name, remote_address, on_connect_message_);
  udp_interfaces_map_.emplace(iface_name, new_UDPConnection);
  logger_->info("Spawned UDP Server for Interface: {}, Port {}", iface_name, port_in);

}

void ConnectionManager::spawn_serial(std::string iface_name, std::string port) {

}

void ConnectionManager::start_io_service()
{
  // tcp_server_ptr_ = std::make_unique<TcpServer>();
  // auto function_to_thread_out = std::bind(&ConnectionManager::run_io_service, this);
  // io_service_thread_ = std::thread(function_to_thread_out);
  io_service_thread_ = std::thread(&ConnectionManager::run_io_service,this);
  std::cout <<  "Started IO Service from ConnectionManager::ConnectionManager thread (run_io_service)" << std::endl;
}
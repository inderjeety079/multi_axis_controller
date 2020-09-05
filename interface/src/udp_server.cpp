//
// Created by inderjeet on 3/24/20.
//

#include <udp_server.hpp>

UDPServer::UDPServer(boost::asio::io_service &io_service, unsigned short port, std::string name, std::string remote_address, std::string on_connect_message):
    iface_name(name),
    port(port),
    io_service_(io_service),
    ep(boost::asio::ip::address_v4::any(), port),
    work(io_service),
    on_connect_message(on_connect_message) {

  logger_ = spdlog::get("butler_control_interface")->clone("udp_server");

   std::cout<<"Hello"<<std::endl;
    try
  {
     _default = std::make_shared<UDPConnection>(io_service_, &this->txq, &this->rxq, remote_address, port, port, on_connect_message);
      _default->connection_status_ = true;
  udp_connection_map_.emplace(iface_name, _default);
  }
  catch(std::exception &e)
  {
    //logger_->debug("Exception:{}",e.what());
    std::cout<<"Exception :{}"<<e.what()<<std::endl;
  }
 
 

}

UDPServer::~UDPServer() {}

UDPServer::UdpConnectionMapType UDPServer::get_udp_connection_map() {

}



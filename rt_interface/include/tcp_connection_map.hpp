/**
 * \file tcp_connection_map.hpp
 * @author Inderjeet Yadav (inderjeet.y@greyorange.sg)
 * \brief 
 * @version 0.1
 * \date 11-07-2019
 * 
 * @copyright Copyright (c) 2019 Inderjeet Yadav
 * GreyOrange India Pte Ltd
 * 
 */
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include "tcp_server.hpp"
#include "json/json.h"


class TcpConnectionMap: public TcpServer {
 public:
  TcpConnectionMap();

  ~TcpConnectionMap();
  bool set_tcp_connection_list(const Json::Value value);



  bool load_configuration();
 private:
  Json::Value tcp_connection_list_;
  std::vector<tcp::acceptor> acceptor_vector_;
  

};
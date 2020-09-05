
/**
 * \file interface.hpp
 * @author Inderjeet Yadav (inderjeet.y@greyorange.sg)
 * \brief
 * @version 0.1
 * \date 3/27/20
 *
 * @copyright Copyright (c) 2019 Inderjeet Yadav
 * GreyOrange India Pte Ltd
 *
 */

#ifndef INTERFACE_HPP_
#define INTERFACE_HPP_

#include <memory>
#include <string>

#include "connection.hpp"
#include "connection_manager.hpp"

class Interface {
 public:
  Interface(const std::string config_file_path, const std::string connection_name,
            const std::string on_connect_msg_)
      : config_file_path_(config_file_path), on_connect_msg_(on_connect_msg_) {
    Json::Value connection_list;
    // on_connect_msg_ = "";
    connection_manager_ = std::make_shared<ConnectionManager>(on_connect_msg_);

    config_parser_ptr_ = ConfigParser::get_unique_instance(config_file_path);
    config_parser_ptr_->get_connection_list(connection_list);

    connection_manager_->load_connections(connection_list);

    connection_ = connection_manager_->get_connection_handle(connection_name);
  }

  ~Interface() = default;
  typedef std::shared_ptr<Interface> InterfaceSPtrType;
  typedef std::unique_ptr<Interface> InterfaceUPtrType;


 private:
  std::string config_file_path_;
  std::string on_connect_msg_;
  ConfigParser::ConfigParserPtr config_parser_ptr_;

 protected:
  Connection::ConnectionSPtrType connection_;
  ConnectionManager::ConnectionManagerSPtrType connection_manager_;
};
#endif  // INTERFACE_HPP_

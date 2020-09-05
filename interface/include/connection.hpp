
/**
 * \file connection.hpp
 * @author Inderjeet Yadav (inderjeet.y@greyorange.sg)
 * \brief
 * @version 0.1
 * \date 3/21/20
 *
 * @copyright Copyright (c) 2019 Inderjeet Yadav
 * GreyOrange India Pte Ltd
 *
 */

#ifndef CONNECTION_HPP_
#define CONNECTION_HPP_

#define abstract

#include <json/value.h>

#include <memory>
//#include <>

abstract class Connection {
 private:
  std::string connection_type_;

 public:
  Connection() : state_(CONSTRUCTED) {
  }

  ~Connection() = default;
  enum { CONSTRUCTED, INITIALIZED, RUNNING } state_;
  typedef std::shared_ptr<Connection> ConnectionSPtrType;

  //  virtual bool init_connection(const Json::Value& connection_params);

  abstract virtual void start() = 0;

  abstract virtual bool get_conn_status() = 0;

  //  virtual ConnectionSPtrType get_connection_handle();

  abstract virtual void send_message(const std::string& msg) = 0;

  abstract virtual void retreive_message(std::string* msg) = 0;
};

#endif  // CONNECTION_HPP_

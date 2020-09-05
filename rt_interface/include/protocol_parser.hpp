/**
 * \file protocol_parser.hpp
 * @author Inderjeet Yadav (inderjeet.y@greyorange.sg)
 * \brief 
 * @version 0.1
 * \date 18-07-2019
 * 
 * @copyright Copyright (c) 2019 Inderjeet Yadav
 * GreyOrange India Pte Ltd
 * 
 */

#include <boost/algorithm/string.hpp>
#include <string>
#include <cstring>
#include <utility>
#include <vector>
#include "json/json.h"
#include "actuator_info.hpp"
#include <spdlog/spdlog.h>
//#include <spdlog/fmt/bundled/printf.h>

class ProtocolParser
{
 private:

 public:
  ProtocolParser();
  ~ProtocolParser();
  typedef boost::shared_ptr<ProtocolParser> pointer;

  std::shared_ptr<spdlog::logger> logger_;

  const static std::string header_;
  const static std::string axis_name_delim_;
  const static std::string axis_id_delim_;
  const static std::string key_value_seperator_;
  const static std::string key_value_delim_ ;
  const static std::string message_delim_ ;
  const static std::string message_delim_new_line_;
  const static std::string list_element_seperator_ ;
  const static std::string list_start_char_ ;
  const static std::string list_end_char_ ;

  static bool unpack_key_value_string(std::string *str,
   std::string *axis_name, std::pair<std::string, std::string> *key_value_pair);

  static std::string pack_key_value_string(std::pair<std::string,
   std::string> *key_value_pair);

  static bool unpack_msg(const std::string &str, std::string *axis_name,
   std::vector<std::pair<std::string, std::string>> *key_value_vector);

  static void pack_msg(std::string axis_name,
    std::vector<std::pair<std::string, std::string>> *key_value_vector, std::string *packed_string);

  static void pack_cmd_msg(const std::string &axis_name,
   const std::string &command_type, std::vector<Json::Value>* command, std::string *packed_msg);
  static bool unpack_feedback_msg(const std::string &str, int *axis_id, std::string *axis_name,
                           std::vector<std::pair<std::string, std::string>> *key_value_vector);
  static void add_header(const std::string &interface, std::string *msg, int seq_id);

};

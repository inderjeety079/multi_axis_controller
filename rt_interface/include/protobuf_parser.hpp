// created by indraneel on 28/07/2020

#ifndef PROTOBUF_PARSER_HPP
#define PROTOBUF_PARSER_HPP

#include <string>
#include <cstring>
#include <utility>
#include <vector>
#include <spdlog/spdlog.h>
#include <boost/shared_ptr.hpp>
#include <map>
#include <tuple>
#include <json/json.h>

#include "rt_nrt_comm.pb.h"

#define COMM_RT_NRT_HEADER_1 (0xFFU)
#define COMM_RT_NRT_HEADER_2 (0xFFU)

#define COMM_NRT_RT_HEADER_1 (0xFFU)
#define COMM_NRT_RT_HEADER_2 (0xFEU)

#define COMM_HEADER_LEN 6

// Big Endian unless otherwise specified


class ProtobufParser {

private:
    std::shared_ptr<spdlog::logger> logger_;
    std::shared_ptr<spdlog::logger> logger_fb;

    typedef enum proto_parser_state_e_ {

    HEADER = 1,
    LENGTH = 2,
    MSG_ID = 3,
    PAYLOAD = 4,
    //CHECKSUM = 5,
    NUM_RX_PARSER_STATES
  }proto_parser_state_e_t;

  proto_parser_state_e_t parser_state;

  typedef std::tuple<sub_odom_feed_data,sub_md_io_feed_data> axis_container;

  bool unpack_feedback_msg(main_feedback_message message, \
                    std::vector<std::map<std::string,std::string>> *key_value_map_vector);

  void add_header(uint16_t num_bytes,std::string *msg);

  enum_mtrdrvtype parse_motordriver_type(std::string MD_type_string);

public:
    ProtobufParser();
    ~ProtobufParser();
    typedef boost::shared_ptr<ProtobufParser> pointer;
/** @brief : Populates a vector of map of key value pairs
  *   @note :  Vector is introduced mainly to serve the multiple axes in the main feedback messages 
  *            Size of the vector = number of axes in the feedback message
  */
  bool unpack_msg(const std::string &str, \
        std::vector<std::map<std::string,std::string>> *key_value_map_vector);

  void pack_cmd_msg(const int &axis_id,
   const enum_command_req_msg_type command_type, std::vector<Json::Value>* command,
   std::string *packed_msg); 


};
#endif
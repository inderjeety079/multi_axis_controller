/**
 * \file protocol_parser.cpp
 * @author Inderjeet Yadav (inderjeet.y@greyorange.sg)
 * \brief 
 * @version 0.1
 * \date 18-07-2019
 * 
 * @copyright Copyright (c) 2019 Inderjeet Yadav
 * GreyOrange India Pte Ltd
 * 
 */
#include <string>
#include <iostream>
#include "protocol_parser.hpp"



const std::string ProtocolParser::header_ = "{";
const std::string ProtocolParser::axis_name_delim_ = "&";
const std::string ProtocolParser::axis_id_delim_ = "&";
const std::string ProtocolParser::key_value_seperator_ = "=";
const std::string ProtocolParser::key_value_delim_ = "&";
const std::string ProtocolParser::message_delim_ = "}";
const std::string ProtocolParser::message_delim_new_line_ = "\n";


const std::string ProtocolParser::list_element_seperator_  = ",";
const std::string ProtocolParser::list_start_char_  = "[";
const std::string ProtocolParser::list_end_char_  = "]";




ProtocolParser::ProtocolParser() {
 logger_ = spdlog::get("butler_control_interface")->clone("protocol_parser");
}

ProtocolParser::~ProtocolParser() = default;

bool ProtocolParser::unpack_key_value_string(std::string *str,
  std::string *axis_name, std::pair<std::string, std::string> *key_value_pair) {

  std::string key_value_string = *str;

  std::size_t key_value_sep_pos = key_value_string.find(key_value_seperator_);

  if (key_value_sep_pos != std::string::npos) {
      std::string key = key_value_string.substr(0, key_value_sep_pos);
      std::string value = key_value_string.substr((key_value_sep_pos + 1),
       (key_value_string.length() - key_value_sep_pos -1));
       *key_value_pair = std::make_pair(key, value);
       return true;
  }

  return false;
}

std::string ProtocolParser::pack_key_value_string(
    std::pair<std::string, std::string> *key_value_pair) {

  std::string key = key_value_pair->first;
  std::string value = key_value_pair->second;
  std::string key_value_string;
  key_value_string.append(key);
  key_value_string.append(key_value_seperator_);
  key_value_string.append(value);

  return key_value_string;

}

bool ProtocolParser::unpack_msg(const std::string &str, std::string *axis_name,
 std::vector<std::pair<std::string, std::string>> *key_value_vector) {

  std::size_t key_value_delim_pos = 0;
  std::string message_string = str;
  std::string key_value_string;
  std::string msg_payload;
  std::pair<std::string, std::string> key_value_pair;
  bool unpack_status = false;

  /* Unpack message header params before unpacking key value pairs */

  std::vector<std::string> substrings;
  boost::split(substrings, str, [](char c){ return c == '&';});

  for ( auto substr_itr = substrings.begin(); substr_itr !=
      (substrings.end() - 1); substr_itr++) {
    std::vector<std::string> key_value;
    boost::split(key_value,*substr_itr, [](char c) { return c == '=';});
    std::string key = key_value.front();
    std::string value = key_value.back();
    key_value_vector->push_back(std::make_pair(key,value));
  }

  return true;
}

void ProtocolParser::pack_msg(std::string axis_name,
    std::vector<std::pair<std::string, std::string>> *key_value_vector,
    std::string *packed_string) {


  /* Append key value pairs from the key_value_vector passed to this
   function*/
  //
  for (auto & key_value_itr : *key_value_vector) {

    std::string key_value_string = pack_key_value_string(
    &key_value_itr);
    packed_string->append(key_value_string);
    packed_string->append(key_value_delim_);
  }

  packed_string->append(message_delim_);
  packed_string->append("\n\r");

}

void ProtocolParser::pack_cmd_msg(const std::string &axis_name,
   const std::string &command_type, std::vector<Json::Value>* command,
   std::string *packed_msg) {

  // TODO(Inderjeet): add sequence ID and timestamp
  packed_msg->append("accessory_name=");
  /*Append axis name*/
  packed_msg->append(axis_name);

  /* Append axis name delimeter */
  packed_msg->append(axis_name_delim_);
  packed_msg->append("accessory_controls/jobs");
  packed_msg->append(key_value_seperator_);
  packed_msg->append(command_type);
  packed_msg->append(list_element_seperator_);
  packed_msg->append(list_start_char_);

  if ( "velocity" == command_type) {

    for (auto element : *command) {

      packed_msg->append(list_start_char_);
      packed_msg->append(element["timestamp"].asString());
      packed_msg->append(list_element_seperator_);
      packed_msg->append(element["job_id"].asString());
      packed_msg->append(list_element_seperator_);

      packed_msg->append(element["timeout"].asString());
      packed_msg->append(list_element_seperator_);
      packed_msg->append(element["velocity"].asString());
      packed_msg->append(list_end_char_);
      packed_msg->append(list_element_seperator_);
    }

    packed_msg->pop_back();

  } else if ("position" == command_type) {

    for (auto element : *command) {
      packed_msg->append(list_start_char_);
      packed_msg->append(element["timestamp"].asString());
      packed_msg->append(list_element_seperator_);
      packed_msg->append(element["job_id"].asString());
      packed_msg->append(list_element_seperator_);

      packed_msg->append(element["timeout"].asString());
      packed_msg->append(list_element_seperator_);
      packed_msg->append(element["pos"].asString());
      packed_msg->append(list_element_seperator_);
      packed_msg->append(element["max_vel"].asString());
      packed_msg->append(list_element_seperator_);
      packed_msg->append(element["accel"].asString());
      packed_msg->append(list_element_seperator_);
      packed_msg->append(element["decel"].asString());
      packed_msg->append(list_end_char_);
      packed_msg->append(list_element_seperator_);
    }
    packed_msg->pop_back();
  }

  packed_msg->append(list_end_char_);
  packed_msg->append(message_delim_);
  packed_msg->append("\n\r");
}


bool ProtocolParser::unpack_feedback_msg(const std::string &str, int *axis_id,
  std::string *axis_name, std::vector<std::pair<std::string, std::string>>
  *key_value_vector) {

  std::vector<std::string> substrings;
  boost::split(substrings, str, [](char c){ return c == '&';});

  for ( auto substr_itr = substrings.begin(); substr_itr !=
    (substrings.end() - 1); substr_itr++) {

    std::vector<std::string> key_value;
    boost::split(key_value,*substr_itr, [](char c) { return c == '=';});
    std::string key = key_value.front();
    std::string value = key_value.back();
    key_value_vector->push_back(std::make_pair(key,value));
//    std::cout << key << " = " << value << ", ";
  }

//std::cout << "accessory_id: {" << *axis_id << "} axis_name: {" << axis_name->data()
//    << "} enc_handle: {" << feedback->encoder_handle << "} meas_vel: {" <<
//    feedback->meas_velocity << "} abs_ticks : {" << feedback->abs_ticks << "}"
//    << std::endl;

//  if (data_fields_to_read == data_fields_parsed ) {
//    unpack_status = true;
//  } else {
//    unpack_status = false;
//  }

  return true;
}

void ProtocolParser::add_header(const std::string &interface, std::string *msg, int seq_id) {

  // int seq_id;
  ActuatorInfo::timestamp_t timestamp;
  timestamp.secs = 0;
  timestamp.msecs = 0;

  msg->append(ProtocolParser::header_);
  msg->append("seq_id");
  msg->append(ProtocolParser::key_value_seperator_);
  msg->append(std::to_string(seq_id));
  msg->append(ProtocolParser::key_value_delim_);

  msg->append("ts_seconds");
  msg->append(ProtocolParser::key_value_seperator_);
  msg->append(std::to_string(timestamp.secs));
  msg->append(ProtocolParser::key_value_delim_);

  msg->append("ts_msecs");
  msg->append(ProtocolParser::key_value_seperator_);
  msg->append(std::to_string(timestamp.msecs));
  msg->append(ProtocolParser::key_value_delim_);
}
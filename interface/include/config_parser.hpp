/**
 * \file config_parser.hpp
 * @author Saket Gupta (saket.g@greyorange.com)
 * \brief
 * @version 0.1
 * \date 4/17/20
 *
 * @copyright Copyright (c) 2019 Saket Gupta
 * GreyOrange India Pte Ltd
 *
 */

#pragma once

#include <json/json.h>
#include <spdlog/spdlog.h>

#include <fstream>
#include <memory>

class ConfigParser {
 public:
  ~ConfigParser() = default;
  ConfigParser(const ConfigParser&) = delete;
  void operator=(const ConfigParser&) = delete;
  ConfigParser(ConfigParser&&) noexcept = default;
  ConfigParser& operator=(ConfigParser&&) noexcept = default;

  typedef ConfigParser* ConfigParserPtr;

  void load_configuration();
  bool parse_json_config();
  bool get_config_element(const std::string& base_element, Json::Value& config);
  bool get_connection_list(Json::Value& connection_config);
  bool get_accessories_list(Json::Value& accessories_config);

  static ConfigParser::ConfigParserPtr get_unique_instance(const std::string& config_file_path_);

  std::shared_ptr<spdlog::logger> logger_;
  static ConfigParser::ConfigParserPtr config_parser_ptr_;

 private:
  explicit ConfigParser(const std::string& config_file_path_);

  static bool is_initialized_;
  //static ConfigParser::ConfigParserPtr config_parser_ptr_;

  std::string config_file_path_;
  Json::Value butler_config;
};

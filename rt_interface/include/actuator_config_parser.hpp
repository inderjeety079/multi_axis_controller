//
// Created by void on 17/04/20.
//

#ifndef SRC_ACTUATOR_CONFIG_PARSER_HPP
#define SRC_ACTUATOR_CONFIG_PARSER_HPP

#include <config_parser.hpp>
#include <string>

#include "actuator.hpp"

class ActuatorConfigParser {
 public:
  explicit ActuatorConfigParser(std::string filepath);
  typedef std::unordered_map<std::string, std::shared_ptr<Actuator>> actuator_map_t;
  typedef std::shared_ptr<actuator_map_t> actuator_map_ptr_t;
  typedef ActuatorConfigParser* ActuatorConfigParserPtr;

  static ActuatorConfigParser::ActuatorConfigParserPtr actuator_config_parser_ptr_;
  actuator_map_ptr_t get_actuator_map_ptr();
  actuator_map_t get_actuator_map();
  bool load_actuator_params(std::shared_ptr<Actuator> actuator, Json::Value params_config);
  bool load_actuator_controls(std::shared_ptr<Actuator> actuator, Json::Value controls_config);
  static ActuatorConfigParser::ActuatorConfigParserPtr get_unique_instance(const std::string& config_file_path_);
  static bool is_initialized_;

  std::vector<std::shared_ptr<Actuator>> get_actuators() {
    return actuators_;
  }
  ConfigParser::ConfigParserPtr config_parser_;

 private:

  std::string config_path_;
  std::shared_ptr<spdlog::logger> logger_;
  std::vector<std::shared_ptr<Actuator>> actuators_;
  actuator_map_t actuator_map_;
  actuator_map_ptr_t actuator_map_ptr_;
};

#endif  // SRC_ACTUATOR_CONFIG_PARSER_HPP

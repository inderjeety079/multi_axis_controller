
#include "config_parser.hpp"

ConfigParser::ConfigParserPtr ConfigParser::config_parser_ptr_ = nullptr;
bool ConfigParser::is_initialized_ = false;

ConfigParser::ConfigParser(const std::string& filename) : config_file_path_(filename) {
  logger_ = spdlog::get("butler_control_interface")->clone("config_parser");
  load_configuration();
}

void ConfigParser::load_configuration() {
  parse_json_config();
}

bool ConfigParser::parse_json_config() {
  Json::Reader reader;
  Json::Value config;
  std::filebuf fb;
  bool parsing_status = false;

  if (fb.open(config_file_path_, std::ios::in)) {
    std::istream config_stream(&fb);
    parsing_status = reader.parse(config_stream, config, false);
  }

  if (!parsing_status) {
    logger_->error("Failed to parse config json. Error: {} ", reader.getFormattedErrorMessages());
    return parsing_status;
  }

  logger_->debug("robot hardware config parsing successful");

  // Todo: Add try catch throw here
  std::string bot_variant = config["bot_variant"].asString();
  logger_->info("Bot Variant:{}", bot_variant);

  butler_config = config[bot_variant];
  // const Json::Value actuator_list = butler_config["accessories"];
  // connection_list_ = butler_config["subsystems"];
  return parsing_status;
}

bool ConfigParser::get_config_element(const std::string& base_element, Json::Value& config) {
  bool result = false;
  if (butler_config.isMember(base_element)) {
    config = butler_config[base_element];
    result = true;
  } else {
    config = Json::Value();
    result = false;
  }
  return result;
}

bool ConfigParser::get_connection_list(Json::Value& connection_config) {
  return get_config_element("subsystems", connection_config);
}

bool ConfigParser::get_accessories_list(Json::Value& accessories_config) {
  return get_config_element("accessories", accessories_config);
}

ConfigParser::ConfigParserPtr ConfigParser::get_unique_instance(
    const std::string& config_file_path_) {
  if (!is_initialized_) {
    try {
      config_parser_ptr_ = new ConfigParser(config_file_path_);
      config_parser_ptr_->logger_->debug("Created Config Parser Object");
      is_initialized_ = true;
      return config_parser_ptr_;
    } catch (std::bad_alloc& ba) {
      spdlog::get("butler_control_interface")
          ->error("Bad allocation while creating config parser : {}", ba.what());
    }
  } else {
    config_parser_ptr_->logger_->debug(
        "Instance of config parser already exists, Returning the pointer");
  }

  return config_parser_ptr_;
}

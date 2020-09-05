//
// Created by void on 17/04/20.
//

#include "actuator_config_parser.hpp"

ActuatorConfigParser::ActuatorConfigParserPtr ActuatorConfigParser::actuator_config_parser_ptr_ = nullptr;
bool ActuatorConfigParser::is_initialized_ = false;

// TODO: Fix this for DI
ActuatorConfigParser::ActuatorConfigParser(std::string filepath)
    : config_path_(std::move(filepath)) {
  logger_ = spdlog::get("butler_control_interface")->clone("actuator_config_parser");
  config_parser_ = ConfigParser::get_unique_instance(config_path_);

  Json::Value actuator_list;
  bool actuators = config_parser_->get_accessories_list(actuator_list);
  if (!actuators) {
    // Todo: throw something;
  }
  auto actuator_itr = actuators_.begin();
  std::size_t index = 0;

  for (const auto& itr : actuator_list) {
    Json::Value actuator_config;
    actuator_config = itr;

    // Get the accessory name to which the actuator is attached
    std::string accessory_idx = actuator_config["accessory_idx"].asString();
    std::string accessory_name = actuator_config["accessory_name"].asString();

    // Construct a new actuator class object for an actuator added
    // in the config json
    std::shared_ptr<Actuator> actuator = Actuator::spawn_new_actuator(accessory_name);
    actuators_.emplace_back(actuator);
    actuator_map_.emplace(accessory_name, actuator);
    // actuator->set_actuator_param("accessory_idx", accessory_idx);
    actuator->set_actuator_param("accessory_name", accessory_name);
    actuator->set_accessory_idx(actuator_config["accessory_idx"].asString());
    actuator->set_accessory_polarity(actuator_config["accessory_polarity"].asInt());
    actuator->set_actuator_param("accessory_idx", actuator_config["accessory_idx"].asString());
    load_actuator_params(actuators_.at(index), actuator_config["accessory_params"]);
    load_actuator_controls(actuators_.at(index), actuator_config["accessory_controls"]);
    // load_actuator_info(actuators_.at(index),
    //  actuator_config["accessory_info"]);

    actuator_itr++;
    index++;
  }
  actuator_map_ptr_ = std::make_shared<actuator_map_t>(actuator_map_);
}

ActuatorConfigParser::actuator_map_ptr_t ActuatorConfigParser::get_actuator_map_ptr() {
  return actuator_map_ptr_;
}

ActuatorConfigParser::actuator_map_t ActuatorConfigParser::get_actuator_map() {
  return actuator_map_;
}

bool ActuatorConfigParser::load_actuator_params(const std::shared_ptr<Actuator> actuator,
                                                Json::Value params_config) {
  actuator->set_actuator_param("accessory_params/phys_params/ticks_per_motor_rev",
                               params_config["phys_params"]["ticks_per_motor_rev"].asString());
  actuator->set_actuator_param("accessory_params/phys_params/wheel_radius",
                               params_config["phys_params"]["wheel_radius"].asString());
  actuator->set_actuator_param("accessory_params/phys_params/gearbox_ratio",
                               params_config["phys_params"]["gearbox_ratio"].asString());
  actuator->set_actuator_param("accessory_params/phys_params/max_motor_speed",
                               params_config["phys_params"]["max_motor_speed"].asString());

  /**
   * \brief set the entire class
   *
   */
  std::string actuator_specs_str;
  actuator_specs_str.append(params_config["phys_params"]["ticks_per_motor_rev"].asString());
  actuator_specs_str.append(",");
  actuator_specs_str.append(params_config["phys_params"]["wheel_radius"].asString());
  actuator_specs_str.append(",");
  actuator_specs_str.append(params_config["phys_params"]["gearbox_ratio"].asString());
  actuator_specs_str.append(",");
  actuator_specs_str.append(params_config["phys_params"]["max_motor_speed"].asString());

  actuator->set_actuator_param_class_wise("accessory_params/phys_params", actuator_specs_str);

  actuator->set_actuator_param("accessory_params/sensor_params/sensor_handle",
                               params_config["sensor_params"]["sensor_handle"].asString());

  actuator->set_actuator_param("actuator_params/comm_type",
                               params_config["actuator_params"]["comm_type"].asString());

  actuator->set_actuator_param("actuator_params/driver_name",
                               params_config["actuator_params"]["driver_name"].asString());
  actuator->set_actuator_param("actuator_params/channel",
                               params_config["actuator_params"]["channel"].asString());

  actuator->set_actuator_param("actuator_params/uart/channel",
                               params_config["actuator_params"]["uart"]["channel"].asString());
  actuator->set_actuator_param("actuator_params/uart/baudrate",
                               params_config["actuator_params"]["uart"]["baudrate"].asString());

  std::string accessory_params_str;
  accessory_params_str.append(params_config["actuator_params"]["driver_name"].asString());
  accessory_params_str.append(",");
  accessory_params_str.append(params_config["actuator_params"]["comm_type"].asString());
  accessory_params_str.append(",");
  accessory_params_str.append(",");
  accessory_params_str.append(params_config["actuator_params"]["uart"]["channel"].asString());
  accessory_params_str.append(",");
  accessory_params_str.append(params_config["actuator_params"]["uart"]["baudrate"].asString());

  actuator->set_actuator_param_class_wise("actuator_params", accessory_params_str);
  return false;
}

bool ActuatorConfigParser::load_actuator_controls(const std::shared_ptr<Actuator> actuator,
                                                  Json::Value controls_config) {
  actuator->set_actuator_control("feedback_frequency",
                                 controls_config["feedback_frequency"].asString());
  actuator->set_actuator_control("publish_feedback",
                                 controls_config["publish_feedback"].asString());
  return false;
}

ActuatorConfigParser::ActuatorConfigParserPtr ActuatorConfigParser::get_unique_instance(const std::string& config_file_path_) {
  if (!is_initialized_) {
    try {
      actuator_config_parser_ptr_ = new ActuatorConfigParser(config_file_path_);
      actuator_config_parser_ptr_->logger_->debug("Created Actuator Config Parser Object");
      is_initialized_ = true;
      return actuator_config_parser_ptr_;
    } catch (std::bad_alloc& ba) {
      spdlog::get("butler_control_interface")->error("Bad allocation while creating actuator config parser : {}", ba.what());
    }
  } else {
    actuator_config_parser_ptr_->logger_->debug("Instance of config parser already exists, Returning the pointer");
  }

  return actuator_config_parser_ptr_;
}

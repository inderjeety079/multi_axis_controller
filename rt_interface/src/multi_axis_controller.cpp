/**
 * \file axes_controller.cpp
 * @author Inderjeet Yadav (inderjeet.y@greyorange.sg)
 * \brief
 * @version 0.1
 * \date 30-07-2019
 *
 * @copyright Copyright (c) 2019 Inderjeet Yadav
 * GreyOrange India Pte Ltd
 *
 */
#include "multi_axis_controller.hpp"

bool ConfigParser::is_initialized_ = false;
//ConfigParser::config_parser_ptr_t ConfigParser::config_parser_ptr_ = nullptr;

MultiAxisController::MultiAxisController(const std::string config_file_path) {

  logger_ = spdlog::get("butler_control_interface")->clone("multi_axis_controller");
  logger_fb = spdlog::get("butler_feedback_interface")->clone("multi_axis_controller_fb");

  logger_->debug("Getting Instance of config Parser");
  act_config_parser_ptr_ = ActuatorConfigParser::get_unique_instance(config_file_path);
  
  /**
   * \brief Get the actuator handles from config parser
   *
   */
  actuator_map_ptr_ = act_config_parser_ptr_->get_actuator_map_ptr();
  actuator_map_ = act_config_parser_ptr_->get_actuator_map();

  logger_->debug("Creating RtInterface Object");
  rt_interface_ = std::make_unique<RtInterface>(config_file_path);
  logger_->debug("RtInterface Object Created");

  //pack_config_msg_new();
}

MultiAxisController::~MultiAxisController() {
}

bool MultiAxisController::set_command_new(const std::string& axis_name,
                                      std::vector<Json::Value>* command) {
    std::string command_msg;

    rt_interface_->pack_command_msg(axis_name,enum_command_req_msg_type::JOB_EXECUTE_REQ, command, &command_msg);
    if(!command_msg.empty())
      rt_interface_->send_control_iface_msg(command_msg);                             
 }

// Function is deprecated!!
bool MultiAxisController::set_command(const std::string& axis_name, const std::string& command_type,
                                      std::vector<Json::Value>* command) {
  std::string command_msg;

  rt_interface_->protocol_parser_->add_header("control_iface", &command_msg, 0);
  command_msg.append("accessory_idx=");

  command_msg.append(actuator_map_.at(axis_name)->get_accessory_idx());
  command_msg.append("&");
  //rt_interface_->pack_command_msg(axis_name, command_type, command, &command_msg);
  rt_interface_->send_control_iface_msg(command_msg);

  return true;
}

bool MultiAxisController::get_accessory_state(const std::string& axis_name,
                                              ActuatorInfo::feedback_s* feedback) {
  if (actuator_map_ptr_->at(axis_name)->actuator_info_.is_data_available()) {
    actuator_map_ptr_->at(axis_name)->get_actuator_feedback(feedback);
    return true;
  } else {
    return false;
  }
}

bool MultiAxisController::get_axes_states(
    std::unordered_map<std::string, ActuatorInfo::feedback_s>* axes_feedback) {
  //  ActuatorInfo::feedback_s feedback;
  for (auto& actuator_map_itr : actuator_map_) {
    auto actuator_name = actuator_map_itr.first;
    auto actuator_ptr = actuator_map_itr.second;

    //      Wait here till the feedback queue is empty
    bool status = actuator_ptr->get_actuator_feedback(&feedback_);

    if (status) {
      axes_feedback->operator[](actuator_name) = feedback_;
      logger_->debug(
          "Encoder feedback: axis : {}; ts: {}.{}; vel: {};"
          " abs_ticks: {}",
          actuator_name, feedback_.timestamp.secs, feedback_.timestamp.msecs,
          feedback_.meas_velocity, feedback_.abs_ticks);
    }
  }
  return true;
}

bool MultiAxisController::send_configuration_to_rt() {
}



bool MultiAxisController::pack_config_msg_new() {

  /* Local definitions */
  Json::Value actuator_params,actuator_controls;
  std::vector<Json::Value> actuator_data_vec;
  std::string packed_msg;

  logger_->debug("Packing Robot Configuration for the RT side:");

  for (auto& actuator : actuator_map_) {
    auto actuator_name = actuator.first;
    auto actuator_ptr_value = actuator.second;

    logger_->debug("Axis: {} ", actuator_name);

    /* Pack actuator config */
    std::vector<std::pair<std::string, std::string>> params_vector;
    actuator_ptr_value->get_accessory_params(&params_vector);
    // Convert vector of pairs to a vector of Json for uniformity on packing side
    actuator_params.clear();
    actuator_data_vec.clear();

    for (auto& element : params_vector) {
      logger_->debug("{}: {} ", element.first, element.second);
      actuator_params[element.first] = element.second;
    }
    actuator_data_vec.push_back(actuator_params);
  
    packed_msg.clear();
    rt_interface_->protobuf_parser_->pack_cmd_msg(std::stoi(actuator_ptr_value->get_accessory_idx()),\
                          enum_command_req_msg_type::CREATE_ACTUATOR_REQ , &actuator_data_vec, &packed_msg);

    /* Send actuator config */
     if(!packed_msg.empty())
      rt_interface_->send_control_iface_msg(packed_msg); 

    std::this_thread::sleep_for(std::chrono::milliseconds(30)); 

    /* Pack sensor config*/
    actuator_controls.clear();
    actuator_data_vec.clear();

    std::vector<std::pair<std::string, std::string>> controls_vector;
    actuator_ptr_value->get_accessory_controls(&controls_vector);

    // Convert vector of pairs to a vector of Json for uniformity on packing side
    for (auto& element : controls_vector) {
      logger_->debug("{}: {} ", element.first, element.second);
      actuator_controls[element.first] = element.second;
    }

    // Add params to controls object too
    for (auto& element : params_vector) {
      //logger_->debug("{}: {} ", element.first, element.second);
      actuator_controls[element.first] = element.second;
    }

    actuator_data_vec.push_back(actuator_controls);

    packed_msg.clear();
    rt_interface_->protobuf_parser_->pack_cmd_msg(std::stoi(actuator_ptr_value->get_accessory_idx()),\
                          enum_command_req_msg_type::CREATE_SENSOR_REQ , &actuator_data_vec, &packed_msg);

    /* Send sensor config  */
    if(!packed_msg.empty())
      rt_interface_->send_control_iface_msg(packed_msg);  
    
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

  }


}

// Function is deprecated!
bool MultiAxisController::pack_config_msg() {
  ProtocolParser* protocol_parser = new ProtocolParser();

  logger_->debug("Robot Configuration:");

  for (auto& actuator : actuator_map_) {
    auto actuator_name = actuator.first;
    auto actuator_ptr_value = actuator.second;

    logger_->debug("Axis: {} ", actuator_name);

    std::vector<std::pair<std::string, std::string>> params_vector;
    actuator_ptr_value->get_accessory_params(&params_vector);
    for (auto& element : params_vector) {
      logger_->debug("{}: {} ", element.first, element.second);
    }

    std::string packed_msg;
    protocol_parser->add_header("control_iface", &packed_msg, 0);
    packed_msg.append("create=");
    std::string axis_idx = actuator_ptr_value->get_accessory_idx();
    packed_msg.append(axis_idx);
    packed_msg.append("&");
    protocol_parser->pack_msg(actuator_name, &params_vector, &packed_msg);
    //on_connect_message_ += packed_msg;

    std::vector<std::pair<std::string, std::string>> controls_vector;
    actuator_ptr_value->get_accessory_controls(&controls_vector);

    for (auto& element : controls_vector) {
      logger_->debug("{}: {} ", element.first, element.second);
    }

    std::string packed_controls_msg;
    protocol_parser->add_header("control_iface", &packed_controls_msg, 0);
    packed_controls_msg.append("update=");
    packed_controls_msg.append(actuator_ptr_value->get_accessory_idx());
    packed_controls_msg.append("&");
    //    protocol_parser->pack_msg(actuator_name, &controls_vector,
    //                               &packed_controls_msg);
    //    on_connect_message_ += packed_msg;
  }
}

void MultiAxisController::on_init(void) {
  rt_interface_->on_init();
  // wait for rt_interfaces to init
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  pack_config_msg_new();
  //send_configuration_to_rt();
}

bool MultiAxisController::get_feedback_iface_conn_status() {
  return rt_interface_->get_feedback_iface_conn_status();
}

bool MultiAxisController::get_control_iface_conn_status() {
  return rt_interface_->get_control_iface_conn_status();
}

bool MultiAxisController::get_rt_iface_status() {
  return rt_interface_->get_rt_interface_conn_status();
}

void MultiAxisController::wait_for_rt_iface() {
  rt_interface_->wait_for_rt_interface();
}

void MultiAxisController::join_rt_listener_threads() {
  rt_interface_->join_listener_threads();
}

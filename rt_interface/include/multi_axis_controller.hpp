/**
 * \file rt_controller.hpp
 * @author Inderjeet Yadav (inderjeet.y@greyorange.sg)
 * \brief
 * @version 0.1
 * \date 30-07-2019
 *
 * @copyright Copyright (c) 2019 Inderjeet Yadav
 * GreyOrange India Pte Ltd
 *
 */

#include "rt_interface.hpp"
#include <connection_manager.hpp>
// #include "config_parser.hpp"
#include "actuator_config_parser.hpp"

class MultiAxisController {
 public:
  typedef std::shared_ptr<MultiAxisController> multiaxis_controller_ptr_t;

  static bool is_initialized_;
  explicit MultiAxisController(const std::string config_file_path);
  ~MultiAxisController();

  std::shared_ptr<spdlog::logger> logger_;
  std::shared_ptr<spdlog::logger> logger_fb;

 private:
  RtInterface::rt_interface_u_ptr_t rt_interface_;
  //ConfigParser::config_parser_ptr_t config_parser_ptr_;
  ActuatorConfigParser::ActuatorConfigParserPtr act_config_parser_ptr_;
  ActuatorConfigParser::actuator_map_ptr_t actuator_map_ptr_;
  ActuatorConfigParser::actuator_map_t actuator_map_;

  ActuatorInfo::feedback_s feedback_;

 public:
 // Uses protocol
  bool set_command(const std::string& axis_name,
                   const std::string& command_type,
                   std::vector<Json::Value>* command);
  // Uses protobuf
  bool set_command_new(const std::string& axis_name,
                   std::vector<Json::Value>* command);

  bool get_accessory_state(const std::string& axis_name,
                           ActuatorInfo::feedback_s* feedback);

  bool get_axes_states(
      std::unordered_map<std::string, ActuatorInfo::feedback_s>* axes_feedback);

  bool get_feedback_iface_conn_status();

  bool get_control_iface_conn_status();

  bool get_rt_iface_status();

  void wait_for_rt_iface();

  void on_init();

  void join_rt_listener_threads();

  bool send_configuration_to_rt();

  // Uses protobuf
  bool pack_config_msg_new();

  // Uses protocol
  bool pack_config_msg();
  void get_rt_interface(RtInterface::rt_interface_u_ptr_t* rt_interface) {
    //    rt_interface = rt_interface_.get();
  }
  ActuatorConfigParser::actuator_map_ptr_t get_actuator_map_ptr();

  ActuatorConfigParser::actuator_map_t get_actuator_map() { return actuator_map_; }
};
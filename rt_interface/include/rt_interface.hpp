/**
 * \file rt_interface.hpp
 * @author Inderjeet Yadav (inderjeet.y@greyorange.sg)
 * \brief 
 * @version 0.1
 * \date 18-07-2019
 * 
 * @copyright Copyright (c) 2019 Inderjeet Yadav
 * GreyOrange India Pte Ltd
 * 
 */
#include <inttypes.h>
#include <json/json.h>

#include <connection_manager.hpp>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "../../interface/include/config_parser.hpp"
#include "protocol_parser.hpp"
#include "config_parser.hpp"
#include "actuator_config_parser.hpp"
#include "interface.hpp"
#include "protobuf_parser.hpp"

/** @note to developer : currently axis = actuator = sensor = accessory */

class RtInterface : public Interface {

 private:
  std::string robot_config_file_path_;
  std::string bot_variant_;

  Connection::ConnectionSPtrType control_interface_;
  Connection::ConnectionSPtrType feedback_interface_;
  ActuatorConfigParser::ActuatorConfigParserPtr config_parser_ptr_;

  std::thread control_iface_listener_;
  std::thread control_iface_send_;

  std::thread update_feedback_thread_;


/**
 * \brief Thread for listening the feedback msg published by RT
 * 
 */
  std::thread feedback_iface_listener_;

  ActuatorConfigParser::actuator_map_t actuator_map_;
  /**
   * \brief 
   * 
   */
  std::condition_variable control_msg_send_cv_;
  /**
   * \brief 
   * 
   */
  bool new_msg_available_;
  /**
   * \brief 
   * 
   */
  std::string rt_message_string_;
  /**
   * \brief 
   * 
   */
  bool receive_msg_thread_sig;

  bool rt_conn_status_;
  std::condition_variable rt_conn_cv_;
//  std::mutex mutexLock;

  int control_iface_seq_id_;
  int feedback_iface_seq_id_;

  bool feedback_data_avail_;
  std::unordered_map<std::string, bool> feedback_data_state_map_;
  std::condition_variable feedback_data_cv_;
  std::mutex feedback_data_mutex_;
  //ConnectionManager::ConnectionManagerSPtrType connection_manager_;


 public:
  /**
   * \brief Construct a new Rt Interface object
   * 
   * \param robot_config_file_path
   */
  explicit RtInterface(std::string config_file_path,std::string connection_name = " ",std::string on_connect_msg_ = "RT Interface is established!");
  /**
   * \brief Destroy the Rt Interface object
   * 
   */
  ~RtInterface();
  /**
   * \brief 
   * 
   */
  void on_init();
  void join_listener_threads();
  /**
   * \brief 
   * 
   */
  void read_robot_config();
  /**
   * \brief Based on protocol
   * 
   */
  void parse_control_iface_msg();
   /**
   * \brief Based on protobuf
   * 
   */
  void parse_control_iface_msg_new();
  /**
   * \brief 
   * 
   */
  void retrieve_control_iface_msg(std::string *msg);
  /**
   * \brief 
   * 
   * \param str 
   */
  void send_control_iface_msg(const std::string& str);

  /**
   * \brief Old function which used protocol_parser
   * 
   */
  void parse_feedback_iface_msg();

    /**
   * \brief New function which uses protobuf parser
   * 
   */
  void parse_feedback_iface_msg_new();
  /**
   * \brief 
   * 
   */
  void retrieve_feedback_iface_msg(std::string *msg);
  /**
   * \brief 
   * 
   * \param str 
   */
  void send_feedback_iface_msg(const std::string& str);

  /**
   * \brief 
   * 
   * \param axis_name 
   * \param key_value_vector 
   * \return std::string 
   */
  bool pack_rt_msg(const std::string &axis_name,
    std::vector<std::pair<std::string, std::string>> *key_value_vector, std::string *packed_string);
  /**
   * \brief 
   * 
   * \param str 
   */
  bool unpack_control_iface_msg(const std::string& str);

  bool pack_command_msg(const std::string &axis_name,
   const enum_command_req_msg_type command_type, std::vector<Json::Value>* command,
   std::string *msg);

  bool unpack_feedback_msg(std::string &msg);

  bool get_accessory_name(
    std::vector<std::pair<std::string, std::string>>* key_value_vector,
    std::string *axis_name);

  bool get_accessory_name_new(
    std::map<std::string, std::string> key_value_map,
    std::string *axis_name);

  bool get_accessory_id(std::string axis_name,int &axis_id);

  /**
   * \brief 
   * 
   * \param axis_name 
   * \param key_value_vector 
   * \return true 
   * \return false 
   */
  // bool update_data(std::string axis_name,
  //  std::vector<std::pair<std::string, std::string>> *key_value_vector);
  
  bool update_data_map(const std::string axis_name,
    std::vector<std::pair<std::string, std::string>> *key_value_vector);
    /**
   * \brief 
   * 
   */
  bool update_data_map_new(const std::string axis_name,
    std::map<std::string, std::string> key_value_map);
  /**
   * \brief 
   * 
   */
  void send_actuators_params_to_rt(const std::string axis_name,
    std::vector<std::pair<std::string, std::string>> *params_vector);

/**
 * \brief 
 * 
 */
  void send_actuators_params_to_rt_class_wise(void);
  /**
   * \brief 
   * 
   */
  void send_actuators_controls_to_rt(void);
  /**
   * \brief 
   * 
   */
  void send_actuators_info_params_to_rt(void);

  Connection::ConnectionSPtrType get_control_iface(void) {
    return control_interface_;
  }

  Connection::ConnectionSPtrType get_feedback_iface(void) {
    return feedback_interface_;
  }

  bool get_control_iface_conn_status(void) {
    return control_interface_->get_conn_status();
  }

  bool get_feedback_iface_conn_status(void) {
    return feedback_interface_->get_conn_status();
  }

  bool get_rt_interface_conn_status(void) {
    rt_conn_status_ = (get_control_iface_conn_status() &&
      get_feedback_iface_conn_status());
    rt_conn_cv_.notify_all();
    return rt_conn_status_;
  }

  void wait_for_rt_interface(void) {
    std::mutex mutexLock;
    std::unique_lock<std::mutex> lock(mutexLock);

    bool send_init_params = false;

    if (!rt_conn_status_) {
      send_init_params = true;
    }
    //std::cout << "Waiting for rt connection" << std::endl;
    rt_conn_cv_.wait(lock, [this] { return rt_conn_status_;});
    //std::cout << "rt connection flag true" << std::endl;
    if (send_init_params) {
      // on_init();
    }

  }
  bool update_feedback();

  bool update_job_status(std::string axis_name, std::vector<std::pair<std::string, std::string>>* key_value_vector);

  // Uses protobuf
  bool update_job_status_new(std::map<std::string,std::string> info_key_value_map);

  int get_control_iface_seq_id() {
    if (control_iface_seq_id_ > INT32_MAX) {
      control_iface_seq_id_ = 0;
    }
    control_iface_seq_id_++;
    return control_iface_seq_id_;
  }

  int get_feedback_iface_seq_id() {
    if (feedback_iface_seq_id_ > INT32_MAX) {
      feedback_iface_seq_id_ = 0;
    }
    feedback_iface_seq_id_++;
    return feedback_iface_seq_id_;
  }

  void notify_one_rt_conn_cv() {
    rt_conn_cv_.notify_one();
  }

  void set_rt_conn_status(bool value) {
    rt_conn_status_ = value;
  }

  typedef std::unique_ptr<RtInterface> rt_interface_u_ptr_t;
  std::shared_ptr<spdlog::logger> logger_;
  std::shared_ptr<spdlog::logger> logger_fb;
  std::unique_ptr<ProtocolParser> protocol_parser_;  //deprecated
  std::unique_ptr<ProtobufParser> protobuf_parser_;
};

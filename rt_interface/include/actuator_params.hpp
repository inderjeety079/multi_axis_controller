#include <iostream>
#include <inttypes.h>
#include <unordered_map>

#include <json/json.h>

class ActuatorParams {

 public:
  ActuatorParams() {
      state_ = 1;
  }
  ~ActuatorParams() {}

 private:

  typedef struct {
    std::string class_params_str_;
    uint32_t ticks_per_motor_rev_;
    float wheel_radius_;
    float gearbox_ratio_;
    uint32_t max_motor_speed_;
  }actuator_specs_s;

  typedef struct uart_comm_params_ {
      std::string port_;
      uint32_t baud_rate_;
  }uart_comm_params_s;

  typedef struct can_comm_params_ {
      std::string address_;
  }can_comm_params_s;

  typedef struct motor_driver_params_ {
      std::string class_params_str_;
      std::string device_name_;
      std::string comm_interface_;
      union comm_param_{
          uart_comm_params_s uart_;
          can_comm_params_s can_;
      };
  }motor_driver_params_s;


  actuator_specs_s actuator_specs_;
  motor_driver_params_s motor_driver_params_;
  std::string encoder_handle_;

  bool state_;

  Json::Value phys_params_;
  Json::Value encoder_params_;
  Json::Value md_params_;


 public:
  std::unordered_map<std::string, std::string> params_map_;
  std::unordered_map<std::string, std::string> class_wise_params_map_;
  uint32_t get_ticks_per_motor_rev() {
      return actuator_specs_.ticks_per_motor_rev_;
  }

  void set_ticks_per_motor_rev(const uint32_t & ticks_per_motor_rev) {
      actuator_specs_.ticks_per_motor_rev_ = (uint32_t)(std::stoul(
            params_map_.at("ticks_per_motor_rev"), nullptr, 10));
  }

  double get_wheel_radius() {
      return actuator_specs_.wheel_radius_;
  }
  void set_wheel_radius(const double & wheel_radius) {
      actuator_specs_.wheel_radius_ = std::stof(params_map_.at("wheel_radius"));
  }

  float get_gearbox_ratio(void) {
      return actuator_specs_.gearbox_ratio_;
  }
  void set_gearbox_ratio(const float & gearbox_ratio) {
      actuator_specs_.gearbox_ratio_ = std::stof(params_map_.at("gearbox_ratio"));
  }

  uint32_t get_max_motor_speed(void) {
      return actuator_specs_.max_motor_speed_;
  }

  void set_max_motor_speed(void) {
      actuator_specs_.max_motor_speed_ = std::stof(params_map_.at("max_motor_speed"));
  }

  std::string get_encoder_handle(void) {
      return encoder_handle_;
  }

  void set_encoder_handle(void) {
      encoder_handle_ = params_map_.at("encoder_handle");
  }

  std::string get_param(const std::string &key) {
      return params_map_.at(key);
  }

  void set_param(const std::string &key, const std::string &value) {

      auto key_idx = params_map_.find(key);

      if (key_idx != params_map_.end()) {
        params_map_.at(key) = value;
//        std::cout << "key : { " << key << " } exists already. replacing value : { " << value <<" }"<< std::endl;
      } else {
        params_map_.emplace(key, value);
//        std::cout << "Inserting key : { " << key << " }" << " value: { " << value << "}" << std::endl;
      }
  }



  std::string get_key_value_string(const std::string &key) {

      std::string key_value_string("");
      key_value_string.append(key);
      key_value_string.append("=");
      key_value_string.append(params_map_.at(key));

      return key_value_string;
  }

  void update_params(void) {

  }

  std::string get_params_str_by_class(std::string param_class) {
      return class_wise_params_map_.at(param_class);
  }

  bool set_params_str_by_class(std::string param_class, std::string param_class_value) {
     class_wise_params_map_.emplace(param_class, param_class_value);
  }

};

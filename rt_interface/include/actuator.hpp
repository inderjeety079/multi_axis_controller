/**
 * \file actuator.hpp
 * @author Inderjeet Yadav (inderjeet.y@greyorange.sg)
 * \brief 
 * @version 0.1
 * \date 10-07-2019
 * 
 * @copyright Copyright (c) 2019 Inderjeet Yadav
 * GreyOrange India Pte Ltd
 * 
 */
#include <string>
#include <unordered_map>
#include <memory>
#include <utility>
#include <vector>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include "actuator_params.hpp"
#include "actuator_info.hpp"
#include "actuator_controls.hpp"
#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

class Actuator : public boost::enable_shared_from_this<Actuator>{

 private:
  std::string axis_name_;
  std::string axis_idx_;
  int axis_polarity_;
  std::shared_ptr<spdlog::logger> logger_;

 public:
  ActuatorParams actuator_params_;
  ActuatorControls actuator_controls_;
  ActuatorInfo actuator_info_;
  typedef std::shared_ptr<Actuator> ActuatorSPtrType;
  typedef std::unique_ptr<Actuator> ActuatorUPtrType;

 public:
  Actuator(std::string axis_name):actuator_info_(100) {
    axis_name_ = axis_name;
  }
//  Actuator() {
//  }

  ~Actuator() {
  }

  typedef boost::shared_ptr<Actuator> pointer;

//  static pointer create(void) {
//    return pointer(new Actuator());
//  }

  std::string get_actuator_param(const std::string &key) {
      return actuator_params_.get_param(key);
  }

  void set_actuator_param(const std::string &key, const std::string &value) {
      actuator_params_.set_param(key, value);
  }

  std::string get_actuator_param_class_wise(std::string param_class) {
      return actuator_params_.get_params_str_by_class(param_class);
  }

  bool set_actuator_param_class_wise(std::string param_class, std::string
   param_class_value) {
      actuator_params_.set_params_str_by_class(param_class, param_class_value);
  }

  std::string get_actuator_control(const std::string &key) {
      return actuator_controls_.get_control(key);
  }

  void set_actuator_control(const std::string &key, const std::string &value) {
      actuator_controls_.set_control(key, value);
  }

  std::string get_actuator_info(const std::string &key) {
       actuator_info_.get_info(key);
  }

  bool set_job_status(const ActuatorControls::job_status_s &job_status) {
    return actuator_controls_.set_job_status(job_status);
  }

  void get_job_status(ActuatorControls::job_status_s *job_status, bool *job_status_q_empty) {
    actuator_controls_.get_job_status(job_status, job_status_q_empty);
  }

  void set_actuator_info(const std::string &key, const std::string &value) {
       actuator_info_.set_info(key, value);
  }

  bool get_actuator_feedback(ActuatorInfo::feedback_s * feedback) {

//      std::cout << "axis_name: " << axis_name_ << "  ";
      bool status = actuator_info_.get_feedback(feedback);
      if (status) {
          return true;
      } else {
          return false;
      }
  }

  bool set_feedback(ActuatorInfo::feedback_s &feedback) {
      bool status = actuator_info_.set_feedback(&feedback);
      return status;
  }

  std::string get_accessory_name() {
      return axis_name_;
  }

  void set_accessory_name(const std::string & axis_name) {
      axis_name_ = axis_name;
  }

  void set_accessory_idx(std::string axis_id) {
    axis_idx_ = axis_id;
  }

  std::string get_accessory_idx() {
    return axis_idx_;
  }

  void set_accessory_polarity(int polarity) {
      axis_polarity_ = polarity;
  }

  int get_actuator_polarity() {
      return axis_polarity_;
  }

  void load_config_file(std::string filename);

  void get_accessory_params(
      std::vector<std::pair<std::string, std::string>> *key_value_vector) {
      std::cout << "param map size : " << actuator_params_.params_map_.size() << std::endl;
      for (const auto& itr : actuator_params_.params_map_) {
          key_value_vector->push_back(std::make_pair(itr.first,
              itr.second));
      }
    key_value_vector->resize(actuator_params_.params_map_.size());
  }

  void get_accessory_params_class_wise(
      std::vector<std::pair<std::string, std::string>> *key_value_vector) {

      for (const auto& itr : actuator_params_.class_wise_params_map_) {
          key_value_vector->push_back(std::make_pair(itr.first, itr.second));
      }
  }

  void get_accessory_controls(std::vector<std::pair<std::string, std::string>>
   *key_value_vector) {
      for (const auto& itr : actuator_controls_.controls_map_) {
          key_value_vector->push_back(std::make_pair(itr.first, itr.second));
      }
  }


  static std::shared_ptr<Actuator> spawn_new_actuator(std::string axis_name) {
     return  std::make_shared<Actuator>(axis_name);
  }
};


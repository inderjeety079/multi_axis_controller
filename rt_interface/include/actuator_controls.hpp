#include <vector>
#include <string>
#include <list>
#include <unordered_map>
#include "json/json.h"

class ActuatorControls {

 public:
  ActuatorControls() {}
  ~ActuatorControls() {}

  typedef enum operating_mode_e_
  {
    UNKNOWN_MODE = -1,
    VELOCITY_MODE = 1,
    POSITION_MODE
  }operating_mode_e_t;

  typedef struct velocity_cmd_ {
    double timeout;
    double velocity;
  }velocity_cmd_t;

  typedef struct position_cmd_ {
    double timeout;
    double relative_pos;
    double max_vel;
    double accel;
    double decel;
  }position_cmd_t;

  typedef struct job_status_
  {
    uint32_t job_id;
    uint32_t job_timeout;
    int64_t timestamp;
    int success;
    int error;
    int sub_error;
    bool is_recvd;

  } job_status_s;

 private:
  uint32_t feedback_frequency;
  uint32_t publish_feedback;
  uint32_t reset_data;

  Json::Value publish_feedback_;
  Json::Value command_type_;
  std::vector<Json::Value> actuator_command_;
  std::vector<ActuatorControls::velocity_cmd_t> velocity_cmd_;
  std::vector<ActuatorControls::position_cmd_t> position_cmd_;
  ActuatorControls::job_status_s job_status_;
  std::unordered_map<uint32_t, ActuatorControls::job_status_s> job_status_map_;


 public:
  std::unordered_map<std::string, std::string> controls_map_;
  std::unordered_map<std::string, std::string> class_wise_controls_map_;
  ActuatorControls::velocity_cmd_t velocity_cmd_element;
  ActuatorControls::position_cmd_t position_cmd_element;
  operating_mode_e_t command_operation_mode;

  bool is_new_cmd;

  
  std::string get_control(const std::string &key) {
    try {
      std::string value = controls_map_.at(key);
      return value;
    } catch ( int x) {
      std::cout << "Key not present" << std::endl;
    }

    return "";
  }

  void set_control(const std::string &key, const std::string &value) {

      auto key_idx = controls_map_.find(key);

      if (key_idx !=controls_map_.end()) {
          controls_map_.at(key) = value;
      } else {
          controls_map_.emplace(key, value);
      }

  }

  std::string get_key_value_string(const std::string &key) {
      std::string key_value_string("");
      key_value_string.append(key);
      key_value_string.append("=");
      key_value_string.append(controls_map_.at(key));
      return key_value_string;
  }

  void get_control_map();
  void set_control_map();
  void update_controls(void);

  void set_command(std::string cmd_type, std::vector<Json::Value>* command) {

    for (std::vector<Json::Value>::iterator it = command->begin();
      it != command->end(); it++) {
        actuator_command_.push_back(*it);
    }
  }

  bool set_job_status(const ActuatorControls::job_status_s &job_status) {

    bool status = false;
    job_status_ = job_status;

    auto job_status_ptr = job_status_map_.find(job_status.job_id);

    if(job_status_ptr != job_status_map_.end()) {
      job_status_map_[job_status.job_id] = job_status;
      std::cout << "setting job status for job id: " << job_status.job_id << std::endl;
    }

    return status;
  }

  bool emplace_job_status(const ActuatorControls::job_status_s &job_status) {
    job_status_map_.emplace(job_status.job_id, job_status);
    return true;
  }

  bool remove_job_status(uint32_t job_id) {

    job_status_map_.erase(job_id);
    return true;
  }

  bool is_job_status_q_empty() {
    if(job_status_map_.empty()) {
      return true;
    } else {
      return false;
    }
  }

  bool get_job_status(ActuatorControls::job_status_s *job_status, bool *job_status_q_empty) {

    bool status = true;

    //TODO: put locks on job_status_map
    if(!job_status_map_.empty()) {

      for(auto &element:job_status_map_) {
        uint32_t job_id = job_status->job_id;
        job_status_s value = element.second;

        *job_status = value;

        if(value.is_recvd) {
          *job_status_q_empty = false;
          // TODO: find an efficient way to do it
          job_status_map_.erase(job_status->job_id);
          std::cout << "Erased job id " << job_id << "value.is_recvd : " << value.is_recvd <<std::endl;
          break;
        }

      }
    }

    else {
      status = false;
    }

    if(job_status_map_.empty()) {
      *job_status_q_empty = true;
    }


    return status;
  }

};

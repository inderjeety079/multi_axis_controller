#ifndef ACTUATOR_INFO_H

#define ACTUATOR_INFO_H

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <condition_variable>
#include <mutex>

class ActuatorInfo {

 public:

  typedef struct timestamp_{
    int32_t secs;
    int32_t msecs;
  }timestamp_t;

  typedef struct _feedback{
    int32_t seq_id;
    ActuatorInfo::timestamp_t timestamp;
    int32_t encoder_handle;
    int32_t abs_ticks;
    double meas_velocity;  // in rpm
    uint32_t gpio_sensors_state;
  }feedback_s;

 private:
  feedback_s feedback_element_;
  std::queue<feedback_s> feedback_;
  std::mutex mutexLock;
  bool data_condition_;
  bool data_condition_break_;
  bool data_avail_to_read_;
  std::condition_variable data_condition_cv_;
  std::condition_variable read_condition_cv_;
  int queue_size_;

  uint32_t digital_io_status_;


 public:
  std::unordered_map<std::string, std::string> info_map_;
  std::unordered_map<std::string, std::string> class_wise_info_map_;

  ActuatorInfo(int queue_size) {
    queue_size_ = queue_size;
    feedback_.push(feedback_element_);
//    info_map_.emplace("")
//    feedback_.resize(queue_size_);
  }

  ~ActuatorInfo() {}


  void set_info(const std::string &key, const std::string &value) {
      auto key_idx = info_map_.find(key);

    if (key_idx != info_map_.end()) {
        info_map_[key] = value;
//       std::cout << "Info Key: {" << key << "} = "<< value << " updated"<< std::endl;
    } else {
        std::cout << "Info Key: {" << key << "} = "<< value << " inserted"<< std::endl;
        info_map_.emplace(key, value);
    }
  }

  std::string get_info(const std::string &key) {
    auto key_idx = info_map_.find(key);

    if (key_idx != info_map_.end()) {
      return info_map_.at(key);
    } else {
      return nullptr;
    }
  }

  std::unordered_map<std::string, std::string> get_info_map() {
    return info_map_;
  }

  std::string get_info_map_class_wise (std::string info_class) {
    return class_wise_info_map_.at(info_class);
  }

  bool set_feedback(ActuatorInfo::feedback_s * feedback) {
    feedback_.push(*feedback);
//    std::cout<< "Data Pushed to the queue" << std::endl;
    data_avail_to_read_ = true;
    read_condition_cv_.notify_one();
    return true;
  }


  bool get_feedback(ActuatorInfo::feedback_s * feedback) {

    std::unique_lock<std::mutex> lock(mutexLock);

    read_condition_cv_.wait(lock,
      [this] { return (data_avail_to_read_);});

    if (feedback_.empty()) {
      data_avail_to_read_ = false;
      read_condition_cv_.notify_one();
//      std::cout<< "feedback queue empty" << std::endl;
    }

    else {
      *feedback = feedback_.front();
      feedback_.pop();
      if (feedback_.empty()) {
        data_avail_to_read_ = false;
      }
//      std::cout << "feedback element popped " << std::endl;
      return true;
    }

    return false;
  }

  bool update_feedback_with_map() {
//    ActuatorInfo::feedback_s feedback ;
    int count_fields = 0;
    auto itr = info_map_.find("seq_id");
    if (itr != info_map_.end()) {
      feedback_element_.seq_id = (uint32_t)std::stoul(itr->second);
      count_fields ++;
    }

    itr = info_map_.find("ts_seconds");
    if (itr != info_map_.end()) {
        feedback_element_.timestamp.secs = (uint32_t)std::stoul(itr->second);
      count_fields ++;
    }

    itr = info_map_.find("ts_msec");
    if (itr != info_map_.end()) {
        feedback_element_.timestamp.msecs = (uint32_t)std::stoul(itr->second);
      count_fields ++;
    }


    itr = info_map_.find("encoder_handle");
    if (itr != info_map_.end()) {
        feedback_element_.encoder_handle = (uint32_t)std::stoul(itr->second);
      count_fields ++;
    }

    itr = info_map_.find("absolute_ticks");
    if (itr != info_map_.end()) {
        feedback_element_.abs_ticks = (uint32_t)std::stoul(itr->second);
      count_fields ++;
    }

    itr = info_map_.find("current_velocity");
    if (itr != info_map_.end()) {
        feedback_element_.meas_velocity = std::stod(itr->second);
      count_fields ++;
    }

    itr = info_map_.find("gpio_sensors_state");
    if (itr != info_map_.end()) {
      feedback_element_.gpio_sensors_state = std::stod(itr->second);
      count_fields ++;
    }

//    std::cout << "feedback fields matched count" << count_fields << std::endl;
    if(3 < count_fields)
      set_feedback(&feedback_element_);
  }

  bool is_data_available() {
    return !feedback_.empty();

  }

};

#endif /* ACTUATOR_INFO_H */
#include "multi_axis_controller.hpp"
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
// #include <memory>

class RosInterface {

 private:
  std::string config_filename_;
  std::thread rt_cmd_write_thread_;
  std::thread read_feedback_thread_;
  geometry_msgs::Twist cmd_;
  bool rt_iface_status_;
  std::shared_ptr<spdlog::logger> logger_;
  uint32_t  job_element_seq_id_;
  ros::NodeHandle nh_;

 public:
  explicit RosInterface(std::string config_filename,ros::NodeHandle nh);
  ~RosInterface();
  MultiAxisController::multiaxis_controller_ptr_t multi_axis_controller_ptr_;

  void cmd_vel_callback(const geometry_msgs::Twist &cmd_vel);
  void write_cmd_to_rt();
  void read_feedback();

  ros::Subscriber cmd_vel_subs;
  ros::Publisher left_wheel_pub;
  ros::Publisher right_wheel_pub;
  ros::Publisher odom_pub;
  bool keep_alive_;
  std::condition_variable new_cmd_cv_;
  bool cmd_available_;

  bool get_rt_iface_status() {
    return multi_axis_controller_ptr_->get_rt_iface_status();
  }

  void wait_for_rt_iface() {
      return multi_axis_controller_ptr_->wait_for_rt_iface();
  }

  void on_init() {
      return multi_axis_controller_ptr_->on_init();
  }

  void join_rt_listener_threads() {
    multi_axis_controller_ptr_->join_rt_listener_threads();
  }

  void get_rt_interface(RtInterface::rt_interface_u_ptr_t *rt_interface) {
    multi_axis_controller_ptr_->get_rt_interface(rt_interface);
  }

  int get_job_seq_id(void) {
    if (job_element_seq_id_ > INT32_MAX) {
      job_element_seq_id_ = 0;
    }
    job_element_seq_id_++;
    return job_element_seq_id_;
  }

};

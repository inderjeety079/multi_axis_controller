
#include "ros_interface.hpp"

RosInterface::RosInterface(std::string config_filename, ros::NodeHandle nh):nh_(nh) {

  config_filename_ = config_filename;
  cmd_available_ = false;
  logger_ = spdlog::get("butler_control_interface")->clone("ros_interface");

  logger_->debug("Getting Instance of Axes Controller");

  keep_alive_ = true;
  job_element_seq_id_ = 0;

  multi_axis_controller_ptr_ = std::make_shared<MultiAxisController> (
    config_filename);

  rt_cmd_write_thread_ = std::thread(&RosInterface::write_cmd_to_rt, this);
  read_feedback_thread_ = std::thread(&RosInterface::read_feedback, this);

  odom_pub = nh_.advertise<nav_msgs::Odometry>("feedback_odom", 1000);
}

RosInterface::~RosInterface() {
  rt_cmd_write_thread_.join();
  read_feedback_thread_.join();
}

void RosInterface::cmd_vel_callback(const geometry_msgs::Twist &cmd_vel) {

    cmd_available_ = true;
    cmd_ = cmd_vel;
    new_cmd_cv_.notify_one();

}

void RosInterface::write_cmd_to_rt() {

  std::mutex mutexLock;
  double ws = 0.68; // meters
  double wheel_radius = 0.1016;

  while (keep_alive_) {
    std::unique_lock<std::mutex> lock(mutexLock);

    new_cmd_cv_.wait(lock,
      [this] { return cmd_available_;});
    cmd_available_ = false;

    /* Wait for client to connect */
    if (keep_alive_ && rt_iface_status_) {
      multi_axis_controller_ptr_->wait_for_rt_iface();
      // TODO(inderjeet.y@greyorange.sg): write to rt ( call multi axis
      //  controller set_command)
      Json::Value left_wheel, right_wheel;
      std::vector<Json::Value> left_wheel_vec, right_wheel_vec;

      left_wheel["timestamp"]= 0;
      left_wheel["seq_id"] = get_job_seq_id();
      left_wheel["job_type"] = enum_actuatorjobtypes::MTR_CNTRL_SPEED_MODE;
      left_wheel["timeout"] = 0.2; //sec
      //left_wheel["velocity"] = ((cmd_.linear.x - ((cmd_.angular.z * ws)/ 2.0))
      //  / wheel_radius);
      left_wheel["velocity"] = cmd_.linear.x;

      left_wheel_vec.push_back(left_wheel);

      right_wheel["timestamp"] = 0;
      //right_wheel["seq_id"] = get_job_seq_id();
      right_wheel["job_type"] = enum_actuatorjobtypes::MTR_CNTRL_SPEED_MODE; 
      right_wheel["timeout"] = 0.2; //sec
      right_wheel["velocity"] = ((cmd_.linear.x + ((cmd_.angular.z * ws)/ 2.0))
        / wheel_radius);

      right_wheel_vec.push_back(right_wheel);

      multi_axis_controller_ptr_->set_command_new("left_wheel",
        &left_wheel_vec);
      //multi_axis_controller_ptr_->set_command_new("right_wheel",
      //  &right_wheel_vec);
      logger_->info("set command: left_wheel_velocity: {} cm/s"
                    " right_wheel_velocity: {} cm/s", left_wheel["velocity"],
                    right_wheel["velocity"]);

    } else {

      if (!keep_alive_) {
        logger_->debug("Shutting Down ros interface write thread");

      } else if (!rt_iface_status_) {

        logger_->debug("Suspending for 50 ms as rt interface is not up");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

      }
    }
  }
}

void RosInterface::read_feedback() {

  int feedback_interval = 5;

  while (keep_alive_) {

    // TODO: Resize the maps according to config
    std::unordered_map<std::string, ActuatorInfo::feedback_s> feedback_map;
    multi_axis_controller_ptr_->wait_for_rt_iface();

    multi_axis_controller_ptr_->get_axes_states(&feedback_map);

    if (!feedback_map.empty()) {

      for (auto& feedback:feedback_map) {
        std::string axis_name = feedback.first;
        ActuatorInfo::feedback_s feedback_data = feedback.second;
        logger_->info("Feedback: seq_id: {}, ts_msecs: {}, axis: {},  meas_vel:"
                      " {}, abs_ticks: {}", feedback_data.seq_id ,
                      feedback_data.timestamp.msecs, axis_name,
                      feedback_data.meas_velocity, feedback_data.abs_ticks);
        if(axis_name == "left_wheel")
        {
          // Publish feedback on ROS
          nav_msgs::Odometry odom_message = nav_msgs::Odometry();
          odom_message.twist.twist.linear.x = feedback_data.meas_velocity;
          odom_message.header.seq = feedback_data.seq_id;
          odom_message.header.stamp = ros::Time::now();
          odom_message.header.frame_id = "map";
          odom_pub.publish(odom_message);

        }
      }

    } else {
      logger_->info("Feedback Empty !! Suspending read thread for {} "
                    "milliseconds",feedback_interval);
      std::this_thread::sleep_for(std::chrono::milliseconds(feedback_interval));
    }

  }

}

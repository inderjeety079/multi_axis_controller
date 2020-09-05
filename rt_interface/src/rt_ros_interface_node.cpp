
#include "ros_interface.hpp"
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <rt_interface/IPU.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <thread>
#include <signal.h>
#include <chrono>
#include "config_parser.hpp"

#include <spdlog/spdlog.h>
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/async.h"


volatile sig_atomic_t flag = 0;
void signal_event(int sig) {
    flag = 1;
}


int main(int argc, char *argv[]) {

  ros::init(argc, argv, "rt_ros_interface");
  ros::NodeHandle rt_ros_interface_nh;


  int main_thread_period = 50; //ms
  signal(SIGINT, signal_event);

  spdlog::init_thread_pool(8192, 1);
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt >();
  console_sink->set_level(spdlog::level::info);
  auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("/home/indraneel.p/greyorange_ws/proto_test.txt", 1024*1024*100, 3);
  rotating_sink->set_level(spdlog::level::debug);
  std::vector<spdlog::sink_ptr> sinks {console_sink, rotating_sink};
  auto root_logger = std::make_shared<spdlog::async_logger>("butler_control_interface", sinks.begin(), sinks.end(), spdlog::thread_pool(), spdlog::async_overflow_policy::block);
  root_logger->set_level(spdlog::level::debug);
  spdlog::register_logger(root_logger);

  // Create a feedback logger 
  auto rotating_sink_2 = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("/home/indraneel.p/greyorange_ws/proto_feedback_test.txt", 1024*1024*100, 3);
  rotating_sink_2->set_level(spdlog::level::debug);
  std::vector<spdlog::sink_ptr> sinks_2 {console_sink, rotating_sink_2};
  auto root_logger_2 = std::make_shared<spdlog::async_logger>("butler_feedback_interface", sinks_2.begin(), sinks_2.end(), spdlog::thread_pool(), spdlog::async_overflow_policy::block);
  root_logger_2->set_level(spdlog::level::debug);
  spdlog::register_logger(root_logger_2);

  spdlog::flush_every(std::chrono::seconds(1));



  std::string config_path = "/home/indraneel.p/greyorange_ws/src/rt_interface/rt_interface/config/robot_hardware_config.json";
  //"/opt/nav_ws/src//rt_interface/config/robot_hardware_config.json";;



  auto logger = spdlog::get("butler_control_interface")->clone("main_thread_logger");

  logger->debug("Creating Config Parser");
  ConfigParser::get_unique_instance(config_path);
  logger->debug("Config Parser Created");

  RosInterface ros_interface(config_path,rt_ros_interface_nh);

 ros_interface.cmd_vel_subs = rt_ros_interface_nh.subscribe(
      "/cmd_vel", 1, &RosInterface::cmd_vel_callback, &ros_interface);

  ros::AsyncSpinner spinner(1);  // Use 1 threads

  spinner.start();
  logger->info("Started Spinner");

  bool send_config_to_rt = true;
  bool rt_interface_available = false;

  while(rt_ros_interface_nh.ok() && (!flag)) {
    /* check for connections */

    if (send_config_to_rt && ros_interface.get_rt_iface_status()) {
      rt_interface_available = true;
      ros_interface.on_init();
    }

    if (ros_interface.get_rt_iface_status()) {
      send_config_to_rt = false;
    } else {
        if (rt_interface_available) {
          logger->info("Joining Listener Threads");
          ros_interface.join_rt_listener_threads();
          rt_interface_available = false;
          send_config_to_rt = true;
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(main_thread_period));
  }

  if(1 == flag ) {
      ros_interface.keep_alive_ = false;
      ros_interface.cmd_available_ = true;
      ros_interface.new_cmd_cv_.notify_one();

      RtInterface::rt_interface_u_ptr_t rt_interface_ptr;
      ros_interface.get_rt_interface(&rt_interface_ptr);

      rt_interface_ptr->set_rt_conn_status(false);
      rt_interface_ptr->notify_one_rt_conn_cv();
      logger->info("Received Keyboard Interrupt. Shutting Down the node");

  }

  ros::waitForShutdown();


  return 0;
}

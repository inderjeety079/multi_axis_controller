#ifndef PATH_FEEDBACK_H
#define PATH_FEEDBACK_H

#include "GlobalMap.h"
#include "RTInterface.h"
#include "rt_interface/IPU.h"
#include "rt_interface/encoder.h"
#include "rt_interface/heading.h"
#include <condition_variable>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <thread>

#define RAD2DEGREE(x) ((x * 180) / 3.14)
#define DEGREE2RAD(x) ((x * 3.14) / 180)

class PathFeedback : public GlobalMap {
private:
  std::mutex mutexLock;
  // std::thread PublishTfThread;
  std::condition_variable tfDataCondition;

  std::thread UpdateIPUDataThread;
  std::condition_variable IPUDataCondition;
  ipu_data_ ipu_data;
  bool new_ipu_data_available;
  geometry_msgs::Pose2D ipu_ground_truth;
  encoder_data_ encoder_ticks;
  encoder_data_ encoder_ticks_prev;
  butler_position_ butler_position;

  tf::TransformBroadcaster odom_broadcaster;
  bool time_to_publish_tf;
  heading_direction_e heading_direction;

public:
  ros::Subscriber IPU_msgs_subs;
  ros::Subscriber encoder_msg_subs;
  ros::Publisher heading_msg_pub;
  ros::Publisher butler_odom_pub;
  ros::Timer tf_publish_timer;

  PathFeedback();
  ~PathFeedback();
  void PublishTf();
  void UpdateIPUData();
  void ipu_msg_cb(const rt_interface::IPU &ipu_msg);
  void tf_publish_cb(const ros::TimerEvent &event);
  void encoder_msg_cb(const rt_interface::encoder &encoder_msg);
};
#endif

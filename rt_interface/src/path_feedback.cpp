#include "ButlerSpecs.h"
#include "GlobalMap.h"
#include "PathFeedback.h"
#include "RTInterface.h"
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <math.h>
#include <mutex>

//#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <thread>

/**
 * [PathFeedback::PathFeedback description]
 */
PathFeedback::PathFeedback()
    : butler_position({0}), encoder_ticks({0}), encoder_ticks_prev({0}),
      new_ipu_data_available(false), time_to_publish_tf(false),
      UpdateIPUDataThread(&PathFeedback::UpdateIPUData, this) {}

PathFeedback::~PathFeedback() {
  PublishTfThread.join();
  UpdateIPUDataThread.join();
}

void PathFeedback::PublishTf() {
  while (true) {
    std::mutex mutexLock;
    std::unique_lock<std::mutex> lock(mutexLock);
    heading_direction_e heading_direction;
    // ROS_INFO("PublishTf Waiting\n");
    tfDataCondition.wait(lock, [this] { return time_to_publish_tf; });
    ros::Time current_time = ros::Time::now();
    static double prev_odom_calc_time = 0.0f;
    double dt = (current_time.toSec() - prev_odom_calc_time);

    double dist_left = ENCTOLEN(encoder_ticks.left - encoder_ticks_prev.left);

    double dist_right =
        ENCTOLEN(encoder_ticks.right - encoder_ticks_prev.right);

    // ROS_INFO("Dist left:%f, right:%f", dist_left, dist_right);

    double dist_x = (dist_left + dist_right) / 2;
    double th_in_dt = (dist_right - dist_left) / WHEEL_BASE;

    double vx = dist_x / dt;
    double vth = th_in_dt / dt;

    double x_in_dt = cos(th_in_dt) * dist_x;
    double y_in_dt = sin(th_in_dt) * dist_x;

    switch (heading_direction) {

    case HEADING_DIR_0:
      butler_position.x += (cos(butler_position.heading) * x_in_dt -
                            sin(butler_position.heading) * y_in_dt);
      butler_position.y += (sin(butler_position.heading) * x_in_dt +
                            cos(butler_position.heading) * y_in_dt);
      butler_position.heading += th_in_dt;
      break;

    case HEADING_DIR_90:
      butler_position.y += (cos(butler_position.heading) * x_in_dt -
                            sin(butler_position.heading) * y_in_dt);
      butler_position.x += (sin(butler_position.heading) * x_in_dt +
                            cos(butler_position.heading) * y_in_dt);
      butler_position.heading += th_in_dt;
      break;

    case HEADING_DIR_180:
      butler_position.x += (cos(butler_position.heading) * x_in_dt -
                            sin(butler_position.heading) * y_in_dt);
      butler_position.y += (sin(butler_position.heading) * x_in_dt +
                            cos(butler_position.heading) * y_in_dt);
      butler_position.heading += th_in_dt;
      break;

    case HEADING_DIR_270:

      butler_position.y += (cos(butler_position.heading) * x_in_dt -
                            sin(butler_position.heading) * y_in_dt);
      butler_position.x += (sin(butler_position.heading) * x_in_dt +
                            cos(butler_position.heading) * y_in_dt);
      butler_position.heading += th_in_dt;
      break;

    default:
      ROS_INFO("Unknown Heading Direction %d", heading_direction);
    }
    // TODO: Verify this

    ROS_INFO("Butler Position(ENC): X = %f\tY = %f\tHeading = %f\n",
             butler_position.x, butler_position.y,
             RAD2DEGREE(butler_position.heading));

    geometry_msgs::Quaternion odom_quat =
        tf::createQuaternionMsgFromYaw(butler_position.heading);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.y = butler_position.y;
    odom_trans.transform.translation.x = butler_position.x;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    // set the position
    odom.pose.pose.position.x = butler_position.x;
    odom.pose.pose.position.y = butler_position.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = vth;

    // publish the message
    butler_odom_pub.publish(odom);

    encoder_ticks_prev.left = encoder_ticks.left;
    encoder_ticks_prev.right = encoder_ticks.right;

    prev_odom_calc_time = current_time.toSec();

    time_to_publish_tf = false;
    tfDataCondition.notify_one();
    // ROS_INFO("PublishTf Executed\n");
  }
}

/**
 * [PathFeedback::UpdateIPUData description]
 */
void PathFeedback::UpdateIPUData() {
  while (true) {

    std::mutex mutexLock;
    std::unique_lock<std::mutex> lock(mutexLock);
    ROS_INFO("Update IPU Waiting\n");
    IPUDataCondition.wait(lock, [this] { return new_ipu_data_available; });
    // TODO: Find butler global position with IPU data
    
    //GetGlobalPosition(ipu_data.row, ipu_data.column);

    ipu_ground_truth.heading = DEGREE2RAD(ipu_data.heading);
    ipu_ground_truth.x = global_position.x + (ipu_data.deltaX / 100);
    ipu_ground_truth.y = global_position.y + (ipu_data.deltaY / 100);
    new_ipu_data_available = false;
    IPUDataCondition.notify_one();
  }
}

/**
 * [PathFeedback::ipu_msg_cb description]
 * @param ipu_msg [description]
 */
void PathFeedback::ipu_msg_cb(const rt_interface::IPU &ipu_msg) {
  ipu_data.row = ipu_msg.marker_row_id;
  ipu_data.column = ipu_msg.marker_col_id;
  ipu_data.deltaX = ipu_msg.deltaX;
  ipu_data.deltaY = ipu_msg.deltaY;
  ipu_data.heading = ipu_msg.heading;

  // ROS_INFO("IPU Data: row : %d\tcol: %d\tdX:%f\tdY=%f\tHeading=%f\n",
  //          ipu_data.row, ipu_data.column, ipu_data.deltaX, ipu_data.deltaY,
  //          ipu_data.heading);
  new_ipu_data_available = true;
  IPUDataCondition.notify_one();
}

/**
 * [PathFeedback::tf_publish_cb description]
 * @param event [description]
 */
void PathFeedback::tf_publish_cb(const ros::TimerEvent &event) {
  time_to_publish_tf = true;
  tfDataCondition.notify_one();
  // ROS_INFO("Timer Triggered\n");
}

/**
 * [PathFeedback::encoder_msg_cb description]
 * @param encoder_msg [description]
 */
void PathFeedback::encoder_msg_cb(const rt_interface::encoder &encoder_msg) {
  encoder_ticks.left = encoder_msg.left;
  encoder_ticks.right = encoder_msg.right;
  // ROS_INFO("Encoder Data : left = %d\tright = %d\n", encoder_ticks.left,
  //      encoder_ticks.right);
}

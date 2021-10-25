/*
* Copyright (C) Evocortex GmbH - All Rights Reserved
*
* Unauthorized copying of this file, via any medium is strictly prohibited
* Proprietary and confidential
*
* Written by Dr. Christian Pfitzner <christian.pfitzner@evocortex.com>
* 05 2019
*
* Migrated to ROS2 by Dominik Pfeiffer <dominik.pfeiffer@siemens.com>
* 05 2020
*
* Improved by Benjamin Schadde <benjamin.schadde@siemens.com>
* 11 2020
*/

#ifndef MICRODRIVES_H
#define MICRODRIVES_H


// std includes
#include <array>
#include <string>
#include <iostream>

// ros includes
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "rclcpp/logger.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>

#define SIZE_OF_POSE_ARRAY 3

enum twist {linear_x = 0, linear_y, angular_z};
enum joy {honk = 0, power_enable, reset, overwrite_safety, move_up, move_down, change_led_mode};


/**
 * @namespace to define constants for this class
 */
namespace Constants
{
  constexpr auto nr_of_drives    = 4; 
  constexpr auto byte_array_size = 10; 
}

/**
 * @enum MicroDriveSignal
 */
enum MicroDriveSignal
{
  ENABLE         = 1,
  ENABLE2        = 2,
  LIFT_UP        = 4, 
  LIFT_DOWN      = 8,
  HONK           = 16,
  BLINK_LEFT     = 32,
  CHANGE_LED_MODE   = 64,
  DISABLE_SAFETY = 128, 
};

/**
 * @@struct PoseOdometryFeedback
 * @author  Christian Pfitzner 
 */
struct PoseOdometryFeedback
{
  double x = 0.0;     // position in x in meters
  double y = 0.0;     // position in y in meters
  double yaw = 0.0;   // yaw angle in rad
  double roll = 0.0;
  double pitch = 0.0;
}; 

/**
 * @struct MicroDriveMsg
 * 
 */
class MicroDriveMsg
{
  
public:
  void updatePose(float (&pose_array)[SIZE_OF_POSE_ARRAY])
  {
    // update for the pose from PLC
    // transform pose_x from mm to m
    _pose_feedback.x = (double)pose_array[twist::linear_x] / 1000.0;

    // transform pose_y from mm to m
    _pose_feedback.y = (double)pose_array[twist::linear_y] / 1000.0;

    // update angular_z in rad
    _pose_feedback.yaw = (double)pose_array[twist::angular_z];
  }


  nav_msgs::msg::Odometry getOdometryFeedback(const double cycletime) 
  {
    tf2::Quaternion myQuaternion;

    double passed_time;
    double starttime; 
    static double endtime = 0;
    // static auto seq = 0;   unused 
    nav_msgs::msg::Odometry               odom_feedback;
    using builtin_interfaces::msg::Time;
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    Time current_time = ros_clock.now();

    starttime = (current_time.sec+current_time.nanosec*0.000000001);
    passed_time = starttime-endtime;


    odom_feedback.header.frame_id = "odom";
    //odom_feedback.header.seq      = seq++; 
    odom_feedback.header.stamp    = current_time; 

    // set the values for position 
    odom_feedback.pose.pose.position.x    = _pose_feedback.x;
    odom_feedback.pose.pose.position.y    = _pose_feedback.y;
    odom_feedback.pose.covariance[0]      = 0.1;
    odom_feedback.pose.covariance[7]      = 0.1;
    odom_feedback.pose.covariance[35]     = 0.1;

    myQuaternion.setRPY( _pose_feedback.roll, _pose_feedback.pitch, _pose_feedback.yaw);
    odom_feedback.pose.pose.orientation   = tf2::toMsg(myQuaternion); 


    // set the values for velocity
    double v_x   = 0; 
    double v_y   = 0; 
    double v_yaw = 0;

    if(endtime != 0)
    {
      v_x   = (_old_pose_feedback.x   - _pose_feedback.x)   / passed_time; 
      v_y   = (_old_pose_feedback.y   - _pose_feedback.y)   / passed_time; 
      v_yaw = (_old_pose_feedback.yaw - _pose_feedback.yaw) / passed_time; 
    }

    odom_feedback.twist.twist.linear.x    = v_x;
    odom_feedback.twist.twist.linear.y    = v_y;
    odom_feedback.twist.twist.angular.z   = v_yaw; 

    // save the odometry to calculate the velocity
    _old_pose_feedback = _pose_feedback;

    using builtin_interfaces::msg::Time;
    Time end_time = ros_clock.now();
    endtime=(end_time.sec+end_time.nanosec*0.000000001);
    return odom_feedback; 
  }

  private: 
    std::array<int, Constants::nr_of_drives> _rpm     = {{0, 0, 0, 0}};

    PoseOdometryFeedback    _pose_feedback;       //!< feedback of the pose from the controller
    PoseOdometryFeedback    _old_pose_feedback;   //!< feedback of the pose from the controller
};


#endif // MICRODRIVEMSG




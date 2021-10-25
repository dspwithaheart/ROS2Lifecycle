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


// ros includes
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rate.hpp>


// std cpp include
#include <stdlib.h>

// local includes
#include "Chassis.h"
#include "SWCPU_data_exchange.h"

using std::placeholders::_1;



// Initializer the subscribers for this node
// ros::Subscriber _lift_sub;      //!< subscriber for the lifting message
// _twist_sub = node->create_subscription();
// _joy_sub = node->create_subscription();
// _honk_sub = node->create_subscription();
 

// parameter for the kinematics
MicroDriveMsg   _drives_msg;                      //!< message container for the communication with the drives
Chassis         _chassis;                         //!< parameters for the chassis

double twist_msg_[SIZE_OF_TWIST_MSG];
bool joy_msg_[SIZE_OF_JOY_MSG];

bool _honk_active   = false; 
bool _always_enable = false;

class PlcNode : public rclcpp::Node
{
  public:
    PlcNode()
    : Node("plc_node")
    {
      _twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "/twist_mux/cmd_vel", 1, std::bind(&PlcNode::twistCallback, this, _1));

    _joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy_converted", 1, std::bind(&PlcNode::joyCallback, this, _1));

    _honk_sub = this->create_subscription<std_msgs::msg::Int16>(
      "/honk", 1, std::bind(&PlcNode::honkCallback, this, _1));

    _odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 1);
    }
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;

  private:
    /**
     * Callback function for joystick
     * @param msg
     */
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr twist);
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _twist_sub;

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_sub;
    
    void honkCallback(const std_msgs::msg::Int16::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr _honk_sub;
};


void PlcNode::twistCallback(const geometry_msgs::msg::Twist::SharedPtr twist)
{
  twist_msg_[twist::linear_x] = static_cast<double>(twist->linear.x) * 1000.0;       // mm per second
  twist_msg_[twist::linear_y] = static_cast<double>(twist->linear.y) * 1000.0;       // mm per second
  twist_msg_[twist::angular_z] = static_cast<double>(twist->angular.z) * 1000.0;       // mrad per second
}


void PlcNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  //map for all buttons and axes of the PS4 Controller
  //hor -> horizontal (left and right), ver -> vertical (up and down)

  //mapping of joy_converter
  //Todo - get mapping for controller from joy_converter/joy_enum.h
  enum joy_axes
  {
    left_stick_hor = 0,
    left_stick_ver,
    right_stick_hor,
    axes_l2,
    axex_r2,
    right_stick_ver,
    arrow_hor,
    arrow_ver,
    amount_of_axes
  };
  enum joy_buttons
  {
    square = 0,
    cross,
    circle,
    triangle,
    l1,
    r1,
    l2,
    r2,
    share,
    options,
    left_stick,
    right_stick,
    ps_symbol,
    touchpad,
    amount_of_buttons
  };

  //clear joy message
  memset(joy_msg_, false, SIZE_OF_JOY_MSG);

  //update joy message
  if(joy_msg->buttons[joy_buttons::l1] == true || _honk_active)
    joy_msg_[joy::honk] = true;


  if(joy_msg->buttons[joy_buttons::r1] == true || _always_enable == true)
    joy_msg_[joy::power_enable] = true;       //always enable the movement of the robot


  if(joy_msg->buttons[joy_buttons::r2] == true)
    joy_msg_[joy::overwrite_safety] = true;   //always enable the movement of the robot


  if(joy_msg->buttons[joy_buttons::options] == true)
    joy_msg_[joy::change_led_mode] = true;    //change led mode


  if(joy_msg->buttons[joy_buttons::share] == true)
    joy_msg_[joy::reset] = true;


  if(joy_msg->axes[joy_axes::arrow_ver] > 0)
  {
    joy_msg_[joy::move_up] = true;            //lift the robot
  }
  else if (joy_msg->axes[joy_axes::arrow_ver] < 0)
  {
    joy_msg_[joy::move_down] = true;          //lower the robot
  }
}


void PlcNode::honkCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  _honk_active = msg->data; 
}


int main(int argc, char **argv)
{
  //create object of class SwcpuDataExchange
  SwcpuDataExchange rib;

  //broadcaster for odometry
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // initialize ros node
  rclcpp::init(argc, argv);     // "plc_interface_node");
  RCLCPP_INFO(rclcpp::get_logger("PlcNode"), "Starting to communication to PLC");

  auto node = std::make_shared<PlcNode>();

  // set up parameters of chassis
  _chassis.setMs2Rrpm(1500);                // set the conversion factor between rpm and meters per second
  _chassis.setTrack(0.273);                 // set the distance between the left and the right wheel
  _chassis.setWheelbase(0.545);             // set the distance between the front and the rear wheel

  // the loop rate is set to 100 HZ
  // you can try to set more, but this should be sufficient
  rclcpp::Rate loop_rate(100);

  //kill node, if RIB didn't connect successfully
  if(!rib.init())
  {
    rib.die();
    return 0;
  }

  //use this executor to prevent throwing exception after SIGINT -> https://github.com/ros2/rclcpp/issues/1066
  rclcpp::executors::SingleThreadedExecutor ros_executor;
  while(rclcpp::ok())
  {
    //start timer
    loop_rate.reset();

    //send commands to PLC
    rib.send(twist_msg_, joy_msg_);

    //read values from PLC
    MicroDriveMsg drives_feedback = rib.read();

    //publish the odometry
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    geometry_msgs::msg::TransformStamped odom_trans;
    nav_msgs::msg::Odometry odom_msg = drives_feedback.getOdometryFeedback(loop_rate.period().count());
    odom_msg.child_frame_id = "base_link";

    //publish ros message
    node->_odom_pub->publish(odom_msg);

    odom_trans.header.frame_id = odom_msg.header.frame_id;
    odom_trans.header.stamp = odom_msg.header.stamp;
    odom_trans.child_frame_id = odom_msg.child_frame_id;
    odom_trans.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_trans.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_trans.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(odom_trans);

    //spin ros to update callback functions
    ros_executor.spin_node_some(node);

    //sleep until loop rate is reached
    loop_rate.sleep();
  }

  //close RIB connection, when node is terminated
  rib.die();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down");         // info to the user

  return 0;
}

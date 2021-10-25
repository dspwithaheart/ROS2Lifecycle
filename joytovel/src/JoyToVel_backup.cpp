/*
 * JoyToVel.cpp
 *
 *  Created: Jul, 2020
 *      Author: Dominik Pfeiffer <dominik.pfeiffer@siemens.com>
 */

#include <numeric>
#include "JoyToVel.hpp"


namespace evo {


JoyToVel::JoyToVel() : Node("joy_to_vel")
{

  // init subscribers
  _joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy_converted", 10, std::bind(&JoyToVel::joyCallback, this, _1));

  // init publishers
  _vel_pub       = this->create_publisher<geometry_msgs::msg::Twist>("/joy_vel", 10);

  subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10, std::bind(&JoyToVel::topic_callback, this, _1));

  timer_ = create_wall_timer(
    100ms, std::bind(&JoyToVel::timer_callback, this));

  joystop_ = this->create_publisher<geometry_msgs::msg::Twist>("/joy_vel", 10);

  _enable_client = this->create_client<std_srvs::srv::SetBool>("enable_client");

  _disable_client = this->create_client<std_srvs::srv::SetBool>("disable_client");

}

void JoyToVel::topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
{
  //Joy Time
  builtin_interfaces::msg::Time joy_now;
  joy_now=msg->header.stamp;
  joytime=joy_now.sec+(joy_now.nanosec*0.000000001);
}

void JoyToVel::timer_callback()
{  
  //Ros System Time
  using builtin_interfaces::msg::Time;
  rclcpp::Clock ros_clock(RCL_ROS_TIME);
  Time ros_now = ros_clock.now();
  rostime=ros_now.sec+(ros_now.nanosec*0.000000001);
  duration=(rostime-joytime);
  // printf("rostime: %lf, joytime: %lf \n", rostime, joytime);

  if(duration>0.15)
  {
    _out_of_sync = true;
    RCLCPP_WARN(this->get_logger(), "Emergency! Connection to controller lost!");
    geometry_msgs::msg::Twist joy_disconnect = geometry_msgs::msg::Twist();
    _vel_pub->publish(joy_disconnect);
  }
  else
  {
    _out_of_sync = false; 
  }
}


void JoyToVel::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
static sensor_msgs::msg::Joy::SharedPtr msg_old = msg; 

  this->publishVel(msg);

  if(!msg->buttons[static_cast<int>(ButtonMap::dead_man_button)]) _dead_man_released = true;
  else                                                           _dead_man_released = false;


  if(!msg->buttons[static_cast<int>(ButtonMap::dead_man_slow)])   _dead_man_slow_pressed = true;
  else                                                           _dead_man_slow_pressed = false;


  this->publishVel(msg);

  if(!msg->buttons[static_cast<int>(ButtonMap::dead_man_button)]) _dead_man_released = true;
  else                                           

  // save the current one to the old one for rising trigger
  msg_old = msg; 
}


void JoyToVel::publishVel(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  static geometry_msgs::msg::Twist twist_old = geometry_msgs::msg::Twist(); 
  geometry_msgs::msg::Twist twist = geometry_msgs::msg::Twist();

  double v_x = msg->axes[static_cast<int>(AxesMap::trans_x)];
  double v_y = msg->axes[static_cast<int>(AxesMap::trans_y)];
  double r_z = msg->axes[static_cast<int>(AxesMap::rot_z)  ];

  double limit_vel = _joy_config.v_max; 
  if(!_dead_man_slow_pressed)
  {
    limit_vel = 0.4  * _joy_config.v_max; 
  }


  limit_value<double>(v_x, -limit_vel, limit_vel);
  limit_value<double>(v_y, -limit_vel, limit_vel);


  _avg_lin_x.push_front(v_x);
  _avg_lin_y.push_front(v_y);
  _avg_rot.push_front(r_z);

  v_x = std::accumulate(_avg_lin_x.begin(), _avg_lin_x.end(), 0.0) / _avg_lin_x.size();
  v_y = std::accumulate(_avg_lin_y.begin(), _avg_lin_y.end(), 0.0) / _avg_lin_y.size();
  r_z = std::accumulate(_avg_rot.begin(),   _avg_rot.end(),   0.0) / _avg_rot.size();

  _avg_lin_x.pop_back();
  _avg_lin_y.pop_back();
  _avg_rot.pop_back();

  // apply square joystick curve
  twist.linear.x  = v_x * std::abs(v_x) * _joy_config.lin_scale;
  twist.linear.y  = v_y * std::abs(v_y) * _joy_config.lin_scale;
  twist.angular.z = r_z * std::abs(r_z) * _joy_config.rot_scale;

  // check if dead man switch is configured and pressed
  if(_dead_man_released)
  {
    RCLCPP_INFO(this->get_logger(), "Dead man switch is released. The robot will stop immediatly");

    if(!_published_zero_once)
    {
      RCLCPP_INFO(this->get_logger(), "Publishing zero velocity");
      auto twist = geometry_msgs::msg::Twist(); 
      _vel_pub->publish(twist);
      //keep publishing if controller is disconnected
      if(_out_of_sync==false)
      {
      _published_zero_once = true;
      }

    }
  }
  else
  {
//    _timer_for_deadman.setPeriod(ros::Duration(2.0), true);
   _vel_pub->publish(twist);
  }

  if(!_dead_man_released)
  {
    _published_zero_once = false;
  }

  twist_old = twist; 
}


void JoyToVel::publishEnable(void)
{
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = motor::enable;

  while (!_enable_client->wait_for_service(1s)) {
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
}


void JoyToVel::publishDisable(void)
{
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = motor::disable;

  while (!_disable_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
}

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv); //"mechanum_joy_node"
  //RCLCPP_INFO(this->get_logger(),"Starting mechanum_joy_node");

  rclcpp::spin(std::make_shared<evo::JoyToVel>());
  rclcpp::shutdown();

  return 0;
}
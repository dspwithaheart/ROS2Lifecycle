/*
 * joy_converter.cpp
 *
 *  Created on: Nov 26, 2020
 *      Author: Benjamin Schadde <benjamin.schadde@siemens.com>
 */

#include "joy_converter/joy_converter.h"

joyConverter::joyConverter() : Node("joy_converter_node"),
logger_prefix_("JoyConverter"),
print_flag_(false)
{
  //subscriber
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&joyConverter::joyCallback, this, _1));
  joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy_converted", 10);
}

joyConverter::~joyConverter()
{
}

void joyConverter::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  //converted joy message (use the conversion for bluetooth connection)
  sensor_msgs::msg::Joy::SharedPtr joy_msg_converted = std::make_shared<sensor_msgs::msg::Joy>();

  //map for all buttons and axes of the PS4 Controller
  //hor -> horizontal (left and right), ver -> vertical (up and down)
  //usage with cable
  enum cable_axes_PS4
  {
    left_stick_hor_ps4_cable = 0,
    left_stick_ver_ps4_cable,
    axes_l2_ps4_cable,
    right_stick_hor_ps4_cable,
    right_stick_ver_ps4_cable,
    axes_r2_ps4_cable,
    arrow_hor_ps4_cable,
    arrow_ver_ps4_cable,
    amount_of_axes_ps4_cable
  };
  enum cable_buttons_PS4
  {
    cross_ps4_cable = 0,
    circle_ps4_cable,
    triangle_ps4_cable,
    square_ps4_cable,
    l1_ps4_cable,
    r1_ps4_cable,
    l2_ps4_cable,
    r2_ps4_cable,
    share_ps4_cable,
    options_ps4_cable,
    ps_symbol_ps4_cable,
    left_stick_ps4_cable,
    right_stick_ps4_cable,
    amount_of_buttons_ps4_cable
  };

  joy_msg_converted->axes.resize(joy_axes::amount_of_axes);
  joy_msg_converted->buttons.resize(joy_buttons::amount_of_buttons);

  //check if PS4 Controller connected with bluetooth -> no conversion necessary
  if(joy_msg->buttons.size() == joy_buttons::amount_of_buttons && joy_msg->axes.size() == joy_axes::amount_of_axes)
  {
    if(!print_flag_)
    {
      RCLCPP_INFO(rclcpp::get_logger(logger_prefix_), "PS4 Controller with bluetooth connected");
      //print this just one time
      print_flag_ = true;
    }

    joy_msg_converted = joy_msg;
  }
  //check if PS4 Controller connected with cable
  else if(joy_msg->buttons.size() == cable_buttons_PS4::amount_of_buttons_ps4_cable && joy_msg->axes.size() == cable_axes_PS4::amount_of_axes_ps4_cable)
  {
    if(!print_flag_)
    {
      RCLCPP_INFO(rclcpp::get_logger(logger_prefix_), "PS4 Controller with cable connected");
      //print this just one time
      print_flag_ = true;
    }

    //convert axes
    joy_msg_converted->axes[joy_axes::left_stick_hor]  = joy_msg->axes[cable_axes_PS4::left_stick_hor_ps4_cable];
    joy_msg_converted->axes[joy_axes::left_stick_ver]  = joy_msg->axes[cable_axes_PS4::left_stick_ver_ps4_cable];
    joy_msg_converted->axes[joy_axes::right_stick_hor] = joy_msg->axes[cable_axes_PS4::right_stick_hor_ps4_cable];
    joy_msg_converted->axes[joy_axes::right_stick_ver] = joy_msg->axes[cable_axes_PS4::right_stick_ver_ps4_cable];
    joy_msg_converted->axes[joy_axes::axes_l2]         = joy_msg->axes[cable_axes_PS4::axes_l2_ps4_cable];
    joy_msg_converted->axes[joy_axes::axex_r2]         = joy_msg->axes[cable_axes_PS4::axes_r2_ps4_cable];
    joy_msg_converted->axes[joy_axes::arrow_hor]       = joy_msg->axes[cable_axes_PS4::arrow_hor_ps4_cable];
    joy_msg_converted->axes[joy_axes::arrow_ver]       = joy_msg->axes[cable_axes_PS4::arrow_ver_ps4_cable];

    //convert buttons
    joy_msg_converted->buttons[joy_buttons::square]      = joy_msg->buttons[cable_buttons_PS4::square_ps4_cable];
    joy_msg_converted->buttons[joy_buttons::cross]       = joy_msg->buttons[cable_buttons_PS4::cross_ps4_cable];
    joy_msg_converted->buttons[joy_buttons::circle]      = joy_msg->buttons[cable_buttons_PS4::circle_ps4_cable];
    joy_msg_converted->buttons[joy_buttons::triangle]    = joy_msg->buttons[cable_buttons_PS4::triangle_ps4_cable];
    joy_msg_converted->buttons[joy_buttons::l1]          = joy_msg->buttons[cable_buttons_PS4::l1_ps4_cable];
    joy_msg_converted->buttons[joy_buttons::l2]          = joy_msg->buttons[cable_buttons_PS4::l2_ps4_cable];
    joy_msg_converted->buttons[joy_buttons::r1]          = joy_msg->buttons[cable_buttons_PS4::r1_ps4_cable];
    joy_msg_converted->buttons[joy_buttons::r2]          = joy_msg->buttons[cable_buttons_PS4::r2_ps4_cable];
    joy_msg_converted->buttons[joy_buttons::share]       = joy_msg->buttons[cable_buttons_PS4::share_ps4_cable];
    joy_msg_converted->buttons[joy_buttons::options]     = joy_msg->buttons[cable_buttons_PS4::options_ps4_cable];
    joy_msg_converted->buttons[joy_buttons::left_stick]  = joy_msg->buttons[cable_buttons_PS4::left_stick_ps4_cable];
    joy_msg_converted->buttons[joy_buttons::right_stick] = joy_msg->buttons[cable_buttons_PS4::right_stick_ps4_cable];
    joy_msg_converted->buttons[joy_buttons::ps_symbol]   = joy_msg->buttons[cable_buttons_PS4::ps_symbol_ps4_cable];
    joy_msg_converted->buttons[joy_buttons::touchpad]    = 0;                                                //is not available with cable
  }
  //no fitting conversion found -> use default
  else
  {
    if(!print_flag_)
    {
      RCLCPP_ERROR(rclcpp::get_logger(logger_prefix_), "no fitting conversion found -> use bluetooth conversion (buttons: %d | axes: %d", joy_msg->buttons.size(), joy_msg->axes.size());
      //print this just one time
      print_flag_ = true;
    }

    joy_msg_converted = joy_msg;
  }

  joy_pub_->publish(*joy_msg_converted);
}



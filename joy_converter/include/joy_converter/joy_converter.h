/*
 * joy_converter.h
 *
 *  Created on: Nov 26, 2020
 *      Author: Benjamin Schadde <benjamin.schadde@siemens.com>
 */

#ifndef JOY_CONVERTER_JOY_CONVERTER_H_
#define JOY_CONVERTER_JOY_CONVERTER_H_

//ros includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

//custom includes
#include <joy_converter/joy_enum.h>

using std::placeholders::_1;

class joyConverter : public rclcpp::Node
{
public:
  joyConverter();
  virtual ~joyConverter();

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;

  std::string logger_prefix_;
  bool print_flag_;
};

#endif /* JOY_CONVERTER_JOY_CONVERTER_H_ */

/*
 * joy_converter_node.cpp
 *
 *  Created on: Nov 27, 2020
 *      Author: Benjamin Schadde <benjamin.schadde@siemens.com>
 */

#include <joy_converter/joy_converter.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<joyConverter>());

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}



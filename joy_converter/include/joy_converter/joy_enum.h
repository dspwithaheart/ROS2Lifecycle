/*
 * joy_enum.h
 *
 *  Created on: Dec 1, 2020
 *      Author: Benjamin Schadde "benjamin.schadde@siemens.com"
 *
 *  map used for all applications listening to the /joy_converted topic
 */

#ifndef JOY_CONVERTER_INCLUDE_JOY_ENUM_H_
#define JOY_CONVERTER_INCLUDE_JOY_ENUM_H_

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



#endif /* JOY_CONVERTER_INCLUDE_JOY_ENUM_H_ */

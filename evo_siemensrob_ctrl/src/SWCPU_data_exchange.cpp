//////////////////////////////////////////////////////////////////////////////////////////
/// Communication with PLC in both directions
/// 
/// \file SWCPU_data_exchange.cpp
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
///
/// \Validate for SIMATIC OCP - Image: June 2020
///
/// \Written by Benjamin Schadde <benjamin.schadde@siemens.com>
/// \11 2020
////////////////////////////////////////////////////////////////////////////////////////// 


//*******************************************************************************
// Includes
#include "SWCPU_data_exchange.h"


SwcpuDataExchange::SwcpuDataExchange() :
rx_pose_(0),
rx_error_(0),
rx_flags_(0),
rx_safety_(0),
rib_connection_ (RIB::RibConnection::Create())
{
}

SwcpuDataExchange::~SwcpuDataExchange()
{
}

bool SwcpuDataExchange::init()
{
  //use the same string like in plc program
  const std::string rx_pose_str = "pose_array", rx_error_str = "error_array", rx_flags_str = "flags_array", rx_safety_str ="safety_array";

  rib_connection_.Connect();
  if(!rib_connection_.IsConnected())
  {
    LOG_ERR("Connection to RIB not possible");
    return false;
  }

  rib_connection_.RequestRibEnvironmentConfig();

  //request following symbols
  rib_connection_.addRequestedSymbol(rx_pose_str);
  rib_connection_.addRequestedSymbol(rx_error_str);
  rib_connection_.addRequestedSymbol(rx_flags_str);
  rib_connection_.addRequestedSymbol(rx_safety_str);


  //call send function to initialize provider with empty drive_msg
  double twist_msg[SIZE_OF_TWIST_MSG];
  bool joy_msg[SIZE_OF_JOY_MSG];

  memset(twist_msg, 0, SIZE_OF_TWIST_MSG);
  memset(joy_msg, 0, SIZE_OF_JOY_MSG);

  send(twist_msg, joy_msg);

  //Sign in to RIB
  rib_connection_.SignIn();
  if(!rib_connection_.IsSignedIn())
  {
    LOG_ERR("Sign in to RIB not possible");
    rib_connection_.Disconnect();
    return false;
  }

  //*******************************************************************************
  // Consumer creates a symbol image of requested data
  symbol_image_ = rib_connection_.getOrCreateSymbolImageForAllConnectedLifeTimeBuffers().lock();

  // Get Address of requested symbols
  // Access to the symbols and casting to a specific data type
  // getPointerToSymbol => Returns the information where the value for the symbol "rx_msg" is located
  rx_pose_ = reinterpret_cast<float*>(symbol_image_->getPointerToSymbol(rx_pose_str));
  rx_error_ = reinterpret_cast<int8_t*>(symbol_image_->getPointerToSymbol(rx_error_str));
  rx_flags_ = reinterpret_cast<int8_t*>(symbol_image_->getPointerToSymbol(rx_flags_str));
  rx_safety_ = reinterpret_cast<int8_t*>(symbol_image_->getPointerToSymbol(rx_safety_str));

  return true;
}


void SwcpuDataExchange::send(double (&twist_msg)[SIZE_OF_TWIST_MSG], bool (&joy_msg)[SIZE_OF_JOY_MSG])
{
  //*******************************************************************************
  // Create a memory area called "SWCPU_DataExchange" for provided symbols
  const std::string provided_memory_name = "SWCPU_DataExchange";
  const std::string app_description = "SWCPU_DataExchange is a Template App for bidirectional data exchange";
  const std::string app_version = "V0.2";
  constexpr int signal = 10;
  constexpr uint64_t cycle_time_in_usec = 500;

  ProvidedData provided_data;

  // Linux Provider creates Shared Memory with details from above
  static RIB::ConsistentDataTransferWriter transfer_area = rib_connection_.addLifetimeBuffer(provided_memory_name,
      app_description,
      app_version,
      signal,
      provided_data,
      cycle_time_in_usec,
      sizeof(provided_data));

  // Provider updates its Shared Memory
  provided_data.linear_x_  = twist_msg[twist::linear_x];
  provided_data.linear_y_  = twist_msg[twist::linear_y];
  provided_data.angular_z_ = twist_msg[twist::angular_z];

  provided_data.honk_             = (int8_t)joy_msg[joy::honk];
  provided_data.power_enable_     = (int8_t)joy_msg[joy::power_enable];
  provided_data.reset_            = (int8_t)joy_msg[joy::reset];
  provided_data.overwrite_safety_ = (int8_t)joy_msg[joy::overwrite_safety];
  provided_data.move_up_          = (int8_t)joy_msg[joy::move_up];
  provided_data.move_down_        = (int8_t)joy_msg[joy::move_down];
  provided_data.change_led_mode_  = (int8_t)joy_msg[joy::change_led_mode];

  // Update provided symbol values
  transfer_area.writeUserData(&provided_data);

  return;
}


MicroDriveMsg SwcpuDataExchange::read()
{
  float rx_pose_array[SIZE_OF_POSE_ARRAY];
  bool rx_error_array[SIZE_OF_ERROR_ARRAY], rx_flags_array[SIZE_OF_FLAGS_ARRAY], rx_safety_array[SIZE_OF_SAFETY_ARRAY];

  //read user data from shared memory
  symbol_image_->update();

  //copy array from shared memory to local variable

  //get the pose linear_x and linear_y in mm, angular_z in rad from PLC
  for(uint8_t i = 0; i < SIZE_OF_POSE_ARRAY; i++)
  {
    rx_pose_array[i] = float(*(rx_pose_ + i));
  }

  //Todo - The following arrays will be sent by PLC but not used on Linux side
  for(uint8_t i = 0; i < SIZE_OF_ERROR_ARRAY; i++)
  {
    rx_error_array[i] = bool(*(rx_error_ + i));
  }

  for(uint8_t i = 0; i < SIZE_OF_FLAGS_ARRAY; i++)
  {
    rx_flags_array[i] = bool(*(rx_flags_ + i));
  }

  for(uint8_t i = 0; i < SIZE_OF_SAFETY_ARRAY; i++)
  {
    rx_safety_array[i] = bool(*(rx_safety_ + i));
  }

  MicroDriveMsg msg;
  msg.updatePose(rx_pose_array);

  return msg;
}


void SwcpuDataExchange::die()
{
  //*******************************************************************************
  // SignOut & Disconnect
  rib_connection_.SignOut();
  rib_connection_.Disconnect();

  return;
}

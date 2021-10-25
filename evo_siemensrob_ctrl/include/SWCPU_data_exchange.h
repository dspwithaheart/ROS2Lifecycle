//////////////////////////////////////////////////////////////////////////////////////////
/// Communication with PLC in both directions
///
/// \file SWCPU_data_exchange.h
///
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
///
/// \Validate for SIMATIC OCP - Image: June 2020
///
/// \Written by Benjamin Schadde <benjamin.schadde@siemens.com>
/// \11 2020
//////////////////////////////////////////////////////////////////////////////////////////

#ifndef SWCPU_DATA_EXCHANGE_H
#define SWCPU_DATA_EXCHANGE_H

#include "ribConnection.h"
#include "log_datalayer.h"
#include "consistentDataTransferWriter.h"
#include "symbolImage.h"
#include "dataExchangeBuffer.h"
#include "ribConstants.h"
#include "cycleTimer.h"
#include <chrono>
#include <signal.h>
#include <sys/mman.h>
#include <memory>

#include "versionInfo.h"
#include <iostream>
#include <unistd.h>
#include <string.h>

#include "MicroDriveMsg.h"

//provided arrays
#define SIZE_OF_TWIST_MSG 3
#define SIZE_OF_JOY_MSG 7

//requested arrays
#define SIZE_OF_ERROR_ARRAY 3
#define SIZE_OF_FLAGS_ARRAY 5
#define SIZE_OF_SAFETY_ARRAY 2

class SwcpuDataExchange
{
public:
  SwcpuDataExchange();
  virtual ~SwcpuDataExchange();

  //initialize RIB connection with PLC
  bool init();

  //send values to PLC
  void send(double (&twist_msg)[SIZE_OF_TWIST_MSG], bool (&joy_msg)[SIZE_OF_JOY_MSG]);

  //read values from PLC
  MicroDriveMsg read();

  //close RIB connection with PLC
  void die();

private:

  //RIB Connection
  RIB::RibConnection rib_connection_;

  //Consumer
  std::shared_ptr<RIB::SymbolImage> symbol_image_;
  float *rx_pose_;
  int8_t *rx_error_;
  int8_t *rx_flags_;
  int8_t *rx_safety_;
};

#endif /* SWCPU_DATA_EXCHANGE_H */

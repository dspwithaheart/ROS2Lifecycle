//////////////////////////////////////////////////////////////////////////////////////////
/// Class to match providing and corresponding consuming apps
/// 
/// \file dataExchangeBuffer.h
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
///
/// \Validate for SIMATIC OCP - Image: June 2020
/// \Written by Benjamin Schadde <benjamin.schadde@siemens.com>
/// \11 2020
//////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "IRIBBaseDataStruct.h"
#include <string>
#include <list>

// Structure for data exchange
struct ProvidedData : public IRIBBaseDataStruct
{
public:

    //provided messages
    double linear_x_;
    double linear_y_;
    double angular_z_;

    int8_t honk_;
    int8_t power_enable_;
    int8_t reset_;
    int8_t overwrite_safety_;
    int8_t move_up_;
    int8_t move_down_;
    int8_t change_led_mode_;


    std::list<RIB::Symbol> getSymbolList() override
    {
        std::list<RIB::Symbol> symbolList;

        //push back provided messages:    topic_name, data type, size of msg,       Offset
        symbolList.push_back(RIB::Symbol("linear_x",  "double", sizeof(linear_x_),  OFFSET_OF_VARIABLE_IN_STRUCT(linear_x_)));
        symbolList.push_back(RIB::Symbol("linear_y",  "double", sizeof(linear_y_),  OFFSET_OF_VARIABLE_IN_STRUCT(linear_y_)));
        symbolList.push_back(RIB::Symbol("angular_z", "double", sizeof(angular_z_), OFFSET_OF_VARIABLE_IN_STRUCT(angular_z_)));

        symbolList.push_back(RIB::Symbol("honk",             "int8_t", sizeof(honk_),             OFFSET_OF_VARIABLE_IN_STRUCT(honk_)));
        symbolList.push_back(RIB::Symbol("power_enable",     "int8_t", sizeof(power_enable_),     OFFSET_OF_VARIABLE_IN_STRUCT(power_enable_)));
        symbolList.push_back(RIB::Symbol("reset",            "int8_t", sizeof(reset_),            OFFSET_OF_VARIABLE_IN_STRUCT(reset_)));
        symbolList.push_back(RIB::Symbol("overwrite_safety", "int8_t", sizeof(overwrite_safety_), OFFSET_OF_VARIABLE_IN_STRUCT(overwrite_safety_)));
        symbolList.push_back(RIB::Symbol("move_up",          "int8_t", sizeof(move_up_),          OFFSET_OF_VARIABLE_IN_STRUCT(move_up_)));
        symbolList.push_back(RIB::Symbol("move_down",        "int8_t", sizeof(move_down_),        OFFSET_OF_VARIABLE_IN_STRUCT(move_down_)));
        symbolList.push_back(RIB::Symbol("change_led_mode",  "int8_t", sizeof(change_led_mode_),  OFFSET_OF_VARIABLE_IN_STRUCT(change_led_mode_)));

        return symbolList;
    }
};

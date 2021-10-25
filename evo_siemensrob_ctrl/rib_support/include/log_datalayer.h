//////////////////////////////////////////////////////////////////////////////////////////
/// Logging of data to standard output
/// 
/// \file log_datalayer.h
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
/// 
/// \author Team OCP
///
//////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <string>

///
/// Enum for log category
///
enum LOG_CAT
{
    STANDARD,
    DEBUG,
    ERROR
};

/// possible LOG_LEVELs 
/// compile time decission
/// 2 - debug (everything)
/// 1 - log
/// 0 - error
#ifndef LOG_LEVEL
#define LOG_LEVEL 2
#endif

#if (LOG_LEVEL > 0)
///
/// Logging of given data as STANDARD log
/// \param msg log message
/// \return 0
///
#define LOG(msg)    log(STANDARD, __FILE__, __func__, __LINE__, msg)
#else
#define LOG(msg)      
#endif

#if (LOG_LEVEL > 1 )
///
/// Logging of given data as DEBUG log
/// \param msg log message
/// \return 0
///
#define LOG_DBG(x)  log(DEBUG, __FILE__, __func__, __LINE__, x)
#else
#define LOG_DBG(x)
#endif

///
/// Logging of given data as ERROR log
/// \param msg log message
/// \return 0
///
#define LOG_ERR(x)  log(ERROR, __FILE__, __func__, __LINE__, x)

///
/// Logging of given data
/// \param category log level for printing message
/// \param file file name of source
/// \param function function name
/// \param line line of code
/// \param msg log message
///
void log(const LOG_CAT category, const std::string file, const std::string function, const int line, const std::string msg);

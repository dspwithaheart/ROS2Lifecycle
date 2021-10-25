//////////////////////////////////////////////////////////////////////////////////////////
/// Contains the implementation of the RibException class 
/// for handling of specific exceptions when working with applications connected to RIB.
/// 
/// \file ribException.h
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
/// 
/// \author Team OCP
///
//////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <exception>
#include <string>
#include <iosfwd>

namespace RIB
{
    class RIBException : public std::exception
    {
    private:
        std::string m_exceptionText; //TODO : It should be reference
        std::string m_responseValue;
    public:
        RIBException(const std::string& text, const std::string& value = "");
        virtual ~RIBException() = default;
        std::string getResponseValue() const;
        const char* what() const noexcept override;
    };
}

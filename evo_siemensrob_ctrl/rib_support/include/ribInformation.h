//////////////////////////////////////////////////////////////////////////////////////////
// TODO  missing description of header file
/// 
/// \file ribInformation.h
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
/// 
/// \author Team OCP
///
//////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <string>

namespace RIB
{
    class RibInformation
    {
    public:
        RibInformation(const int pid, const std::string& version, const std::string& result, const std::string& errorMessage);
        ~RibInformation() = default;

        int getPid() const;

        const std::string& getVersion() const;

        const std::string& getResult() const;

        const std::string& getErrorMessage() const;

    private:
        int m_Pid;
        std::string m_Version;
        std::string m_Result;
        std::string m_ErrorMessage;
    };
}

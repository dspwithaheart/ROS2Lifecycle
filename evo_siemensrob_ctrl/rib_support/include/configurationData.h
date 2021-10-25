//////////////////////////////////////////////////////////////////////////////////////////
/// This object contains all relevant configuration data that are required by the RIB when an app connects
/// 
/// \file configurationData.h
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
/// 
/// \author Team OCP
///
//////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "applicationData.h"
#include <string>

namespace RIB
{


    class ConfigurationData
    {
    private:
        ///
        /// Type of configuration data that is being sent to RIB
        ///
        std::string dataType;

        ///
        /// Version of configuration
        ///
        std::string interfaceVersion;

        ///
        /// Properties of the application
        ///
        ApplicationData appData;

        ///
        /// File descriptor of the socket
        ///
        int socketfd = 0;


    public:
        /// 
        /// Default destructor
        ///
        ~ConfigurationData() = default;

        /// 
        /// Constructor
        /// \param configuration data
        ///
        ConfigurationData(ConfigurationData const&) = default;

        ///
        /// Default assignment operator
        ///
        ConfigurationData& operator=(ConfigurationData const&) = default;

        ///
        /// Comparison operator
        /// \param configuration data that is compared with this object
        /// \return true if configuration data and compared object are equal
        ///
        bool operator==(ConfigurationData const&) const;

        ///
        /// Constructor 
        /// \param type
        /// \param version
        /// \param applicationData
        /// \param sockFd file descriptor of the socket that is bound to the application with this configuration data
        ///
        ConfigurationData(const std::string& type, const std::string& version, const ApplicationData& applicationData, int sockFd = 0);

        ///
        /// Retrieve data type
        /// \return dataType
        ///
        const std::string& getDataType() const;

        ///
        /// Retrieve version of the interface
        /// \return interfaceVersion
        ///
        const std::string& getInterfaceVersion() const;

        ///
        /// Retrieve application data
        /// \return appData
        ///
        ApplicationData& getAppData();

        ///
        /// Retrieve immutable application data
        /// \return appData
        ///
        const ApplicationData& getAppData() const;

        /// 
        /// Convert the attributes of this class into a json formatted string
        /// \return string in json format
        ///
        std::string toString() const;

        /// 
        /// Check whether the content of this object is valid
        /// \return true if object is valid
        ///
        bool isValid() const;

        /// 
        /// Retrieve the file descriptor of the socket that is bound to the application with this configuration data
        /// \return socket file descriptor
        ///
        int getSocketfd() const;

    };
}

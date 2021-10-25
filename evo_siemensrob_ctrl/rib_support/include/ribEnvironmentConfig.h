//////////////////////////////////////////////////////////////////////////////////////////
// TODO  missing description of header file
/// 
/// \file ribEnvironmentConfig.h
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
/// 
/// \author Team OCP
///
//////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <string>
#include <memory>

namespace RIB
{
    ///
    /// This class represents a set of environment configuration data in the RIB system.
    ///
    class RibEnvironmentConfig
    {
    public:
        ///
        /// Default string to request the RIBs environment configuration via socket.
        ///
        static std::string GetRequestRibEnvironmentConfig();

        ///
        /// Creates a RibEnvironmentConfig object from a given json string.
        /// \param ribEnvironmentConfigString json string with the RIB environment configuration.
        /// \return A pointer to a new created RibEnvironmentConfig object. This object must be released by its creator.
        /// throw std::invalid_argument when 
        ///                     - ribEnvironmentConfigString cannot be parsed as json file
        ///                     - parsed ribEnvironmentConfigString is not a ConfigDataResponse
        ///
        static std::shared_ptr<RibEnvironmentConfig> CreateFromString(const std::string& ribEnvironmentConfigString);

        ///
        /// Creates a new instance of RibEnvironmentConfig
        /// \param version Version of the configuration data.
        /// \param segmentLifeTime Represents the lifetime of buffer segments within the RIB system.
        RibEnvironmentConfig(const std::string& version, const uint64_t segmentLifeTime);

        ///
        /// Provides the version of the configuration data.
        ///     
        std::string getVersion() const;

        ///
        /// Provides the lifetime of buffer segments within the RIB system in miliseconds.
        ///
        uint64_t getSegmentLifeTime() const;

    private:

        ///
        /// Version of RibEnvironmentConfig
        ///
        const std::string m_Version;

        ///
        /// SegmentLifeTime in miliseconds
        ///
        const uint64_t m_SegmentLifeTimeIn_ms;
    };
}

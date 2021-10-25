//////////////////////////////////////////////////////////////////////////////////////////
/// Contains the implementation of the ResponseData class 
/// that manages the response message of the RIB when trying to connect.
/// 
/// \file responseData.h
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
/// 
/// \author Team OCP
///
//////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "ribInformation.h"
#include "dataProviderInfoMap.h"
#include <string>

namespace RIB
{
    class ResponseData
    {
    public:
        ///
        /// Constructor
        /// \param type of the response
        /// \param version
        /// \param ribInformation general information from the RIB
        /// \param dataProviderInfo information about the provided data
        ///
        ResponseData(const std::string& type, const std::string& version, const RibInformation& ribInformation, const DataProviderInfoMap& dataProviderInfo);

        ///
        /// Default copy constructor
        ///
        ResponseData(const ResponseData& responseData) = default;

        ///
        /// Default destructor
        ///
        ~ResponseData() = default;

        ///
        /// Default assignment operator
        ///
        ResponseData& operator= (const ResponseData& responseData) = default;

        ///
        /// Retrieve type
        /// \return type of the response
        ///
        const std::string& getType() const;

        ///
        /// Retrieve version
        /// \return version
        ///
        const std::string& getVersion() const;

        ///
        /// Retrieve general information from the RIB
        /// \return information from the RIB
        ///
        const RibInformation& getRibInfo() const;

        ///
        /// Retrieve information about the provided data
        /// \return map of shared memory IDs and symbols
        ///
        const DataProviderInfoMap& getDataProviderInfo() const;

    private:

        ///
        /// Type of the responseData
        ///
        std::string m_Type;

        ///
        /// Version of the current object to be able to distinguish between different states of the product
        ///
        std::string m_Version;

        ///
        /// Information about the RIB
        ///
        RibInformation m_RibInformation;

        ///
        /// Information about correlation between shared memory and symbols that are located in the shared memory
        ///
        DataProviderInfoMap m_DataProviderInfo;
    };
}

//////////////////////////////////////////////////////////////////////////////////////////
/// Helper functions to handle RIB configurations stored in JSON objects.
/// 
/// \file JsonUtilities.h
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
    class JsonUtilities
    {
    public:

        ///
        /// Read the content of the given file
        /// \param fileName complete path of the file that is being read
        /// \return content of the file
        ///
        std::string ReadJsonFromFile(const std::string& fileName);

        ///
        /// Set process ID and shared memory identifier in the given JSON object
        /// \param jsonToBeModified object in which the given parameters are changed
        /// \param shmIdentifier shared memory identifier that is written to the json object
        /// \return false if jsonToBeModified or shmIdentifier are empty
        ///
        bool SetPidAndSharedMemoryId(std::string& jsonToBeModified, const std::string& shmIdentifier) const;

        ///
        /// Set process ID of the calling process in the given JSON file
        /// \param jsonToBeModified json object in which process id is set
        /// \return false if json object is empty
        ///
        bool SetPid(std::string& jsonToBeModified) const;

    private:

        ///
        /// Replace substring in given string
        /// \param myString string that is being modified
        /// \param what substring that is being replaced
        /// \param with substring that will be used for replacement
        /// 
        void ReplaceInString(std::string& myString, const std::string& what, const std::string& with) const;

    };
}

//////////////////////////////////////////////////////////////////////////////////////////
/// This object contains all relevant data for applications that connect to the RIB
/// 
/// \file ApplicationData.h
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
/// 
/// \author Team OCP
///
//////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "provides.h"
#include "requests.h"
#include <string>
#include <list>

namespace RIB
{
    class ApplicationData
    {
    private:
        ///
        /// Name of the application
        ///
        std::string applicationName;

        ///
        /// Process ID of the application
        ///
        int processID;

        ///
        /// Userdefined description what the application does
        ///
        std::string description;

        ///
        /// Version
        ///
        std::string applicationVersion;

        ///
        /// Manufacturer of the App
        ///
        std::string manufacturer;

        ///
        /// List of the objects that are provided by this application
        ///
        std::list<Provides> provides{};

        ///
        /// Data that are requested by this application
        ///
        Requests requests{};

        /// 
        /// Check whether two lists of provides are identical
        /// \param provides1 first provides list
        /// \param provides2 second provides list
        /// \return true when both lists are identical
        ///
        static bool providesListsAreEqual(const std::list<Provides>& provides1, const std::list<Provides>& provides2);

    public:

        /// 
        /// Default destructor
        ///
        ~ApplicationData() = default;

        /// 
        /// Default asignment operator
        ///
        ApplicationData& operator=(ApplicationData const&) = default;

        /// 
        /// Default copy constructor
        ///
        ApplicationData(const ApplicationData&) = default;

        /// 
        /// Constructor
        /// \param name of the application
        /// \param pid process id of the application
        /// \param descr description of the application
        /// \param version 
        /// \param mnufact manufacturer
        /// \param prov list of provided data
        /// \param req list of requested data
        ///
        ApplicationData(const std::string& name, const int pid, const std::string& descr, const std::string& version, const std::string& mnufact, const std::list<Provides>& prov, const Requests& req);

        /// 
        /// Comparison operator
        /// \param application data that is compared with this object
        /// \return true when provides and requests lists, name, pid, description, version and manufacturer are equal
        ///
        bool operator==(const ApplicationData& appData2) const;

        /// 
        /// Retrieve application name
        /// \return name
        ///
        const std::string& getApplicationName() const;

        /// 
        /// Set application name
        /// \param newApplicationName name to be set
        ///
        void setApplicationName(const std::string newApplicationName);

        /// 
        /// Get process id of current object
        /// \return process id
        ///
        int getPid() const;

        /// 
        /// Set process id for current object
        /// \param newProcessID new process id to be set
        ///
        void setPid(const int newProcessID);

        /// 
        /// Get description of the current object
        /// \return description of the current object
        ///
        const std::string& getDescription() const;

        /// 
        /// Set description for current object
        /// \param newDescription description to be set
        ///
        void setDescription(const std::string newDescription);

        /// 
        /// Get version of the current object
        /// \return version of the current object
        ///
        const std::string& getApplicationVersion() const;

        /// 
        /// Set version for current object
        /// \param newAppVersion description to be set
        ///
        void setApplicationVersion(const std::string newAppVersion);

        /// 
        /// Get manufacturer of the current object
        /// \return manufacturer of the current object
        ///
        const std::string& getManufacturer() const;

        /// 
        /// Set manufacturer for current object
        /// \param newManufacturer manufacturer to be set
        ///
        void setManufacturer(const std::string newManufacturer);

        /// 
        /// Get list of provides of the current object
        /// \return provides list  
        ///
        const std::list<Provides>& getProvides() const;

        /// 
        /// Get list of requests of the current object
        /// \return requests list  
        ///
        const Requests& getRequests() const;

        /// 
        /// Add a new provides object to the existing provides list
        /// \param providedData new object to be added to the list
        ///
        void add(Provides providedData);

        /// 
        /// Remove provides object from the list by shared memory identifier
        /// \param shmIdentifier of provides object that has to be removed
        /// \return false if no provides object with the given identifier was found; true if object was found and removed
        ///
        bool tryRemoveProvide(const std::string& shmIdentifier);

        /// 
        /// Remove given provides object from the list of provides
        /// \param provides object to be removed from the list
        /// \return false if the given object wasn't found; true if the object was found and removed
        ///
        bool tryRemoveProvide(const Provides& providedData);

        /// 
        /// Add symbol to the requests list
        /// \param symbolName that is added
        ///
        void addRequestedSymbolName(const std::string& symbolName);

        /// 
        /// Remove symbol from the requests list
        /// \param symbolName that is removed
        ///
        void removeRequestedSymbolName(const std::string& symbolName);
    };
}

//////////////////////////////////////////////////////////////////////////////////////////
/// This object bundles all information regarding provided data of an application
/// 
/// \file provides.h
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
/// 
/// \author Team OCP
///
//////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "symbol.h"
#include <string>
#include <list>

namespace RIB
{
    class Requests;

class Provides
    {
    private:
        ///
        /// Identifier of the shared memory where the symbols of this provides object are stored in
        ///
        std::string m_shmID;
        
        ///
        /// Signal placeholder that is reserved for future use
        ///
        int m_signal;
        
        ///
        /// Descriptive string for the current object
        ///
        std::string m_description;
        
        ///
        /// Version
        ///
        std::string m_appVersion;
        
        ///
        /// List of symbols that are provided
        ///
        std::list<Symbol> m_symbols;

        ///
        /// Compare given shared memory ID with own
        /// \param otherShmId ID that is compared with own
        /// \return true when both ID's are identical
        ///
        bool isEqualToOwnSharedMemoryIdentifier(const std::string& otherShmId) const;

    public:
        ///
        /// Default copy constructor
        ///
        Provides(Provides const&) = default;
        
        ///
        /// Default destructor
        ///
        ~Provides() = default;
        
        ///
        /// Constructor
        /// \param descr description
        /// \param version
        /// \param symbolList list of symbols
        /// \param shmId shared memory identifier
        /// \param signal
        ///
        Provides(const std::string& descr, const std::string& version, const std::list<Symbol>& symbolList, const std::string& shmId, const int& signal);

        ///
        /// Compare shared memory identifier of given provide object with identifier of current object
        /// \param provide object thats shared memory ID will be taken for comparison
        /// \return true when both shared memory IDs are equal
        ///
        bool operator==(const Provides& provide) const;

        ///
        /// Compare given shared memory identifier with identifier of current object
        /// \param shared memory ID that will be taken for comparison
        /// \return true when both shared memory IDs are equal
        ///
        bool operator==(const std::string& shmIdentifier) const;

        ///
        /// Compare shared memory identifier of given provide object with identifier of current object
        /// \param provide object thats shared memory ID will be taken for comparison
        /// \return false when both shared memory IDs are equal
        ///
        bool operator!=(const Provides& provide) const;
        
        ///
        /// Retrieve signal of current object
        /// \return signal
        ///
        int getSignal() const;

        ///
        /// Retrieve shared memory identifier of current object
        /// \return shared memory identifier
        ///
        const std::string getShmID() const;

        ///
        /// Retrieve description of current object
        /// \return description
        ///
        const std::string& getDescription() const;
        
        ///
        /// Retrieve application version of current object
        /// \return version
        ///
        const std::string& getAppVersion() const;
        
        ///
        /// Retrieve list of provided symbols of current object
        /// \return list of provided symbols
        ///
        const std::list<Symbol>& getSymbols() const;
    };
}

//////////////////////////////////////////////////////////////////////////////////////////
/// This object describes the requested symbols of an application.
/// 
/// \file requests.h
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
/// 
/// \author Team OCP
///
//////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <unordered_set>
#include <string>
namespace RIB
{
    class Requests
    {
    private:
        ///
        /// a list of symbol names, each symbol name is unique in this list.
        ///
        std::unordered_set<std::string> m_SymbolNames;
    public:
        ///
        /// Default copy constructor
        ///
        Requests(Requests const&) = default;

        ///
        /// Default destructor
        ///
        ~Requests() = default;

        ///
        /// Constructor
        /// \param symbolList list of symbols that are requested
        ///
        Requests(const std::unordered_set<std::string>& symbolList = std::unordered_set<std::string>());

        ///
        /// Adding a new symbol to the list of symbols that are requested
        /// \param symbolName name of symbol to be added
        ///
        void addSymbolName(const std::string& symbolName);
        
        ///
        /// Removing a symbol from the list of symbols that are requested
        /// \param symbolName name of symbol to be removed
        ///
        void removeSymbolName(const std::string& symbolName);

        ///
        /// Retrieve all symbols that are currently in the request object
        /// \return list of symbols
        ///
        const std::unordered_set<std::string>& getSymbolNames() const;

        ///
        /// Operator equals compares the two lists of symbols
        /// \param request object to be compared
        /// \return true when both lists are equal
        ///
        bool operator== (const Requests& request) const;
    };
}
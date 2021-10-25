//////////////////////////////////////////////////////////////////////////////////////////
/// Collection of ConsistentDataTransfer objects and functions to control them
/// 
/// \file symbolImage.h
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
/// 
/// \author Team OCP
///
//////////////////////////////////////////////////////////////////////////////////////////


#pragma once

#include "symbol.h"
#include <memory>
#include <list>
#include <string>
#include <set>
#include <unordered_map>

namespace RIB
{
    class IConsistentDataTransferReader;

    class SymbolImage
    {
    private:
        ///
       /// Type alias for a list of IConsistentDataTransferReader.
       ///
        using ConsistentDataTransferReaderList = std::list<std::shared_ptr<IConsistentDataTransferReader>>;

        ///
        /// Type alias for a map of IConsistentDataTransferReader and its symbols.
        ///
        using ConsistentDataTransferReaderToSymbolMap = std::pair<std::shared_ptr<IConsistentDataTransferReader>, Symbol>;

        ///
        /// Type alias for a map to resolve IConsistentDataTransferReader and its symbols by a known symbol name.
        ///
        using SymbolNameResolveMap = std::unordered_map<std::string, ConsistentDataTransferReaderToSymbolMap>;


        ///
        /// List of all consistentDataTranserReader
        ///
        ConsistentDataTransferReaderList m_cdtList = ConsistentDataTransferReaderList();

        ///
        /// Map symbol name to corresponding pair of consistentDataTransferReader and symbol
        ///
        SymbolNameResolveMap m_symbolNameResolveMap = SymbolNameResolveMap();

    public:
        /// 
        /// Default Constructor
        ///
        SymbolImage() = default;

        /// 
        /// Default Destructor
        ///
        ~SymbolImage() = default;

        //TODO: consinder making addConsistentDataTransfer() and removeConsistentDataTransfer() private functions. The end user does not need access to them.
        /// 
        /// Add new ConsistentDataTransferReader object to list of objects
        /// \param newCDT object to add
        /// \param symbolSet list of symbols that is provided by given consitentDataTransferReader object
        /// \return true if adding was successful
        ///
        bool addConsistentDataTransfer(const std::shared_ptr<IConsistentDataTransferReader>& newCDT, std::set<Symbol> symbolSet);

        /// 
        /// Remove ConsistentDataTransferReader from list of objects
        /// \param removeCDT to be removed
        /// \return true if remove was successful
        ///
        bool removeConsistentDataTransfer(const std::shared_ptr<IConsistentDataTransferReader>& removeCDT);

        /// 
        /// Get pointer to the symbol in a buffer of one of the ConsistentDataTransferReader objects
        /// \param symbol name of the requested symbol
        /// \return void pointer to symbol in the symbol image
        ///
        void* getPointerToSymbol(const std::string& symbolName) const;

        ///
        /// performs an update of all added ConsistentDataTransferReader objects.
        /// \return  true if update for all ConsistentDataTransferReader objects was successful, false otherwise.
        ///
        bool update() const;

        /// 
        /// closes all ConsistentDataTransferReader objects which are part of the symbol image. 
        ///
        void close();
    };
}

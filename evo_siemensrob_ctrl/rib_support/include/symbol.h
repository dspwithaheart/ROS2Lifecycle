//////////////////////////////////////////////////////////////////////////////////////////
/// Symbol class defines composition and struction of Symbols  
/// 
/// \file symbol.h
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
/// 
/// \author Team OCP
///
//////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include <string>
#include <sys/types.h>

namespace RIB
{
    //TODO: introduce public interface for this class.
    class Symbol
    {
    private:
        ///
        /// Symbol name
        ///
        std::string name = "";
        ///
        /// Symbol type
        ///
        std::string type = "";
        ///
        /// Symbol length (number of bytes)
        ///
        uint64_t sizeOf = 0;
        ///
        /// Symbol offset (number of bytes)
        /// 
        uint64_t offset = 0;

    public:
        ///
        /// Default constructor
        ///
        Symbol() = default;

        ///
        /// Default Destructor
        ///
        ~Symbol() = default;

        /// 
        /// Default copy constructor
        ///
        Symbol(Symbol const&) = default;

        /// 
        /// Default assignment operator
        ///
        Symbol& operator=(Symbol const&) = default;

        /// 
        /// Comparison operator less than based on alphabetical sort order of the symbol name
        /// \param symbol is compared with Symbol object
        ///
        bool operator<(Symbol const& symbol) const;

        /// 
        /// Comparison operator equal to 
        /// \param symbol is compared with Symbol object
        ///
        bool operator==(Symbol const& symbol) const;

        /// 
        /// Constructor
        /// \param symbolName
        /// \param symbolType
        /// \param size
        /// \param symbolOffset
        ///
        Symbol(const std::string symbolName, const std::string symbolType, const uint64_t size, const uint64_t symbolOffset);

        ///
        /// Retrieves the name of the symbol
        /// \return name of the symbol
        ///
        const std::string& getName() const;

        ///
        /// Set symbol name
        /// \param symbolName   
        ///
        void setName(const std::string& symbolName);

        ///
        /// Retrieves data type of the symbol
        /// \return symbol type
        ///
        const std::string& getType() const;

        ///
        /// Set symbol data type
        /// \param symbolType 
        ///
        void setType(const std::string& symbolType);

        ///
        /// Retrieve size of symbol
        /// \return symbol size
        ///
        uint64_t getSizeOf() const;

        ///
        /// Set symbol size (number of bytes)
        /// \param size Symbol length
        ///
        void setSizeOf(const uint64_t size);

        /// 
        /// Retrieve symbol offset (number of bytes)
        /// \return symbol offset
        ///
        uint64_t getOffset() const;

        /// 
        /// Set symbol offset (number of bytes)
        /// \param symbolOffset 
        ///
        void setOffset(const uint64_t symbolOffset);
    };
}

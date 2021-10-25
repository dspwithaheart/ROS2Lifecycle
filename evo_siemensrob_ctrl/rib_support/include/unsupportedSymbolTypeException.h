//////////////////////////////////////////////////////////////////////////////////////////
/// Contains the implementation of the UnsupportedSymbolTypeException class 
/// for handling of specific exceptions when using an unsupported symbolType 
/// while working with RIB.
/// 
/// \file unsupportedSymbolTypeException.h
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
/// 
/// \author Team OCP
///
//////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "ribException.h"

namespace RIB
{
    class UnsupportedSymbolTypeException final : public RIBException
    {
    public:
        ///
        /// Constructor
        /// \param symbolName
        /// \param unsupportedSymbolType
        ///
        UnsupportedSymbolTypeException(const std::string& symbolName, const std::string& unsupportedSymbolType);

        ///
        /// Destructor
        ///
        ~UnsupportedSymbolTypeException();
    };
}

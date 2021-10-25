//////////////////////////////////////////////////////////////////////////////////////////
/// Interface for a basic RIB data structure 
/// 
/// \file IRIBBaseDataStruct.h
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
/// 
/// \author Team OCP
///
//////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "symbol.h"
#include <list>

//TODO: To be verfied whether offset calculation is valid and secure 
//-> Different Compiler might generate different memory layout for the same code. In that case, is this calculation still valid
#define OFFSET_OF_VARIABLE_IN_STRUCT(x) reinterpret_cast<uint64_t>(std::addressof(x)) - reinterpret_cast<uint64_t>(std::addressof(*this))


// TODO: Using a base class creates a vtable that is always written to the buffer element in the shared memory. This vtable must not be stored in the life time buffer (simplify data storage and consistency issues)
class IRIBBaseDataStruct
{
public:
    ///
    /// Destructor for clean up.
    ///
    virtual ~IRIBBaseDataStruct() {};

    ///
    /// Access the symbols description for the RIB-Configuration.
    /// \return A list of Symbol objects describing the data within a shared memory.
    ///
    virtual std::list<RIB::Symbol> getSymbolList(void) = 0;
};

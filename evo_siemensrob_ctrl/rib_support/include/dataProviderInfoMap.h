//////////////////////////////////////////////////////////////////////////////////////////
/// Description of types that are used for DataProviderInfo object
/// 
/// \file dataProviderInfoMap.h
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
/// 
/// \author Team OCP
///
//////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "symbol.h"
#include <set>
#include <unordered_map>

namespace RIB
{
    ///
    /// Represents a pair of SharedMemoryId and a set of Symbols that can be found there.
    /// 
    using DataProviderInfo = std::pair<std::string, std::set<Symbol>>;

    ///
    /// Represents a map of SharedMemoryIds and a related set of Symbols that can be found there.
    /// 
    using DataProviderInfoMap = std::unordered_map<std::string, std::set<Symbol>>;

}

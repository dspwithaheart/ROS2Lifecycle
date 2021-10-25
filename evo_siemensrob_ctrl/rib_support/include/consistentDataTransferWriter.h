//////////////////////////////////////////////////////////////////////////////////////////
/// Object for writing information of data exchange buffer in shared memory
/// 
/// \file consistentDataTransferWriter.h
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
/// 
/// \author Team OCP
///
//////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "consistentDataTransfer.h"
#include <memory>

class IRIBBaseDataStruct;

namespace RIB
{
    class IRibShm;

    class ConsistentDataTransferWriter : public ConsistentDataTransfer
    {
    private:

        ///
        /// Creates the structure of the buffer in the shared memory
        /// \param segmentCount number of segments of the buffer        
        /// \param userDataSize size of user data to be exchanged
        /// \return a pointer of type header to the start of the lifetime buffer
        /// \throw std::invalid_argument when segmentCount is less than Constants::minimalBufferSizeForConsistency
        ///
        virtual void createBuffer(const uint32_t segmentCount, const size_t userDataSize);

    public:

        ///
        /// Provider's constructor to create a buffer 
        /// \param segmentCount number of segments of the buffer
        /// \param ribShm pointer to the shared memory 
        /// \param userDataSize size of user data to be exchanged
        /// \param watchdog watchdog to ensure consistency
        ///
        ConsistentDataTransferWriter(const uint32_t segmentCount, std::shared_ptr<IRibShm> const ribShm, const size_t userDataSize);

        ///
        /// Destructor
        ///
        ~ConsistentDataTransferWriter() = default;

        ///
        /// Copy constructor for test matters
        ///
        ConsistentDataTransferWriter(const ConsistentDataTransferWriter&) = default;

        ///
        /// Overload of operator= for test matters
        /// return a reference to a ConsistentDataTransfer object
        ///
        ConsistentDataTransferWriter& operator=(const ConsistentDataTransferWriter&) = default;

        ///
        /// Writes user data into a buffer for consistency matters
        /// \param userData pointer to struct with user data
        /// \throw std::invalid_argument when userData is nullptr
        ///
        void writeUserData(IRIBBaseDataStruct* const userData);
    };
}

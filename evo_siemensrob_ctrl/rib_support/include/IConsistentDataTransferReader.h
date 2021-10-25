//////////////////////////////////////////////////////////////////////////////////////////
/// Interface for reading information of data exchange buffer in shared memory
/// 
/// \file IConsistentDataTransferReader.h
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
/// 
/// \author Team OCP
///
//////////////////////////////////////////////////////////////////////////////////////////

#pragma once

class IRIBBaseDataStruct;

namespace RIB
{
    class IConsistentDataTransferReader
    {
    public:
        ///
        /// Destructor
        ///
        virtual ~IConsistentDataTransferReader() = default;

        ///
        /// Overload of operator== for comparison and lists
        /// return true if equal, false if not equal
        ///
        virtual bool operator==(const IConsistentDataTransferReader& compareTo) const = 0;

        ///
        /// Reads consistent user data from a buffer
        /// \param userData reference to a user data struct variable where the read data will be written
        /// \return true when data was correctly read from buffer, false when no data has been written to buffer yet.
        ///
        virtual bool readUserData(IRIBBaseDataStruct* const userData) const = 0;

        ///
        /// Updates the byteBuffer consistently with the latest valid buffer element data from the life time buffer
        /// \return true when data was correctly read from buffer, false when no data has been written to buffer yet.
        ///
        virtual bool update() const = 0;

        ///
        /// Gives access to a byteBuffer containing the data of a buffer element 
        /// \return a pointer to the byteBuffer
        ///
        virtual void* getByteBuffer() const = 0;

        //TODO: hide close from customer, is internally used only.
        ///
        /// Closes the ConsistentDataTransferReader and the related shared memory.
        ///
        virtual void close() = 0;

    };
}

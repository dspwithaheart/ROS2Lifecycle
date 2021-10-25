//////////////////////////////////////////////////////////////////////////////////////////
/// Object for reading and writing information of data exchange buffer in shared memory
/// 
/// \file consistentDataTransfer.h
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
/// 
/// \author Team OCP
///
//////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <memory>

namespace RIB
{
    struct Header;
    class IRibShm;

    class ConsistentDataTransfer
    {
    private:

        ///
        /// Sets the pointer to the start of the segments of the buffer     
        /// \return address of segments' start
        ///
        void* calculateSegmentsBase();

        ///
        /// Initialize user data and consistency counter of all segments to zero
        /// \param segmentCount number of segments of the buffer
        /// \param segmentSize size of segments of the buffer
        ///
        void initializeSegments(const uint32_t segmentCount, const uint32_t segmentSize);

    protected:

        std::shared_ptr<IRibShm> m_RibShm;
        void* m_SegmentsBase = nullptr;
        Header* m_bufferHeader = nullptr;

        ///
        /// ConsistentDataTransfer base class constructor 
        /// \param ribShm pointer to the shared memory  
        /// \throw std::invalid_argument when ribShm or buffer is nullptr
        ///
        ConsistentDataTransfer(std::shared_ptr<IRibShm> const ribShm);

        ///
        /// Virtual destructor
        ///
        virtual ~ConsistentDataTransfer() = default;

        ///
        /// Copy constructor for test matters
        ///
        ConsistentDataTransfer(const ConsistentDataTransfer&) = default;

        ///
        /// Assignment operator for test matters
        /// return a reference to a ConsistentDataTransfer object
        ///
        ConsistentDataTransfer& operator=(const ConsistentDataTransfer&) = default;

        ///
        /// Overload of operator== for comparison and lists
        /// return true if equal, false if not equal
        ///
        virtual bool operator==(const ConsistentDataTransfer& compareCDT) const;

        ///
        /// Get start address of given segment
        /// \param segmentNumber Segment from which the start address should be returned
        /// \return the address of given segment number
        ///     
        uint8_t* getSegmentStart(const uint32_t segmentNumber) const;

        ///
        /// get getSegmentCount 
        /// \return the current valid set segmentCount
        ///
        uint32_t getSegmentCount() const;
    };
}

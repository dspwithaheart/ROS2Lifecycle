//////////////////////////////////////////////////////////////////////////////////////////
/// Constants that are useful when working with the RIB
/// 
/// \file ribConstants.h
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
/// 
/// \author Team OCP
///
//////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>

namespace RIB
{
    namespace Constants
    {
        ///
        /// a billion is Giga (10E9)
        ///
        constexpr uint32_t billion = 1000000000;

        ///
        /// Size of buffer has to be at least 3
        /// These are backoff segments to ensure that the guaranteed lifetime is always adhered
        ///
        constexpr uint32_t minimalBufferSizeForConsistency = 3;
    }

    namespace ConfigParameters
    {

        class RibProcessConfig // TODO: must this be a class or better a namespace? (see also SocketConnectionConfig, BufferInfo below)
        {
        public:
            ///
            /// Scheduling priority for the Apps when working in real time mode
            ///
            static constexpr uint16_t rib_rt_prio = 30;

        private:
            ///
            /// Default constructor
            ///
            RibProcessConfig() = default;

            ///
            /// Default destructor
            ///
            ~RibProcessConfig() = default;
        };

        class SocketConnectionConfig
        {
        public:
            ///
            /// Maximum string size for receiving a string over a socket
            ///
            static constexpr uint16_t max_received_string_size = 4096;

            ///

            /// Default IP that the RIB is listening to when waiting for an app to connect
            ///
            // does not work due flaw in C++11
            //static const char rib_app_ip_addr[] = "127.0.0.1";
            // does not work due -Wwrite-strings
            //static constexpr char* rib_app_ip_addr = "127.0.0.1";

            ///
            /// Default port that the RIB is listening to when waiting for an app to connect
            ///
            static constexpr uint16_t rib_app_port = 27567;

            ///
            /// queue length for completely established sockets waiting to be accepted by the RIB
            ///
            static constexpr uint16_t sockets_queue_length = 10;

        private:
            ///
            /// Default constructor
            ///
            SocketConnectionConfig() = default;

            ///
            /// Default destructor
            ///
            ~SocketConnectionConfig() = default;
        };

        class BufferInfo
        {
        public:
            ///
            /// Major digit of version
            ///
            static constexpr uint8_t Major = 1;

            ///
            /// Minor digit of version
            ///
            static constexpr uint8_t Minor = 0;
        private:

            ///
            /// Default constructor
            ///
            BufferInfo() = default;

            ///
            /// Default destructor
            ///
            ~BufferInfo() = default;
        };
    }
}

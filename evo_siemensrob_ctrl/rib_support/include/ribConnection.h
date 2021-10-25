//////////////////////////////////////////////////////////////////////////////////////////
/// Contains the implementation of the Ribconnection class that manages 
/// the connection of an application to the RIB, e.g. 
/// connecting, connection status, data transfer using lifetime buffer concept.
/// 
/// \file ribConnection.h
/// 
/// \copyright Copyright (c) Siemens AG 2020 All Rights Reserved. Confidential
/// 
/// \author Team OCP
///
//////////////////////////////////////////////////////////////////////////////////////////

#pragma once

//TODO: check SOLID principles from this class.

#include "configurationData.h"
#include "ribConstants.h"
#include <string>
#include <memory>
#include <unordered_set>

class IRIBBaseDataStruct;

namespace RIB
{
    class ConsistentDataTransferWriter;
    class IConsistentDataTransferReader;
    class ISocketConnection;
    class ResponseData;
    class IShmMapper;
    class IRibShm;
    class RibEnvironmentConfig;
    class SymbolImage;

    class RibConnection
    {

    public:
        ///
        /// Constructor
        /// \param config Configuration data of the app that is connecting to the RIB
        /// \param socketConnection to send and receive data to and from the RIB
        /// \param shmMapper accessor for shared memory
        /// \throw std::invalid_argument when socketConnection or shmMapper is nullptr
        ///
        RibConnection(ConfigurationData& config, std::shared_ptr<ISocketConnection> socketConnection, std::shared_ptr <IShmMapper> shmMapper);

        ///
        /// Default copy constructor
        ///
        RibConnection(RibConnection const&) = default;

        ///
        /// Default destructor
        ///
        ~RibConnection() = default;

        ///
        /// Default assignment operator
        ///
        RibConnection& operator=(const RibConnection& ribConnection) = default;

        ///
        /// Creates a RIB Connection object with minimal connect configuration.
        /// \return RibConnection object
        ///
        static RibConnection Create();

        ///
        /// Creates a RIB Connection object based on the given connectionConfiguration string which represents a valid RIB json configuration.
        /// \param connectionConfiguration String with the connection configuration
        /// \return RibConnection object
        ///
        static RibConnection Create(const std::string& connectionConfiguration);

        ///
        /// get configuration data
        /// \return A ConfigurationData object which contains info to be able to connect to the RIB
        ///
        ConfigurationData& getConfig();

        ///
        /// get response data
        /// \return A pointer to an responseData object which contains the RIBs response
        ///
        const std::shared_ptr<ResponseData> getResponse() const;

        ///
        /// Add a shared memory given information to build a Provides object and a struct containing the variables to be shared
        /// \param shmId string with shared memory identifier (name)
        /// \param descr optional description of the provided data set
        /// \param version optional version of the provided data set
        /// \param signal signal is raised when the provided data has changed
        /// \param dataStruct structure which contains the variables to be shared
        /// \param cycleTimeInMicroSeconds cycletime which is given to
        /// \param sizeOfUserDataStruct size of user data struct
        /// \return A ConsistentDataTransferWriter object to write consistent user data to the lifetime buffer 
        /// \throw RIBException when RibConnection::getRibEnvironmentConfig() hasn't been called before
        ///
        ConsistentDataTransferWriter addLifetimeBuffer(const std::string& shmIdentifier, const std::string& descr, const std::string& version, int signal, IRIBBaseDataStruct& dataStruct, uint64_t cycleTimeInMicroSeconds = 1, uint32_t sizeOfUserDataStruct = 0);

        ///
        /// Connect to an existing lifetime buffer with a given share memory identifier
        /// \param shmIdentifier string with the name of the shared memory to be created
        /// \return A ConsistencyDataTransferRader object to read user data from the lifetime buffer
        ///
        std::weak_ptr<IConsistentDataTransferReader> connectToExistingLifetimeBuffer(const std::string& shmIdentifier);

        ///
        /// Disconnect from lifetime buffer
        /// \param consistentDataTransferReader weak_ptr to an object of the class ConsistentDataTransferReader
        ///
        void disconnectFromExistingLifetimeBuffer(std::weak_ptr<IConsistentDataTransferReader> consistentDataTransferReader);

        ///
        /// Provides a SymbolImage for all symbols that shall be read from the RIB.
        /// \return A valid SymbolImage when the connection to  RIB is established successfully and only if data providers are available, otherwise a nullptr.
        ///
        std::weak_ptr<SymbolImage> getOrCreateSymbolImageForAllConnectedLifeTimeBuffers();

        ///
        /// Release SymbolImage and all related reader and shared memory objects.
        ///
        void releaseSymbolImage();

        ///
        /// Requests the configuration data of the RIB environment.
        ///
        void RequestRibEnvironmentConfig();

        ///
        /// get RibEnvironmentConfig
        /// \return A pointer to a RibEnvironmentConfig object which contains the RIBs environmental config data
        ///
        std::shared_ptr<RibEnvironmentConfig> getRibEnvironmentConfig() const;

        ///
        /// Establishes a connection to the RIB based on the given configuration.
        /// /param  ipAddr  IP address where RIB_APP listens
        /// /param  port    port where RIB_APP listens
        ///
        void Connect(const std::string& ipAddr = std::string("127.0.0.1"), const uint16_t port = RIB::ConfigParameters::SocketConnectionConfig::rib_app_port);

        ///
        /// Close the connection to the RIB.
        ///
        void Disconnect();

        ///
        /// Indicates if a connection to the RIB is established.
        /// \return true when the connection is established, otherwise false
        ///
        bool IsConnected() const;

        ///
        /// Sign in to RIB to start providing and consuming data
        /// throw RIBException when RibConnection is not connected to the RIB or RibConnection does not contain RibEnvironmentConfig
        ///
        void SignIn();

        ///
        /// Sign out from RIB and stop reading from shared memories
        ///
        void SignOut();

        ///
        /// Indicates if a connection to the RIB is established.
        /// \return true when the connection is established, otherwise false
        ///
        bool IsSignedIn() const;

        ///
        /// Add a request symbol name to ribConnection-object
        /// \param requestedSymbolName the name of a requested symbol
        ///
        void addRequestedSymbol(const std::string& requestedSymbolName);

        ///
        /// Add requested symbols using a list of symbol names.
        /// \param requestedSymbolName a list of the requested symbol names
        ///
        void addRequestedSymbols(const std::unordered_set<std::string>& requestedSymbolNames);

        ///
        /// get getSegmentCount (public for test purpose)
        /// \return the current valid set segmentCount
        ///
        uint32_t getSegmentCount() const;

        ///
        /// Calculate segmentCount from given cycleTime and segmentLifetime provided by RIB (public for test purpose)
        /// \param cycleTimeInMicroSeconds cycleTime of the app that generates the buffer
        /// \return Calculated segment count
        /// \throw std::invalid_argument when 
        ///                     - cycleTimeInMicroSeconds is 0
        ///                     - segmentLifeTime is 0 or segmentLifeTime >= UINT64MAX / 1000
        /// \throw RIBException when segmentCount > UINT32_MAX
        ///
        uint32_t calculateSegmentCount(const uint64_t cycleTimeInMicroSeconds) const;

    private:
        ///
        /// set configuration data
        /// \param config A ConfigurationData object which contains info to be able to connect to the RIB
        ///
        void setConfig(const ConfigurationData& config);

        ///
        /// Add a shared memory given information to build a Provides object and a struct containing the variables to be shared
        /// \param shmId string with shared memory identifier (name)
        /// \param segmentCount number of segments of the buffer
        /// \param sizeOfUserDataStruct size of user data struct
        /// \return A void pointer with the address of the created shared memory
        ///
        std::shared_ptr<IRibShm> addSharedMemory(const std::string& shmId, uint32_t segmentCount = 1, uint32_t sizeOfUserDataStruct = 0);

        ///
        /// Create a shared memory given a string with the shared memory id (name for the shared memory)
        /// \param shmIdentifier string with the name of the shared memory to be created
        /// \param sizeOfSharedMemory string with the name of the shared memory to be created
        /// \return A void pointer with the address of the created shared memory
        /// \throw RibException if the shared memory cannot be created
        ///
        std::shared_ptr<IRibShm> createSharedMemory(const std::string& shmIdentifier, const uint32_t sizeOfSharedMemory);

        ///
        /// Adding a new provides object to the current 
        /// \param shmIdentifier shared memory identifier
        /// \param descr description of the provided data set
        /// \param version version of the provided data set
        /// \param signal signal is raised when the provided data has changed
        /// \param dataStruct struct of provided data
        ///
        void addProvideToAppConfig(const std::string& shmIdentifier, const std::string& descr, const std::string& version, int signal, IRIBBaseDataStruct& dataStruct);

        ///
        /// Configuration data of the providing app
        ///
        ConfigurationData m_Config;

        ///
        /// Socket connection of the connection between app and RIB
        ///
        std::shared_ptr<ISocketConnection> m_SocketConnection = nullptr;

        ///
        /// Access to shared memeory
        ///
        std::shared_ptr<IShmMapper> m_SharedMemoryMapper = nullptr;

        ///
        /// Object to store environment information of the RIB
        ///
        std::shared_ptr<RibEnvironmentConfig> m_RibEnvironmentConfig = nullptr;

        ///
        /// Set of all known ConsistentDataTransferReader objects created with method connectToExistingLifetimeBuffer()
        ///
        std::unordered_set<std::shared_ptr<IConsistentDataTransferReader>> m_consistentDataTransferReaders = std::unordered_set<std::shared_ptr<IConsistentDataTransferReader>>();

        ///
        /// A valid SymbolImage when the connection to  RIB is established successfully, otherwise a nullptr.
        ///
        std::shared_ptr<SymbolImage> m_SymbolImage = nullptr;

        ///
        /// IP-Address of RIB_App
        ///
        std::string m_IpToConnect = "";

        ///
        /// Port of RIB_App
        ///
        uint16_t m_PortToConnect = 0;

        ///
        /// Connect to an existing lifetime buffer with a given share memory identifier without adding the item to the list m_consistentDataTransferReaders
        /// \param shmIdentifier string with the name of the shared memory to be created
        /// \return A ConsistencyDataTransferRader object to read user data from the lifetime buffer
        /// \throw RIBException when
        ///                    - RibConnection::getRibEnvironmentConfig() hasn't been called before
        ///                    - no connection to the RIB has been established
        ///                    - connect to shared Memory was not successful
        ///
        std::shared_ptr<IConsistentDataTransferReader> connectToExistingLifetimeBufferInternal(const std::string& shmIdentifier);
    };
}

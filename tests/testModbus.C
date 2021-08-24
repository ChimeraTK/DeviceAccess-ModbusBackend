/*
 * testModbus.cc
 *
 *  Created on: Mar 19, 2020
 *      Author: Klaus Zenker (HZDR)
 */

#define BOOST_TEST_MODULE ModbusTest

#include "ModbusBackend.h"

#include <ChimeraTK/Device.h>
#include <ChimeraTK/UnifiedBackendTest.h>

#include <netinet/ip.h>
#include <arpa/inet.h>

//#define BOOST_NO_EXCEPTIONS
#include <boost/test/included/unit_test.hpp>
//#undef BOOST_NO_EXCEPTIONS
using namespace boost::unit_test_framework;

using namespace ChimeraTK;

/**********************************************************************************************************************/

struct ModbusTestServer {
  ModbusTestServer() {
    // create mapping
    _mapping.nb_bits = 0;
    _mapping.nb_input_bits = 0;
    _mapping.nb_registers = sizeof(_map_holding) / 2;
    _mapping.nb_input_registers = sizeof(_map_input) / 2;
    _mapping.start_bits = 0;
    _mapping.start_input_bits = 0;
    _mapping.start_registers = 0;
    _mapping.start_input_registers = 1024 / 2; // map file address is in bytes, this on is in (16 bit) words
    _mapping.tab_bits = nullptr;
    _mapping.tab_input_bits = nullptr;
    _mapping.tab_registers = static_cast<uint16_t*>(static_cast<void*>(&_map_holding));
    _mapping.tab_input_registers = static_cast<uint16_t*>(static_cast<void*>(&_map_input));

    // launch server
    _serverThread = std::thread([this] { this->theServer(); });
  }

  ~ModbusTestServer() {
    _shutdown = true;
    modbus_connect(modbus_new_tcp("127.0.0.1", serverPort()));
    _serverThread.join();
  }

  int serverPort() { return _serverPort; }

  struct map_holding {
    int16_t reg1[1];
    int16_t reg2[1];
    uint16_t reg3[1];
    int16_t reg4_raw[1];
    int32_t reg32[1];
    float reg754[1];
  };
  struct map_input {
    int16_t reg1[1];
    int16_t reg2[1];
  };

  std::unique_lock<std::mutex> getLock() { return std::unique_lock<std::mutex>(_mx_mapping); }
  map_holding& getHolding() { return _map_holding; }
  map_input& getInput() { return _map_input; }

  void setException(bool enable) { _exception = enable; }

 protected:
  void theServer() {
    // Create modbus server
    modbus_t* ctx = modbus_new_tcp("0.0.0.0", serverPort());
    int server_socket = modbus_tcp_listen(ctx, 1);
    if(server_socket == -1) {
      modbus_free(ctx);
      std::cout << "Unable to listen TCP connection\n";
    }

    // Prepare set of FDs for select() with just the server socket in for now. (Client connections will be added later
    // when requests come in.)
    fd_set refset;
    FD_ZERO(&refset);
    FD_SET(server_socket, &refset);

    // Keep track of the max file descriptor, needed for select()
    auto fdmax = server_socket;

    for(;;) {
      // Wait for any of the connections / server socket to receive data
      fd_set rdset = refset;
      if(select(fdmax + 1, &rdset, nullptr, nullptr, nullptr) == -1) {
        std::cout << "Server select() failure.\n";
      }

      // shutdown thread?
      if(_shutdown) {
        return;
      }

      // Check which socket has received data
      for(int master_socket = 0; master_socket <= fdmax; master_socket++) {
        if(!FD_ISSET(master_socket, &rdset)) {
          continue;
        }

        // New data on server_socket: new client tries to connect
        if(master_socket == server_socket) {
          // Create new connection
          struct sockaddr_in clientaddr;
          socklen_t addrlen = sizeof(clientaddr);
          memset(&clientaddr, 0, sizeof(clientaddr));
          int newfd = accept(server_socket, (struct sockaddr*)&clientaddr, &addrlen);
          if(newfd == -1) {
            perror("Server accept() error");
          }
          else {
            // Add new
            FD_SET(newfd, &refset);

            if(newfd > fdmax) {
              /* Keep track of the maximum */
              fdmax = newfd;
            }
            std::cout << "New connection from " << inet_ntoa(clientaddr.sin_addr) << ":" << clientaddr.sin_port
                      << " on socket " << newfd << "\n";
          }
        }
        // Data received on any other socket: client is requesting read/write
        else {
          // Process request
          modbus_set_socket(ctx, master_socket);
          static uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH]; // static to prevent need for big stack
          int rc = modbus_receive(ctx, query);
          if(rc > 0) {
            if(!_exception) {
              std::cout << "Incoming request on " << master_socket << " (no exception)\n";
              std::unique_lock<std::mutex> lk(_mx_mapping);
              modbus_reply(ctx, query, rc, &_mapping);
            }
            else {
              std::cout << "Incoming request on " << master_socket << " (exception reply)\n";
              modbus_reply_exception(ctx, query, MODBUS_EXCEPTION_NOT_DEFINED);
            }
          }
          else if(rc == -1) {
            std::cout << "Error on socket " << master_socket << "\n";
            // Currently the connection is closed in case of any error.
            close(master_socket);
            FD_CLR(master_socket, &refset);
          }
        }
      }
    }
  }

  int _serverPort{5555};

  std::mutex _mx_mapping;
  modbus_mapping_t _mapping;

  map_holding _map_holding;
  map_input _map_input;

  std::thread _serverThread;

  std::atomic<bool> _shutdown{false};
  std::atomic<bool> _exception{false};
};
ModbusTestServer testServer;

/**********************************************************************************************************************/

template<typename Derived, typename RAW_USER_TYPE>
struct RegisterDefaults {
  Derived* derived{static_cast<Derived*>(this)};

  typedef RAW_USER_TYPE rawUserType;

  bool isReadable() { return true; }
  ChimeraTK::AccessModeFlags supportedFlags() { return {ChimeraTK::AccessMode::raw}; }
  size_t nChannels() { return 1; }
  size_t writeQueueLength() { return std::numeric_limits<size_t>::max(); }
  size_t nRuntimeErrorCases() { return 1; }

  static constexpr auto capabilities = TestCapabilities<>()
                                           .disableForceDataLossWrite()
                                           .disableAsyncReadInconsistency()
                                           .disableSwitchReadOnly()
                                           .disableSwitchWriteOnly()
                                           .disableTestWriteNeverLosesData();

  template<typename UserType>
  UserType rawToCooked(rawUserType rv) {
    return ChimeraTK::numericToUserType<UserType>(rv / derived->rawPerCooked);
  }

  template<typename UserType>
  std::vector<std::vector<UserType>> generateValue() {
    auto lk = testServer.getLock();
    auto val = testServer.getHolding().*(derived->pReg);
    std::vector<UserType> rval(derived->nElementsPerChannel());
    for(size_t i = 0; i < derived->nElementsPerChannel(); ++i) {
      rval[i] = rawToCooked<UserType>(val[i] + derived->delta + i);
    }
    return {rval};
  }

  template<typename UserType>
  std::vector<std::vector<UserType>> getRemoteValue() {
    auto lk = testServer.getLock();
    auto* val = testServer.getHolding().*(derived->pReg);
    std::vector<UserType> rval(derived->nElementsPerChannel());
    for(size_t i = 0; i < derived->nElementsPerChannel(); ++i) {
      rval[i] = rawToCooked<UserType>(val[i]);
    }
    return {rval};
  }

  void setRemoteValue() {
    auto lk = testServer.getLock();
    for(size_t i = 0; i < derived->nElementsPerChannel(); ++i) {
      (testServer.getHolding().*(derived->pReg))[i] += derived->delta + i;
    }
  }

  void setForceRuntimeError(bool enable, size_t) { testServer.setException(enable); }
};

/**********************************************************************************************************************/

struct HoldingReg1 : RegisterDefaults<HoldingReg1, int16_t> {
  typedef int16_t minimumUserType;

  std::string path() { return "/holding/reg1"; }
  minimumUserType (ModbusTestServer::map_holding::*pReg)[1] = &ModbusTestServer::map_holding::reg1;

  bool isWriteable() { return true; }
  size_t nElementsPerChannel() { return 1; }

  double rawPerCooked = 1.0;
  minimumUserType delta = 42;
};

/**********************************************************************************************************************/

BOOST_AUTO_TEST_CASE(unifiedBackendTest) {
  auto ubt = ChimeraTK::UnifiedBackendTest<>().addRegister<HoldingReg1>();
  ubt.runTests("(modbus:localhost?type=tcp&map=dummy.map&port=" + std::to_string(testServer.serverPort()) + ")");
}
#if 0
/**********************************************************************************************************************/

BOOST_AUTO_TEST_CASE(testReading) {
  ChimeraTK::Device dev(
      "(modbus:localhost?type=tcp&map=dummy.map&port=" + std::to_string(testServer.serverPort()) + ")");
  dev.open();
  BOOST_CHECK(dev.isOpened());

  ModbusTestServer::map_holding holding;
  ModbusTestServer::map_input input;

  input.reg1 = 42;
  input.reg2 = 120;
  holding.reg1 = 1;
  holding.reg2 = 2;
  holding.reg3 = 3;
  holding.reg4_raw = std::round(2.71828 * 255.);
  holding.reg32 = 32323232;
  holding.reg754 = 3.14159265;

  testServer.setInput(input);
  testServer.setHolding(holding);

  // Read single element register
  BOOST_CHECK_EQUAL(dev.read<int>("holding.reg1"), 1);
  BOOST_CHECK_EQUAL(dev.read<int>("holding.reg2"), 2);
  BOOST_CHECK_EQUAL(dev.read<int>("holding.reg3"), 3);
  BOOST_CHECK_CLOSE(dev.read<float>("holding.reg4"), 2.71828, 0.5);
  BOOST_CHECK_EQUAL(dev.read<int>("holding.reg32"), 32323232);
  BOOST_CHECK_CLOSE(dev.read<float>("holding.reg754"), 3.14159265, 0.001);
  BOOST_CHECK_EQUAL(dev.read<int>("input.reg1"), 42);
  BOOST_CHECK_EQUAL(dev.read<int>("input.reg2"), 120);

  dev.close();
  BOOST_CHECK(!dev.isOpened());
}
#endif
/**********************************************************************************************************************/

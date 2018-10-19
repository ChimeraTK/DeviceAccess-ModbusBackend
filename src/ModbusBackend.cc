/*
 * ModbusBackend.cc
 *
 *  Created on: Oct 9, 2018
 *      Author: zenker
 */

#include "ModbusBackend.h"

#include <ChimeraTK/BackendFactory.h>
#include <ChimeraTK/DeviceAccessVersion.h>

#include <bitset>

// You have to define an "extern C" function with this signature. It has to return
// CHIMERATK_DEVICEACCESS_VERSION for version checking when the library is loaded
// at run time. This function is used to determine that this is a valid DeviceAcces
// backend library. Just copy this code, sorry for the boiler plate.
extern "C"{
  const char * deviceAccessVersionUsedToCompile(){
    return CHIMERATK_DEVICEACCESS_VERSION;
  }
}

namespace ChimeraTK{
std::mutex modubus_mutex;

/********************************************************************************************************************/

  ModbusBackend::BackendRegisterer ModbusBackend::gOneWireBackend;

  ModbusBackend::BackendRegisterer::BackendRegisterer() {
    BackendFactory::getInstance().registerBackendType("modbus","",&ModbusBackend::createInstance, CHIMERATK_DEVICEACCESS_VERSION);
    std::cout << "onewire::BackendRegisterer: registered backend type modbus" << std::endl;
  }

/********************************************************************************************************************/

  ModbusBackend::ModbusBackend(std::string IPAddress, int port, std::string mapFileName):
     NumericAddressedBackend(mapFileName), _ctx(nullptr), _IPAddress(IPAddress), _port(port), _opened(false){

  }

  void ModbusBackend::open(){
    if (_opened) {
      throw ChimeraTK::logic_error("Device already has been opened");
    }
    std::cout << "Connecting to: " << _IPAddress.c_str() << ":" << _port << std::endl;
#ifndef DUMMY
    _ctx = modbus_new_tcp(_IPAddress.c_str(), _port);
    if (_ctx == NULL) {
      throw ChimeraTK::runtime_error(std::string("Unable to allocate libmodbus context: ") + modbus_strerror(errno));
    }
    if (modbus_connect(_ctx) == -1) {
      throw ChimeraTK::runtime_error(std::string("Connection failed: ") + modbus_strerror(errno));
      modbus_free(_ctx);
    }
#endif
    _opened = true;
  }

  void ModbusBackend::close(){
#ifndef DUMMY
    if(_opened){
      modbus_close(_ctx);
      modbus_free(_ctx);
    }
#endif
    _opened = false;
  }

  boost::shared_ptr<DeviceBackend> ModbusBackend::createInstance(std::string /*host*/, std::string /*instance*/,
      std::list<std::string> parameters, std::string mapFileName) {

    // check presense of required parameters
    if(parameters.size() > 2 || parameters.size() < 1) {
      throw ChimeraTK::logic_error("modbus::Backend: The SDM URI has not the correct number of parameters: only IP address and port (optional) are supported. "
          "If no port is given MODBUS_TCP_DEFAULT_PORT is considered.");
    }

    // form server address
    std::string IPAddress;
    auto it = parameters.begin();
    IPAddress = *it;
    int port = MODBUS_TCP_DEFAULT_PORT;
    if(parameters.size() == 2){
      it++;
      port = std::stoi(std::string(*it));
    }

    return boost::shared_ptr<DeviceBackend> (new ModbusBackend(IPAddress, port, mapFileName));
  }


  void ModbusBackend::read(uint8_t bar, uint32_t address, int32_t* data,  size_t sizeInBytes){
    std::lock_guard<std::mutex> lock(modubus_mutex);
    size_t length = sizeInBytes/sizeof(int32_t);
    if(length == 0)
      length = 1;
    int32_t toFill[length];
    uint16_t tab_reg[length];
#ifndef DUMMY
    int rc = modbus_read_registers(_ctx, address, length, tab_reg);
    if (rc == -1) {
      std::cerr << "Failed reading address: " << address << " (length: " << length << ")" << std::endl;
      throw ChimeraTK::logic_error(modbus_strerror(errno));
    }
    if(rc != (int)length){
      std::cerr << "Failed reading address: " << address << " (length: " << length << ")" << std::endl;
      throw ChimeraTK::logic_error("modbus::Backend: Not all registers where read...");
    }
#else
//    std::cout << "Attempt to read bar: " << unsigned(bar) << " address: " << address << " sizeInBytes: " << sizeInBytes << std::endl;
//    std::cout << "Filling " << length << " elements." << std::endl;
    int32Touint16 test;
    float myData = 42.42;
    // convert test data to int32_t
    int32_t* pmyData = (int32_t*)&myData;
    test.data32 = *pmyData;

    for(size_t i = 0; i < length; i++){
      if(i%2 == 0)
        tab_reg[i] = test.data16[0];
      else
        tab_reg[i] = test.data16[1];
    }
#endif
    int32Touint16 tmp;
    tmp.data16[1] = 0;
    for(size_t i = 0; i < length; i++){
      tmp.data16[0] = tab_reg[i];
      toFill[i] = tmp.data32;
    }
    memcpy((void*)data, &toFill[0], length*sizeof(int32_t));
    return;
  }

  void ModbusBackend::write(uint8_t bar, uint32_t address, int32_t const* data,  size_t sizeInBytes){
    std::lock_guard<std::mutex> lock(modubus_mutex);
    size_t length = sizeInBytes/sizeof(uint32_t);
    int32Touint16 inputData[length];
    uint16_t tab_reg[length];
    for(size_t i = 0; i < length; i++){
      inputData[i].data32 = data[i];
      tab_reg[i] = inputData[i].data16[0];
    }

    int rc = modbus_write_registers(_ctx, address, length, &tab_reg[0]);
    if(rc != (int)length){
      throw ChimeraTK::logic_error("modbus::Backend: Not all registers where written...");
    }
    return;
  }
}


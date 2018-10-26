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
    BackendFactory::getInstance().registerBackendType("modbus", &ModbusBackend::createInstance, {"type", "map"});
    std::cout << "onewire::BackendRegisterer: registered backend type modbus" << std::endl;
  }

/********************************************************************************************************************/

  ModbusBackend::ModbusBackend(std::string address, ModbusType type, std::map<std::string,std::string> parameters):
     NumericAddressedBackend(parameters["map"]), _ctx(nullptr), _address(address), _parameters(parameters), _opened(false), _type(type){

  }

  void ModbusBackend::open(){
    if (_opened) {
      throw ChimeraTK::logic_error("Device already has been opened");
    }
    if(_type == tcp){
      std::cout << "Connecting to: " << _address.c_str() << ":" << _parameters["port"] << std::endl;
    } else {
    std::cout << "Connecting to: " << _address.c_str() <<
        "\n\t baud rate: " << _parameters["baud"] <<
        "\n\t parity: " << _parameters["parity"] <<
        "\n\t data bits: " << _parameters["data_bits"] <<
        "\n\t stop bits: " << _parameters["stop_bits"] << std:: endl;
    }
#ifndef DUMMY
    if(_type == tcp){
       _ctx = modbus_new_tcp_pi(_address.c_str(), _parameters["port"].c_str());
    } else {
      _ctx = modbus_new_rtu(_address.c_str(),
          std::stoi(_parameters["baud"]),
          _parameters["parity"].c_str()[0],
          std::stoi(_parameters["data_bits"]),
          std::stoi(_parameters["stop_bits"]));
    }
    if (_ctx == NULL) {
      throw ChimeraTK::runtime_error(std::string("Unable to allocate libmodbus context: ") + modbus_strerror(errno));
    }
    if (modbus_connect(_ctx) == -1) {
      throw ChimeraTK::runtime_error(std::string("Connection failed: ") + modbus_strerror(errno));
    }
#else
    std::cout << "Running in test mode" << std::endl;
    std::cout << "Map file is: " << _parameters["map"] <<std::endl;
    std::cout << "Type is: " << _type << std::endl;

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

  void ModbusBackend::reconnect(){
    close();
    open();
  }

  boost::shared_ptr<DeviceBackend> ModbusBackend::createInstance(std::string address, std::map<std::string,std::string> parameters) {
    if(parameters["map"].empty()) {
      throw ChimeraTK::logic_error("Map file name not specified.");
    }
    ModbusType type;
    if(parameters["type"].empty()) {
      throw ChimeraTK::logic_error("No modbus type (rtu/tcp) specified.");
    }

    if(parameters["type"].compare("rtu") == 0)
      type = rtu;
    else if (parameters["type"].compare("tcp") == 0)
      type = tcp;
    else
      throw ChimeraTK::logic_error("Unknown modbus type. Available types are: rtu and tcp.");

    if(type == tcp){
      if(parameters["port"].empty()) {
        parameters["port"] = std::to_string(MODBUS_TCP_DEFAULT_PORT);
      }
    } else {
      if(parameters["parity"].empty())
        parameters["parity"] = "N";
      if(parameters["data_bits"].empty())
        parameters["data_bits"] = "8";
      if(parameters["stop_bits"].empty())
        parameters["stop_bits"] = "1";
    }
    return boost::shared_ptr<DeviceBackend> (new ModbusBackend(address, type, parameters));
  }


  void ModbusBackend::read(uint8_t bar, uint32_t address, int32_t* data,  size_t sizeInBytes){
    if(!_opened)
      reconnect();
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
      // broken pipe
      if(errno == 32){
        _opened = false;
      // connection timed out
      } else if(errno == 110){
        _opened = false;
      }
      throw ChimeraTK::runtime_error(modbus_strerror(errno));
    }
    if(rc != (int)length){
      std::cerr << "Failed reading address: " << address << " (length: " << length << ")" << std::endl;
      throw ChimeraTK::runtime_error("modbus::Backend: Not all registers where read...");
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
    if(!_opened)
      reconnect();
    std::lock_guard<std::mutex> lock(modubus_mutex);
    size_t length = sizeInBytes/sizeof(uint32_t);
    int32Touint16 inputData[length];
    uint16_t tab_reg[length];
    for(size_t i = 0; i < length; i++){
      inputData[i].data32 = data[i];
      tab_reg[i] = inputData[i].data16[0];
    }

    int rc = modbus_write_registers(_ctx, address, length, &tab_reg[0]);
    if (rc == -1) {
      std::cerr << "Failed writing address: " << address << " (length: " << length << ")" << std::endl;
      // broken pipe
      if(errno == 32){
        _opened = false;
      // connection timed out
      } else if(errno == 110){
        _opened = false;
      }
      throw ChimeraTK::runtime_error(modbus_strerror(errno));
    }
    if(rc != (int)length){
      throw ChimeraTK::runtime_error("modbus::Backend: Not all registers where written...");
    }
    return;
  }
}


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
extern "C" {
const char* deviceAccessVersionUsedToCompile() {
  return CHIMERATK_DEVICEACCESS_VERSION;
}
}

namespace ChimeraTK {
  static std::mutex modbus_mutex;

  /********************************************************************************************************************/

  ModbusBackend::BackendRegisterer ModbusBackend::gModbusBackend;

  ModbusBackend::BackendRegisterer::BackendRegisterer() {
    BackendFactory::getInstance().registerBackendType("modbus", &ModbusBackend::createInstance, {"type", "map"});
    std::cout << "modbus::BackendRegisterer: registered backend type modbus" << std::endl;
  }

  /********************************************************************************************************************/

  ModbusBackend::ModbusBackend(std::string address, ModbusType type, std::map<std::string, std::string> parameters)
  : NumericAddressedBackend(parameters["map"]), _ctx(nullptr), _address(address), _parameters(parameters), _type(type) {
    _opened = false;
  }

  /********************************************************************************************************************/

  void ModbusBackend::open() {
    if(_opened) {
      return;
    }
    if(_type == tcp) {
      std::cout << "modbus::Backend: Connecting to: " << _address.c_str() << ":" << _parameters["port"] << std::endl;
    }
    else {
      std::cout << "modbus::Backend: Connecting to: " << _address.c_str() << " slave id:" << _parameters["slaveid"]
                << "\n\t baud rate: " << _parameters["baud"]
                << "\n\t parity: " << _parameters["parity"] << "\n\t data bits: " << _parameters["databits"]
                << "\n\t stop bits: " << _parameters["stopbits"] << std::endl;
    }
#ifndef DUMMY
    if(_type == tcp) {
      _ctx = modbus_new_tcp_pi(_address.c_str(), _parameters["port"].c_str());
    }
    else {
      _ctx = modbus_new_rtu(_address.c_str(), std::stoi(_parameters["baud"]), _parameters["parity"].c_str()[0],
          std::stoi(_parameters["databits"]), std::stoi(_parameters["stopbits"]));
    }
    if(_ctx == NULL) {
      throw ChimeraTK::runtime_error(
          std::string("modbus::Backend: Unable to allocate libmodbus context: ") + modbus_strerror(errno));
    }
    if(modbus_set_slave(_ctx, std::stoi(_parameters["slaveid"])) < 0) {
      throw ChimeraTK::runtime_error(std::string("modbus::Backend: Set slave ID failed: ") + modbus_strerror(errno));
    }
    if(modbus_connect(_ctx) == -1) {
      throw ChimeraTK::runtime_error(std::string("modbus::Backend: Connection failed: ") + modbus_strerror(errno));
    }
#else
    std::cout << "modbus::Backend: Running in test mode" << std::endl;
    std::cout << "modbus::Backend: Map file is: " << _parameters["map"] << std::endl;
    std::cout << "modbus::Backend: Type is: " << _type << std::endl;

#endif
    _opened = true;
    _hasException = false;
  }

  /********************************************************************************************************************/

  void ModbusBackend::close() {
#ifndef DUMMY
    if(_opened) {
      modbus_close(_ctx);
      modbus_free(_ctx);
    }
#endif
    _opened = false;
  }

  /********************************************************************************************************************/

  boost::shared_ptr<DeviceBackend> ModbusBackend::createInstance(
      std::string address, std::map<std::string, std::string> parameters) {
    if(parameters["map"].empty()) {
      throw ChimeraTK::logic_error("modbus::Backend: Map file name not specified.");
    }
    ModbusType type;
    if(parameters["type"].empty()) {
      throw ChimeraTK::logic_error("modbus::Backend: No modbus type (rtu/tcp) specified.");
    }

    if(parameters["type"].compare("rtu") == 0)
      type = rtu;
    else if(parameters["type"].compare("tcp") == 0)
      type = tcp;
    else
      throw ChimeraTK::logic_error("modbus::Backend: Unknown modbus type. Available types are: rtu and tcp.");

    if(type == tcp) {
      if(parameters["port"].empty()) {
        parameters["port"] = std::to_string(MODBUS_TCP_DEFAULT_PORT);
      }
      if(parameters["slaveid"].empty()) {
        parameters["slaveid"] = std::to_string(MODBUS_TCP_SLAVE);
      }
    }
    else {
      if(parameters["baud"].empty()) parameters["baud"] = "115200";
      if(parameters["parity"].empty()) parameters["parity"] = "N";
      if(parameters["databits"].empty()) parameters["databits"] = "8";
      if(parameters["stopbits"].empty()) parameters["stopbits"] = "1";
      if(parameters["slaveid"].empty()) parameters["slaveid"] = "1";
    }
    return boost::shared_ptr<DeviceBackend>(new ModbusBackend(address, type, parameters));
  }

  /********************************************************************************************************************/

  void ModbusBackend::read(uint64_t bar, uint64_t addressInBytes, int32_t* data, size_t sizeInBytes) {
    if(_hasException) {
      throw ChimeraTK::runtime_error("previous error detected.");
    }
    std::lock_guard<std::mutex> lock(modbus_mutex);
    if(addressInBytes % 2 != 0) {
      throw ChimeraTK::runtime_error("Address must be multiple of 2.");
    }
    size_t address = addressInBytes / 2;
    size_t length = sizeInBytes / 2;
    if(length == 0) length = 1;

    int rc;
    if(bar == 3) {
      rc = modbus_read_registers(_ctx, address, length, (uint16_t*)data);
    }
    else if(bar == 4) {
      rc = modbus_read_input_registers(_ctx, address, length, (uint16_t*)data);
    }
    else {
      throw ChimeraTK::runtime_error(
          "Bar number " + std::to_string((int)bar) + " is not supported by the ModbusBackend.");
    }

    if(rc == -1) {
      std::cerr << "modbus::Backend: Failed reading address: " << address << " (length: " << length << ")" << std::endl;
      _hasException = true;
      throw ChimeraTK::runtime_error(modbus_strerror(errno));
    }
    if(rc != (int)length) {
      std::cerr << "modbus::Backend: Failed reading address: " << address << " (length: " << length << ")" << std::endl;
      _hasException = true;
      throw ChimeraTK::runtime_error("modbus::Backend: Not all registers where read...");
    }

    return;
  }

  /********************************************************************************************************************/

  void ModbusBackend::write(uint64_t bar, uint64_t addressInBytes, int32_t const* data, size_t sizeInBytes) {
    if(_hasException) {
      throw ChimeraTK::runtime_error("previous error detected.");
    }
    std::lock_guard<std::mutex> lock(modbus_mutex);
    size_t address = addressInBytes / 2;
    size_t length = sizeInBytes / 2;

    int rc;
    if(bar == 3) {
      if(length == 1) {
        rc = modbus_write_register(_ctx, address, *((uint16_t*)data));
      }
      else {
        rc = modbus_write_registers(_ctx, address, length, (uint16_t*)data);
      }
    }
    else {
      throw ChimeraTK::runtime_error(
          "Writing bar number " + std::to_string((int)bar) + " is not supported by the ModbusBackend.");
    }
    if(rc == -1) {
      std::cerr << "modbus::Backend: Failed writing address: " << address << " (length: " << length << ")" << std::endl;
      _hasException = true;
      throw ChimeraTK::runtime_error(modbus_strerror(errno));
    }
    if(rc != (int)length) {
      _hasException = true;
      throw ChimeraTK::runtime_error("modbus::Backend: Not all registers where written...");
    }
    return;
  }

  /********************************************************************************************************************/

  bool ModbusBackend::barIndexValid(uint64_t bar) {
    return (bar == 3) || (bar == 4);
  }

  /********************************************************************************************************************/

  bool ModbusBackend::isFunctional() const {
    if(_opened && !_hasException)
      return true;
    else
      return false;
  }

  /********************************************************************************************************************/

} // namespace ChimeraTK

// SPDX-FileCopyrightText: Helmholtz-Zentrum Dresden-Rossendorf, FWKE, ChimeraTK Project <chimeratk-support@desy.de>
// SPDX-License-Identifier: LGPL-3.0-or-later
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
    std::cout << "ModbusBackendRegisterer: registered backend type modbus" << std::endl;
  }

  /********************************************************************************************************************/

  ModbusBackend::ModbusBackend(std::string address, ModbusType type, std::map<std::string, std::string> parameters)
  : NumericAddressedBackend(parameters["map"]), _ctx(nullptr), _address(std::move(address)),
    _parameters(std::move(parameters)), _type(type) {
    _opened = false;

    auto disable_merging_str = _parameters.find("disableMerging");
    if(disable_merging_str != _parameters.end()) {
      _mergingEnabled = (std::stoi(disable_merging_str->second) == 0);
    }
  }

  /********************************************************************************************************************/

  void ModbusBackend::open() {
    if(_ctx == nullptr) {
      if(_type == tcp) {
        _ctx = modbus_new_tcp_pi(_address.c_str(), _parameters["port"].c_str());
      }
      else {
        _ctx = modbus_new_rtu(_address.c_str(), std::stoi(_parameters["baud"]), *_parameters["parity"].c_str(),
            std::stoi(_parameters["databits"]), std::stoi(_parameters["stopbits"]));
      }
      if(_ctx == nullptr) {
        throw ChimeraTK::logic_error(
            std::string("ModbusBackend: Unable to create libmodbus context: ") + modbus_strerror(errno));
      }
      if(modbus_set_slave(_ctx, std::stoi(_parameters["slaveid"])) < 0) {
        throw ChimeraTK::logic_error(std::string("ModbusBackend: Set slave ID failed: ") + modbus_strerror(errno));
      }
      if(modbus_connect(_ctx) == -1) {
        // If server cannot be reached, errors ECONNREFUSED and EINPROGRESS might alternate. This will create
        // many log messages e.g. in ApplicationCore and hence is here "filtered". EINPROGRESS is anyway a bit
        // misleading in this context and hence is replaced with ECONNREFUSED.
        auto error = errno;
        if(error == EINPROGRESS) error = ECONNREFUSED;
        auto message = std::string("ModbusBackend: Connection failed: ") + modbus_strerror(error);
        setException(message);
        closeConnection();
        throw ChimeraTK::runtime_error(message);
      }
    }
    setOpenedAndClearException();

    // verify connection is ok again with a dummy read of the address which failed last.
    if(_lastFailedAddress.has_value()) {
      int32_t temp;
      size_t dummyReadSize = _lastFailedAddress->first <= 1 ? 1 : 2; // depends on bar
      try {
        read(_lastFailedAddress->first, _lastFailedAddress->second, &temp, dummyReadSize);
      }
      catch(ChimeraTK::runtime_error& ex) {
        setException(ex.what());
        throw;
      }
    }
  }

  /********************************************************************************************************************/

  void ModbusBackend::closeImpl() {
    if(_opened) {
      _opened = false;
      closeConnection();
    }
  }

  /********************************************************************************************************************/

  void ModbusBackend::closeConnection() {
    if(_ctx != nullptr) {
      modbus_close(_ctx);
      modbus_free(_ctx);
      _ctx = nullptr;
    }
  }

  /********************************************************************************************************************/

  size_t ModbusBackend::minimumTransferAlignment(uint64_t bar) const {
    if(bar == 3 || bar == 4) {
      return 2;
    }
    return 1;
  }

  /********************************************************************************************************************/

  boost::shared_ptr<DeviceBackend> ModbusBackend::createInstance(
      std::string address, std::map<std::string, std::string> parameters) {
    if(parameters["map"].empty()) {
      throw ChimeraTK::logic_error("ModbusBackend: Map file name not specified.");
    }
    ModbusType type;
    if(parameters["type"].empty()) {
      throw ChimeraTK::logic_error("ModbusBackend: No modbus type (rtu/tcp) specified.");
    }

    if(parameters["type"] == "rtu") {
      type = rtu;
    }
    else if(parameters["type"] == "tcp") {
      type = tcp;
    }
    else {
      throw ChimeraTK::logic_error("ModbusBackend: Unknown modbus type. Available types are: rtu and tcp.");
    }

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
    return boost::shared_ptr<DeviceBackend>(new ModbusBackend(std::move(address), type, parameters));
  }

  /********************************************************************************************************************/

  void ModbusBackend::read(uint64_t bar, uint64_t addressInBytes, int32_t* data, size_t sizeInBytes) {
    checkActiveException();

    if(addressInBytes > size_t(std::numeric_limits<int>::max())) {
      throw ChimeraTK::logic_error("Requested read address exceeds maximum.");
    }
    auto address = static_cast<int>(addressInBytes);

    if(sizeInBytes > size_t(std::numeric_limits<int>::max())) {
      throw ChimeraTK::logic_error("Requested read length exceeds maximum.");
    }
    auto length = static_cast<int>(sizeInBytes);

    std::lock_guard<std::mutex> lock(modbus_mutex);

    if(bar == 3 || bar == 4) {
      assert(address % 2 == 0); // guaranteed via minimumTransferAlignment()
      assert(length % 2 == 0);
      address /= 2;
      length /= 2;
    }
    if(length == 0) length = 1;

    int rc;
    if(bar == 0) {
      rc = modbus_read_bits(_ctx, address, length, static_cast<uint8_t*>(static_cast<void*>(data)));
    }
    else if(bar == 1) {
      rc = modbus_read_input_bits(_ctx, address, length, static_cast<uint8_t*>(static_cast<void*>(data)));
    }
    else if(bar == 3) {
      rc = modbus_read_registers(_ctx, address, length, static_cast<uint16_t*>(static_cast<void*>(data)));
    }
    else if(bar == 4) {
      rc = modbus_read_input_registers(_ctx, address, length, static_cast<uint16_t*>(static_cast<void*>(data)));
    }
    else {
      throw ChimeraTK::logic_error(
          "Bar number " + std::to_string((int)bar) + " is not supported by the ModbusBackend.");
    }

    if(rc != (int)length) {
      _lastFailedAddress = {bar, addressInBytes};
      std::string modbusError;
      if(rc == -1) {
        modbusError = modbus_strerror(errno);
      }
      else {
        modbusError = "Not all registers were transferred.";
      }
      throw ChimeraTK::runtime_error("ModbusBackend failed reading address (" + std::to_string(bar) + "," +
          std::to_string(address) + ") length " + std::to_string(length) + ": " + modbusError);
    }
  }

  /********************************************************************************************************************/

  void ModbusBackend::write(uint64_t bar, uint64_t addressInBytes, int32_t const* data, size_t sizeInBytes) {
    checkActiveException();

    if(addressInBytes > size_t(std::numeric_limits<int>::max())) {
      throw ChimeraTK::logic_error("Requested write address exceeds maximum.");
    }
    auto address = static_cast<int>(addressInBytes);

    if(sizeInBytes > size_t(std::numeric_limits<int>::max())) {
      throw ChimeraTK::logic_error("Requested write length exceeds maximum.");
    }
    auto length = static_cast<int>(sizeInBytes);

    std::lock_guard<std::mutex> lock(modbus_mutex);

    if(bar == 3) {
      assert(address % 2 == 0); // guaranteed via minimumTransferAlignment()
      assert(length % 2 == 0);
      address /= 2;
      length /= 2;
    }
    if(length == 0) length = 1;

    int rc;
    if(bar == 0) {
      if(length == 1) {
        rc = modbus_write_bit(_ctx, address, *(static_cast<const uint8_t*>(static_cast<const void*>(data))));
      }
      else {
        rc = modbus_write_bits(_ctx, address, length, static_cast<const uint8_t*>(static_cast<const void*>(data)));
      }
    }
    else if(bar == 3) {
      if(length == 1) {
        rc = modbus_write_register(_ctx, address, *(static_cast<const uint16_t*>(static_cast<const void*>(data))));
      }
      else {
        rc =
            modbus_write_registers(_ctx, address, length, static_cast<const uint16_t*>(static_cast<const void*>(data)));
      }
    }
    else {
      throw ChimeraTK::runtime_error(
          "Writing bar number " + std::to_string((int)bar) + " is not supported by the ModbusBackend.");
    }
    if(rc != (int)length) {
      _lastFailedAddress = {bar, addressInBytes};
      std::string modbusError;
      if(rc == -1) {
        modbusError = modbus_strerror(errno);
      }
      else {
        modbusError = "Not all registers were transferred.";
      }
      throw ChimeraTK::runtime_error("ModbusBackend failed writing address (" + std::to_string(bar) + "," +
          std::to_string(address) + ") length " + std::to_string(length) + ": " + modbusError);
    }
  }

  /********************************************************************************************************************/

  bool ModbusBackend::barIndexValid(uint64_t bar) {
    return (bar == 0) || (bar == 1) || (bar == 3) || (bar == 4);
  }

  /********************************************************************************************************************/

  void ModbusBackend::setExceptionImpl() noexcept {
    closeConnection();
  }

} // namespace ChimeraTK

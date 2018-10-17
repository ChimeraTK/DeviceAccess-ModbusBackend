/*
 * ModbusBackend.h
 *
 *  Created on: Oct 9, 2018
 *      Author: zenker
 */

#ifndef INCLUDE_MODBUSBACKEND_H_
#define INCLUDE_MODBUSBACKEND_H_

#include <cerrno>
#include <string>

#include "modbus/modbus.h"

#include "ChimeraTK/NumericAddressedBackend.h"

namespace ChimeraTK {
  /**
   * Union used to put uint16_t data into int32_t.
   */
  union int32Touint16{
    uint16_t data16[2];
    int32_t  data32;
  };
  /** A class to provide the modbus backend."
   *
   */
  class ModbusBackend : public NumericAddressedBackend {
  public:
    ModbusBackend(std::string IPAddress, int port, std::string mapFileName);
    virtual ~ModbusBackend(){close();};

    virtual void open() override;
    virtual void close() override;

    virtual void read(uint8_t bar, uint32_t address, int32_t* data,  size_t sizeInBytes) override;
    virtual void write(uint8_t bar, uint32_t address, int32_t const* data,  size_t sizeInBytes) override;

    virtual std::string readDeviceInfo() override {return "Modbus device";};

    static boost::shared_ptr<DeviceBackend> createInstance(std::string host, std::string instance,
        std::list<std::string> parameters, std::string mapFileName);


    /** Class to register the backend type with the factory. */
    class BackendRegisterer {
      public:
        BackendRegisterer();
    };
    static BackendRegisterer gOneWireBackend;
  private:
    modbus_t *_ctx;
    std::string _IPAddress;
    int _port;
    bool _opened;
  };
}



#endif /* INCLUDE_MODBUSBACKEND_H_ */

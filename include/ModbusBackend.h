// SPDX-FileCopyrightText: Helmholtz-Zentrum Dresden-Rossendorf, FWKE, ChimeraTK Project <chimeratk-support@desy.de>
// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

/*
 * ModbusBackend.h
 *
 *  Created on: Oct 9, 2018
 *      Author: zenker
 */

#include "ChimeraTK/NumericAddressedBackend.h"
#include "modbus/modbus.h"

#include <cerrno>
#include <mutex>
#include <optional>
#include <string>

namespace ChimeraTK {
  /**
   * Union used to put uint16_t data into int32_t.
   */
  union int32Touint16 {
    uint16_t data16[2];
    int32_t data32;
  };

  enum ModbusType { rtu, tcp };
  /** A class to provide the modbus backend."
   *
   */
  class ModbusBackend : public NumericAddressedBackend {
   public:
    ModbusBackend(std::string address, ModbusType _type, std::map<std::string, std::string> parameters);
    ~ModbusBackend() override { close(); };

    void open() override;
    void closeImpl() override;
    void closeConnection();
    bool canMergeRequests() const override { return _mergingEnabled; }
    size_t minimumTransferAlignment(uint64_t bar) const override;
    void read(uint64_t bar, uint64_t address, int32_t* data, size_t sizeInBytes) override;
    void write(uint64_t bar, uint64_t address, int32_t const* data, size_t sizeInBytes) override;
    bool barIndexValid(uint64_t bar) override;

    std::string readDeviceInfo() override { return "Modbus device"; };

    static boost::shared_ptr<DeviceBackend> createInstance(
        std::string address, std::map<std::string, std::string> parameters);

    /** Class to register the backend type with the factory. */
    class BackendRegisterer {
     public:
      BackendRegisterer();
    };
    static BackendRegisterer gModbusBackend;

    void setExceptionImpl() noexcept override;

   private:
    // governs access to _ctx
    std::mutex _modbus_mutex;

    modbus_t* _ctx{nullptr};
    std::string _address;
    std::map<std::string, std::string> _parameters;
    ModbusType _type;
    bool _mergingEnabled{true};

    // Address of last exception - used to check whether the exception has been recovered in open()
    std::optional<std::pair<uint64_t, uint64_t>> _lastFailedAddress;
  };
} // namespace ChimeraTK

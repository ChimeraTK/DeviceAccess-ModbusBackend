/*
 * testServer.C
 *
 *  Created on: Oct 10, 2018
 *      Author: zenker
 */

#include "ChimeraTK/ApplicationCore/ApplicationCore.h"
#include "ChimeraTK/ApplicationCore/PeriodicTrigger.h"
#include <ChimeraTK/ApplicationCore/Logging.h>

#include "ChimeraTK/Device.h"

#include <string>

namespace ctk = ChimeraTK;

struct ModbusModule: public ctk::ApplicationModule {
  using ctk::ApplicationModule::ApplicationModule;
  /**
   * \remark
   * Observe this variable by other modules to obtain a trigger
   */
  ctk::ScalarOutput<float> current { this, "SSPA", "", "SSPA Current",
    { "SSPA", "CS" }};

  /** Trigger used to trigger an update of 1-wire sensor data.*/
  ctk::ScalarPushInput<uint64_t> trigger {this, "Trigger", "", "Trigger used to trigger an update of 1-wire sensor data"};

  /** Trigger used to trigger an update of 1-wire sensor data.*/
  ctk::ScalarPushInput<int> current_in {this, "current", "", "Current value"};

  logging::Logger logger{this};
  /**
   * Application core main loop.
   */
  void mainLoop() override {
    while(1){
      trigger.read();
      current_in.read();
      current = current_in;
      current.write();
      logger.sendMessage(std::string("Triggered..."), logging::DEBUG);
    }
  }
};

struct ModbusTestServer: public ctk::Application {
  ModbusTestServer():
    Application("ModbusTestServer"){};
  ~ModbusTestServer() {
    shutdown();
  }
  ctk::ControlSystemModule cs;

  logging::LoggingModule log{this, "Logging", "Logging of 1-wire devices"};

  ModbusModule mod{this, "Modbus", "Modbus test module"};
  ctk::PeriodicTrigger trigger{this, "Trigger", "Trigger used for other modules"};

  ChimeraTK::DeviceModule dev{"RF", ""};

  void defineConnections() override{
    ChimeraTK::setDMapFilePath("devMapFile.dmap");

    ctk::VariableNetworkNode current = dev("current/idc_0", typeid(int), 1, ctk::UpdateMode::push);
    current >> mod.current_in;

    trigger.tick >> mod.trigger;
    cs["configuration"]("timeout") >> trigger.timeout;

    mod.findTag("CS").connectTo(cs);

    //logging
    log.addSource(&mod.logger);
    log.findTag("CS").connectTo(cs["Logging"]);
    cs["Logging"]("TailLength") >> log.tailLength;
    cs["Logging"]("TargetStream") >> log.targetStream;
    cs["Logging"]("LogFile") >> log.logFile;
    cs["Logging"]("LogLevel") >> log.logLevel;

  }
};

// define the application instance
ModbusTestServer theServer;

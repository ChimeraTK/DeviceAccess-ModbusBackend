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
  ModbusModule(EntityOwner *owner, const std::string &name,
        const std::string &description, bool eliminateHierarchy = false,
        const std::unordered_set<std::string> &tags = { }): ctk::ApplicationModule(owner, name, description, eliminateHierarchy, tags){
    ctk::setDMapFilePath("devMapFile.dmap");
    ctk::Device dev;
    dev.open("SSPA");
    auto cat = dev.getRegisterCatalogue();
    for(auto it = cat.begin(); it != cat.end(); it++){
      variables.emplace_back(std::make_pair((*it).getRegisterName(),(*it).getNumberOfElements()));
      if((*it).getNumberOfElements() == 2){
        floatDataIn.emplace_back(this, ((std::string)(*it).getRegisterName()+"in").c_str(), "", 2, "");
        // \ToDo: tag assignment does not work in constructor here but why
        floatDataOut.emplace_back( this, ((std::string)(*it).getRegisterName()).c_str(), "", "te");
        floatDataOut.back().addTag("CS");
      }
    }
    dev.close();
  }

  std::vector<std::pair<ctk::RegisterPath, uint> > variables;

  union Helper{
    uint16_t data16[2];
    int32_t  idata;
    float  fdata;
  };

  /**
   * \remark
   * Observe this variable by other modules to obtain a trigger
   */

  /** Trigger used to trigger an update of 1-wire sensor data.*/
  ctk::ScalarPushInput<uint64_t> trigger {this, "Trigger", "", "Trigger used to trigger an update of 1-wire sensor data"};

  /** Trigger used to trigger an update of 1-wire sensor data.*/
  std::vector<ctk::ArrayPollInput<int32_t> > floatDataIn;
  std::vector<ctk::ScalarOutput<float> > floatDataOut;

  logging::Logger logger{this};
  /**
   * Application core main loop.
   */
  void mainLoop() override {
    while(1){
      trigger.read();

      auto it1 = floatDataIn.begin();
      auto it2 = floatDataOut.begin();
      while(it1 != floatDataIn.end()){
        (*it1).read();
        Helper s[2];
        s[0].idata = (*it1).data()[0];
        s[1].idata = (*it1).data()[1];
        Helper result;
        result.data16[0] = s[0].data16[0];
        result.data16[1] = s[1].data16[0];
        std::stringstream ss;
        ss << "Result is: " << result.fdata << std::endl;
        (*it2) = result.fdata;
        (*it2).write();
        logger.sendMessage(ss.str(), logging::INFO);
        it1++;
        it2++;
      }
    }
  }
};

struct ModbusTestServer: public ctk::Application {
  ModbusTestServer():
    Application("ModbusTestServer"){
  };

  ~ModbusTestServer() {
    shutdown();
  }
  ctk::ControlSystemModule cs;

  logging::LoggingModule log{this, "Logging", "Logging of 1-wire devices"};

  ModbusModule mod{this, "Modbus", "Modbus test module"};
  ctk::PeriodicTrigger trigger{this, "Trigger", "Trigger used for other modules"};

  ctk::DeviceModule dev{"SSPA", ""};

  void defineConnections() override{
//    ctk::setDMapFilePath("devMapFile.dmap");

    ctk::VariableNetworkNode current[2];
    current[0] = dev("Device/current/idc_1", typeid(int), 2, ctk::UpdateMode::poll);
    current[1] = dev("sspa/RF_SLICE/1/current_1", typeid(int), 2, ctk::UpdateMode::poll);

    auto itOut = mod.floatDataIn.begin();
    for(auto it = mod.variables.begin(); it != mod.variables.end(); it++){
      if((*it).second == 2){
        ctk::VariableNetworkNode node = dev((*it).first, typeid(int), 2, ctk::UpdateMode::poll);
        node >> (*itOut);
        itOut++;
      }
    }
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

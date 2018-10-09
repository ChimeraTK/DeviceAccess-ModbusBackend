/*
 * testModbusConnection.C
 *
 *  Created on: Oct 9, 2018
 *      Author: zenker
 */
#include <iostream>
#include "modbus/modbus.h"
#include <cerrno>

using namespace std;

union S{
  uint16_t data[2];
  float fdata;
};

int main(){
  uint16_t tab_reg[32];
  modbus_t *ctx;

  ctx = modbus_new_tcp("127.001", MODBUS_TCP_DEFAULT_PORT);
  if (ctx == NULL) {
    cerr <<  "Unable to allocate libmodbus context: " << modbus_strerror(errno) << endl;
  }

  if (modbus_connect(ctx) == -1) {
    cerr <<  "Connection failed: " << modbus_strerror(errno) << endl;
    modbus_free(ctx);
  } else {
    /* Read 5 registers from the address 0 */
    int rc = modbus_read_registers(ctx, 14, 28, tab_reg);
    if (rc == -1) {
      cerr << modbus_strerror(errno) << endl;
    }
    if(rc != 28){
      cerr << "Not all registers where read..." << endl;
    }

    modbus_close(ctx);
    modbus_free(ctx);
  }
  for(size_t i = 0; i < 8; i++){
    cout << "Current IDC Nr." << i << ": " << modbus_get_float(&tab_reg[2+i*2]) << endl;
  }

  S test;
  test.fdata = 42.42;

  tab_reg[2] = test.data[0];
  tab_reg[3] = test.data[1];
  cout << "Test: " << modbus_get_float(&tab_reg[2]) << endl;
}


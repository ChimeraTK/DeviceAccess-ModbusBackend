/*
 * testModbus.cc
 *
 *  Created on: Mar 19, 2020
 *      Author: Klaus Zenker (HZDR)
 */

//#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE ModbusTest


#include <boost/test/included/unit_test.hpp>
#include <boost/test/unit_test.hpp>
using namespace boost::unit_test_framework;

#include <ChimeraTK/Device.h>

union UFloat{
  uint16_t data16[2];
  int32_t  idata;
  float  fdata;
};

BOOST_AUTO_TEST_CASE(testReading) {
  ChimeraTK::setDMapFilePath("devMapFile.dmap");
  ChimeraTK::Device dev;
  dev.open("Dummy");
  auto reg = dev.getOneDRegisterAccessor<int32_t>("data.float",2,0);
  reg.read();
  BOOST_CHECK_EQUAL(44364, reg.data()[0]);
  BOOST_CHECK_EQUAL(16937, reg.data()[1]);
  UFloat data[2];
  data[0].idata = reg.data()[0];
  data[1].idata = reg.data()[1];
  UFloat result;
  result.data16[0] = data[0].data16[0];
  result.data16[1] = data[1].data16[0];
  BOOST_CHECK_EQUAL(42.42, result.fdata);

}

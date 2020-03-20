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
#include "ModbusBackend.h"

union UFloat{
  uint16_t data16[2];
  int32_t  idata;
  float  fdata;
};

BOOST_AUTO_TEST_CASE(testReading) {
  ChimeraTK::setDMapFilePath("devMapFile.dmap");
  ChimeraTK::Device dev;
  dev.open("Dummy");
  BOOST_CHECK(dev.isOpened());

  // Read single element register
  BOOST_CHECK_EQUAL(dev.read<int>("data.uint16"), 44564);

  // Read single element from multi-element register -> only gives the first element
  BOOST_CHECK_EQUAL(dev.read<int>("data.float"), 44564);

  // We need to use a OneDRegisterAccessor to read all elements
  // wrong element size
  BOOST_CHECK_THROW(dev.getOneDRegisterAccessor<int32_t>("data.float",3), ChimeraTK::logic_error);
  // correct element size will be automatically read from catalog if nElements is 0.
  // You could also put 2 to manually choose the correct size.
  auto reg = dev.getOneDRegisterAccessor<int32_t>("data.float");
  reg.read();
  BOOST_CHECK_EQUAL(44564, reg.data()[0]);
  BOOST_CHECK_EQUAL(16937, reg.data()[1]);

  // Now we convert to float - just to demonstrate how it works
  UFloat data[2];
  data[0].idata = reg.data()[0];
  data[1].idata = reg.data()[1];
  UFloat result;
  result.data16[0] = data[0].data16[0];
  result.data16[1] = data[1].data16[0];
  BOOST_CHECK_CLOSE(42.42, result.fdata,0.001);

  dev.close();
  BOOST_CHECK(!dev.isOpened());
}

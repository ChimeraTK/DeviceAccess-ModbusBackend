project(DeviceAccess-Modbus)
cmake_minimum_required(VERSION 2.8)

set(${PROJECT_NAME}_MAJOR_VERSION 00)
set(${PROJECT_NAME}_MINOR_VERSION 00)
set(${PROJECT_NAME}_PATCH_VERSION 01)
include(cmake/set_version_numbers.cmake)

#Moderate version of the compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fPIC -Wextra -ansi -Wuninitialized")

#use -DCMAKE_BUILD_TYPE=Debug in your cmake command to turn on the coverage option
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -ggdb")

# linker flags
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--export-dynamic")

FIND_PACKAGE(Boost COMPONENTS thread system REQUIRED)

find_library(MODBUS  libmodbus NAMES libmodbus.so.5)

if(MODBUS)
  message(STATUS "libmodbus was found: ${MODBUS}")
else()
  message(FATAL_ERROR "lobmodbus was not found. Consider installing the package: libmodbus-dev")
endif(MODBUS)

include(cmake/add_dependency.cmake)
add_dependency(mtca4u-deviceaccess 00.29 REQUIRED)

include_directories( include )

aux_source_directory(${CMAKE_SOURCE_DIR}/src library_sources)

#add_library(${PROJECT_NAME} SHARED ${library_sources} )
#set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_FULL_LIBRARY_VERSION} SOVERSION ${${PROJECT_NAME}_SOVERSION})
#set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${mtca4u-deviceaccess_CXX_FLAGS}")
#set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_FLAGS "-Wl,--enable-new-dtags")
#target_link_libraries(${PROJECT_NAME} ${mtca4u-deviceaccess_LIBRARIES})

FIND_PACKAGE(Boost COMPONENTS unit_test_framework)

if(Boost_UNIT_TEST_FRAMEWORK_FOUND)
  ENABLE_TESTING()
  add_executable(testModbusConnection ${CMAKE_SOURCE_DIR}/test/testModbusConnection.C )
  target_link_libraries(testModbusConnection ${MODBUS})
  add_test(testModbusConnection testModbusConnection)
  
  add_executable(testMapping ${CMAKE_SOURCE_DIR}/test/testMapping.C )
  target_link_libraries(testMapping ${MODBUS} ${mtca4u-deviceaccess_LIBRARIES})
  add_test(testMapping testMapping)
else()
message(STATUS "Boost unit test framework not found. No tests will be build.")
endif()
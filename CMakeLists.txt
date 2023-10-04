project(ChimeraTK-DeviceAccess-ModbusBackend)
cmake_minimum_required(VERSION 3.16)

option(BUILD_TESTS "Build tests." ON)

set(${PROJECT_NAME}_MAJOR_VERSION 01)
set(${PROJECT_NAME}_MINOR_VERSION 04)
set(${PROJECT_NAME}_PATCH_VERSION 00)
include(cmake/set_version_numbers.cmake)

include(cmake/set_default_build_to_release.cmake)
include(cmake/set_default_flags.cmake)
include(cmake/enable_code_coverage_report.cmake)

# optional dependency: BOOST unit test framework needed for tests
find_package(Boost COMPONENTS unit_test_framework)

if(BUILD_TESTS)
  if(Boost_UNIT_TEST_FRAMEWORK_FOUND)
    set(TESTING_IS_ENABLED "true")
    ENABLE_TESTING()
  else()
    message(" The following packages were not found, so testing will be disabled:")
    message("  * BOOST unit_test_framework")
  endif()
endif(BUILD_TESTS)

find_library(MODBUS libmodbus NAMES libmodbus.so)

if(MODBUS)
  message(STATUS "libmodbus was found: ${MODBUS}")
else()
  message(FATAL_ERROR "libmodbus was not found. Consider installing the package: libmodbus-dev")
endif(MODBUS)

find_package(ChimeraTK-DeviceAccess 02.06 REQUIRED)

include_directories(include)

aux_source_directory(${CMAKE_SOURCE_DIR}/src library_sources)

add_library(${PROJECT_NAME} SHARED ${library_sources})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_FULL_LIBRARY_VERSION} SOVERSION ${${PROJECT_NAME}_SOVERSION})
target_link_libraries(${PROJECT_NAME} PUBLIC ChimeraTK::ChimeraTK-DeviceAccess PRIVATE ${MODBUS})

# Create the executables for automated unit testing.
if(TESTING_IS_ENABLED)
  add_executable(testModbus ${library_sources} tests/testModbus.C)
  target_link_libraries(testModbus PUBLIC ChimeraTK::ChimeraTK-DeviceAccess PUBLIC ${Boost_LIBRARIES} PRIVATE ${MODBUS})
  add_test(testModbus testModbus)

  file(COPY ${CMAKE_SOURCE_DIR}/tests/dummy.map DESTINATION ${PROJECT_BINARY_DIR})
  file(COPY ${CMAKE_SOURCE_DIR}/tests/devMapFile.dmap DESTINATION ${PROJECT_BINARY_DIR})
endif(TESTING_IS_ENABLED)

# Note: Backends do not need to install their header files, so just install the library.
include(GNUInstallDirs)
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

# we support our cmake EXPORTS as imported targets
set(PROVIDES_EXPORTED_TARGETS 1)
include(${CMAKE_SOURCE_DIR}/cmake/create_cmake_config_files.cmake)

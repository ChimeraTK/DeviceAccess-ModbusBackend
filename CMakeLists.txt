cmake_minimum_required(VERSION 3.16)
project(ChimeraTK-DeviceAccess-ModbusBackend)

option(BUILD_TESTS "Build tests." ON)

set(${PROJECT_NAME}_MAJOR_VERSION 01)
set(${PROJECT_NAME}_MINOR_VERSION 05)
set(${PROJECT_NAME}_PATCH_VERSION 01)
include(cmake/set_version_numbers.cmake)

include(cmake/set_default_build_to_release.cmake)
include(cmake/set_default_flags.cmake)
include(cmake/enable_code_coverage_report.cmake)
include(cmake/enable_code_style_check.cmake)
include(cmake/add_linter_target.cmake)

find_library(MODBUS libmodbus NAMES libmodbus.so)

if(MODBUS)
  message(STATUS "libmodbus was found: ${MODBUS}")
else()
  message(FATAL_ERROR "libmodbus was not found. Consider installing the package: libmodbus-dev")
endif(MODBUS)

find_package(ChimeraTK-DeviceAccess 03.14 REQUIRED)

include_directories(include)

aux_source_directory(${CMAKE_SOURCE_DIR}/src library_sources)

add_library(${PROJECT_NAME} SHARED ${library_sources})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_FULL_LIBRARY_VERSION} SOVERSION ${${PROJECT_NAME}_SOVERSION})
target_link_libraries(${PROJECT_NAME} PUBLIC ChimeraTK::ChimeraTK-DeviceAccess PRIVATE ${MODBUS})

if(BUILD_TESTS)
  add_subdirectory(tests)
endif(BUILD_TESTS)

# Note: Backends do not need to install their header files, so just install the library.
include(GNUInstallDirs)
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

# we support our cmake EXPORTS as imported targets
set(PROVIDES_EXPORTED_TARGETS 1)
include(${CMAKE_SOURCE_DIR}/cmake/create_cmake_config_files.cmake)

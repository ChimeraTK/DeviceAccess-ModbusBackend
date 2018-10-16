project(ChimeraTK-DeviceAccess-ModbusBackend)
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
  message(FATAL_ERROR "libmodbus was not found. Consider installing the package: libmodbus-dev")
endif(MODBUS)

include(cmake/add_dependency.cmake)
add_dependency(ChimeraTK-DeviceAccess 00.29 REQUIRED)

include_directories( include )

aux_source_directory(${CMAKE_SOURCE_DIR}/src library_sources)

add_library(${PROJECT_NAME} SHARED ${library_sources} )
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_FULL_LIBRARY_VERSION} SOVERSION ${${PROJECT_NAME}_SOVERSION})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${ChimeraTK-DeviceAccess_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_FLAGS "-Wl,--enable-new-dtags")
target_link_libraries(${PROJECT_NAME} ${ChimeraTK-DeviceAccess_LIBRARIES} ${MODBUS} )

#FILE( COPY ${CMAKE_SOURCE_DIR}/config/sigma_phi_FI004250_RF_SLICE.xlmap DESTINATION ${PROJECT_BINARY_DIR})
#FILE( COPY ${CMAKE_SOURCE_DIR}/config/sigma_phi_FI004250.map DESTINATION ${PROJECT_BINARY_DIR})
#FILE( COPY ${CMAKE_SOURCE_DIR}/config/devMapFile.dmap DESTINATION ${PROJECT_BINARY_DIR})

FIND_PACKAGE(Boost COMPONENTS unit_test_framework)

if(Boost_UNIT_TEST_FRAMEWORK_FOUND)
  ENABLE_TESTING()
  add_executable(testModbusConnection ${CMAKE_SOURCE_DIR}/test/testModbusConnection.C )
  target_link_libraries(testModbusConnection ${MODBUS})
  add_test(testModbusConnection testModbusConnection)
  
  add_executable(testMapping ${CMAKE_SOURCE_DIR}/test/testMapping.C )
  target_link_libraries(testMapping ${MODBUS} ${ChimeraTK-DeviceAccess_LIBRARIES})
  add_test(testMapping testMapping)
  
else()
message(STATUS "Boost unit test framework not found. No tests will be build.")
endif()

#Install the library and the executables
install(TARGETS ${PROJECT_NAME}
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
  )

#all include files go into include/PROJECT_NAME
#The exclusion of ${PROJECT_NAME} prevents the recursive installation of the files just being installed.
#The original headers are in include/*.h, the installed ones in include/PROJECT_NAME/*.h.
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include/${PROJECT_NAME}
          FILES_MATCHING PATTERN "*.h"
          PATTERN ".svn" EXCLUDE
          PATTERN "${PROJECT_NAME}" EXCLUDE
          )

#Create the config files by filling the correct variables into the template (*.cmake.in).
#All variables @VARIABLE@ are replaced with the current values, but they have to be set first....
#Only the reference header is needed. The other variables are for dependencies of this library,
#but there are none.
set(${PROJECT_NAME}_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include ${Boost_INCLUDE_DIRS}")
set(${PROJECT_NAME}_LIBRARIES "${PROJECT_NAME} ${MODBUS} ${ChimeraTK-DeviceAccess_LIBRARIES}")
set(${PROJECT_NAME}_LIBRARY_DIRS "${CMAKE_INSTALL_PREFIX}/lib ${ChimeraTK-DeviceAccess_LIBRARY_DIRS}")
set(${PROJECT_NAME}_CPPFLAGS "${ChimeraTK-DeviceAccess_CXX_FLAGS}")
set(${PROJECT_NAME}_LINKER_FLAGS "-Wl,-rpath=${CMAKE_INSTALL_PREFIX}/lib,--enable-new-dtags")
include(${CMAKE_SOURCE_DIR}/cmake/create_cmake_config_files.cmake)

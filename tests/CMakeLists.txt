find_package(Boost COMPONENTS unit_test_framework REQUIRED)

add_executable(testModbus ${library_sources} testModbus.C)
target_link_libraries(testModbus PUBLIC ChimeraTK::ChimeraTK-DeviceAccess PUBLIC ${Boost_LIBRARIES} PRIVATE ${MODBUS})
add_test(testModbus testModbus)

file(COPY dummy.map DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY devMapFile.dmap DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
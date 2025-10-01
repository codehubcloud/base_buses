set(MODBUS_SOURCES modbus.c modbus_hal.c)
set(MODBUS_HEADERS modbus.h modbus_hal.h)
add_library(modbus STATIC ${MODBUS_SOURCES} ${MODBUS_HEADERS})
target_include_directories(modbus PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include)
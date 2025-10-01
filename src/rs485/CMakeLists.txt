set(RS485_SOURCES rs485.c rs485_hal.c)
set(RS485_HEADERS rs485.h rs485_hal.h)
add_library(rs485 STATIC ${RS485_SOURCES} ${RS485_HEADERS})
target_include_directories(rs485 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include)